/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.tts;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.tts.cartesia.CartesiaSynthesisBackend;
import edu.tufts.hrilab.tts.kokoro.KokoroSynthesisBackend;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.FloatControl;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.Mixer;
import javax.sound.sampled.SourceDataLine;
import javax.sound.sampled.UnsupportedAudioFileException;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Streaming TTS behind a pluggable synthesis backend (kokoro by default). The LLM streaming
 * path pushes phrase fragments via pushStreamingSpeech/endStreamingSpeech; the backend turns
 * one phrase into PCM and nothing else.
 */
public class StreamingTTSComponent extends DiarcComponent implements SpeechProductionInterface {

  private static final int BITS_PER_SAMPLE = 16;
  private static final int CHANNELS = 1;
  private static final double PRIME_SECONDS = 0.04;
  private static final byte[] END_SENTINEL = new byte[0];
  private static final long CHUNK_POLL_TIMEOUT_MS = 30000;
  private static final int FIRST_CHUNK_MAX_CHARS = 80;
  private static final int FIRST_SOFT_MIN_CHARS = 25;
  private static final int MAX_CHUNK_CHARS = 140;
  private static final int SOFT_SPLIT_MIN_CHARS = 90;
  private static final int TAIL_MIN_CHARS = 12;
  private static final long DISPATCHER_POLL_TIMEOUT_MS = 50;
  private static final long POST_DRAIN_REARM_DELAY_MS = 500;
  private static final long NO_UID = -1L;
  private static final long STREAM_TTL_MS = 60000;
  private static final int MAX_STREAMS = 16;
  private static final int MAX_BUFFERED_CHARS = 16384;

  private SynthesisBackend.Engine backendType = SynthesisBackend.Engine.KOKORO;
  private String voiceId = "";
  private String model = "";
  private String language = "";
  private String speed = "";
  private String ttsEndpoint = "";

  private int mixerID = -1;
  private Mixer.Info mixer;
  private boolean muteMics;
  private volatile float volume = 1.0f;
  private float configuredVolume = 1.0f;
  private static final int VOICE_VOLUME_STEP = 15;
  private float clipVolume = 0.4f;
  private volatile boolean speaking = false;

  private SynthesisBackend backend;
  private int sampleRate;
  private SourceDataLine audioLine;
  private final Object lineLock = new Object();
  private final Map<String, byte[]> clipCache = new ConcurrentHashMap<>();

  private final ExecutorService executor = Executors.newSingleThreadExecutor();

  private volatile StreamTarget currentTarget;
  private final Object streamingLock = new Object();
  private final Map<Long, SpeechStream> streams = new HashMap<>();
  private final AtomicBoolean pendingRearm = new AtomicBoolean(false);
  private final ScheduledExecutorService rearmExecutor = Executors.newSingleThreadScheduledExecutor();
  private final AtomicReference<ScheduledFuture<?>> rearmTask = new AtomicReference<>();
  // goes up every time speech is stopped, so a retrigger that was set up beforehand does
  // not go off afterwards
  private final AtomicLong stopCount = new AtomicLong();
  // if something sets up a retrigger after the user has interrupted, hold onto it until we
  // actually speak again -- a reply that is still coming in should not open a listen while
  // the user is talking
  private final AtomicBoolean playedSinceStop = new AtomicBoolean(true);
  private final AtomicInteger audioJobs = new AtomicInteger();

  private static final class StreamTarget implements SynthesisBackend.Sink {
    final LinkedBlockingQueue<byte[]> chunks = new LinkedBlockingQueue<>();
    final CompletableFuture<Void> done = new CompletableFuture<>();
    volatile boolean aborted = false;
    volatile String errorReason;
    long tFirstByte;
    long tFirstAudio;
    long totalBytes;
    long maxGapMs;
    boolean underrun;

    @Override
    public void pushAudio(byte[] pcm) {
      if (pcm != null && pcm.length > 0) {
        chunks.offer(pcm);
      }
    }

    boolean isFinished() {
      return done.isDone() || aborted;
    }

    @Override
    public void end() {
      chunks.offer(END_SENTINEL);
      done.complete(null);
    }

    @Override
    public void error(String reason) {
      errorReason = reason;
      chunks.offer(END_SENTINEL);
      done.complete(null);
    }

    @Override
    public boolean isAborted() {
      return aborted;
    }
  }

  private static final class StreamingTextBuffer {
    private final LinkedBlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private volatile boolean producerDone = false;
    final AtomicBoolean dropLogged = new AtomicBoolean(false);

    void push(String fragment) {
      if (fragment != null && !fragment.isEmpty()) queue.offer(fragment);
    }

    void complete() { producerDone = true; }

    String pollFragment(long timeoutMs) throws InterruptedException {
      return queue.poll(timeoutMs, TimeUnit.MILLISECONDS);
    }

    boolean isEmpty() { return queue.isEmpty(); }

    boolean isComplete() { return producerDone && queue.isEmpty(); }
  }

  private static final class SpeechStream {
    final long uid;
    final long createdAtNs = System.nanoTime();
    final StreamingTextBuffer buffer = new StreamingTextBuffer();
    boolean released;
    boolean cancelled;
    boolean stopped;
    boolean micOff;
    int bufferedChars;
    Future<Boolean> job;
    volatile StreamTarget target;

    SpeechStream(long uid) {
      this.uid = uid;
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("ttsBackend").hasArg().desc("synthesis backend: kokoro|cartesia (default kokoro)").build());
    options.add(Option.builder("voiceId").hasArg().desc("backend voice name or UUID").build());
    options.add(Option.builder("model").hasArg().desc("Cartesia model id (default sonic-3)").build());
    options.add(Option.builder("language").hasArg().desc("language code (default en)").build());
    options.add(Option.builder("speed").hasArg().desc("speech speed: slowest|slow|normal|fast|fastest or float").build());
    options.add(Option.builder("ttsEndpoint").hasArg().desc("kokoro sidecar base URL (default http://127.0.0.1:5002)").build());
    options.add(Option.builder("volume").hasArg().desc("output volume 0.0-2.0").build());
    options.add(Option.builder("clipVolume").hasArg().desc("ding/cue clip volume 0.0-1.0 (default 0.4)").build());
    options.add(Option.builder("showMixers").desc("Show all mixers").build());
    options.add(Option.builder("matchMixer").hasArgs().desc("String to match for mixer. Do not quote argument if spaces needed.").build());
    options.add(Option.builder("mixer").hasArg().desc("ID of mixer for speaker output").build());
    options.add(Option.builder("muteMics").desc("Mute mics when speaking").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("ttsBackend")) {
      try {
        backendType = SynthesisBackend.Engine.valueOf(cmdLine.getOptionValue("ttsBackend").toUpperCase());
      } catch (IllegalArgumentException e) {
        log.error("Cannot parse ttsBackend " + cmdLine.getOptionValue("ttsBackend") + ". Defaulting to kokoro.");
      }
    }
    if (cmdLine.hasOption("voiceId")) {
      voiceId = cmdLine.getOptionValue("voiceId");
    }
    if (cmdLine.hasOption("model")) {
      model = cmdLine.getOptionValue("model");
    }
    if (cmdLine.hasOption("language")) {
      language = cmdLine.getOptionValue("language");
    }
    if (cmdLine.hasOption("speed")) {
      speed = cmdLine.getOptionValue("speed");
    }
    if (cmdLine.hasOption("ttsEndpoint")) {
      ttsEndpoint = cmdLine.getOptionValue("ttsEndpoint");
    }
    if (cmdLine.hasOption("volume")) {
      try {
        volume = Float.parseFloat(cmdLine.getOptionValue("volume"));
        if (volume < 0.0f) volume = 0.0f;
        if (volume > 2.0f) volume = 2.0f;
      } catch (NumberFormatException e) {
        log.error("[parseArgs] cannot parse volume", e);
      }
    }
    if (cmdLine.hasOption("clipVolume")) {
      try {
        clipVolume = Float.parseFloat(cmdLine.getOptionValue("clipVolume"));
        if (clipVolume < 0.0f) clipVolume = 0.0f;
        if (clipVolume > 1.0f) clipVolume = 1.0f;
      } catch (NumberFormatException e) {
        log.error("[parseArgs] cannot parse clipVolume", e);
      }
    }
    if (cmdLine.hasOption("showMixers")) {
      ConfigurableMicrophone.showMixers();
    }
    if (cmdLine.hasOption("muteMics")) {
      muteMics = true;
    }
    if (cmdLine.hasOption("mixer")) {
      mixerID = Integer.parseInt(cmdLine.getOptionValue("mixer"));
    } else if (cmdLine.hasOption("matchMixer")) {
      String matchString = String.join(" ", cmdLine.getOptionValues("matchMixer"));
      mixerID = ConfigurableMicrophone.matchMixer(matchString);
      log.debug("Matching mixer string \"" + matchString + "\"");
    }
    if (mixerID >= 0) {
      mixer = AudioSystem.getMixerInfo()[mixerID];
      log.debug("Using mixer " + mixerID);
    }
  }

  @Override
  protected void init() {
    configuredVolume = volume > 0.0f ? volume : 1.0f;
    backend = createBackend(backendType);
    backend.init();
    if (!backend.isReady()) {
      log.error("[init] {} backend not ready; component will not function.", backendType);
      return;
    }
    sampleRate = backend.getSampleRate();
    audioLine = openLine();
    warmAudioLine();
    log.info("[init] streaming TTS ready. backend={} voiceId={} speed={} muteMics={} mixerID={}",
            backendType, voiceId, speed, muteMics, mixerID);
  }

  private SynthesisBackend createBackend(SynthesisBackend.Engine engine) {
    switch (engine) {
      case CARTESIA:
        return new CartesiaSynthesisBackend(voiceId, model, language, speed);
      case KOKORO:
      default:
        return new KokoroSynthesisBackend(ttsEndpoint, voiceId, speed);
    }
  }

  private void warmAudioLine() {
    if (audioLine == null) return;
    int frameSize = (BITS_PER_SAMPLE / 8) * CHANNELS;
    int silenceFrames = sampleRate / 20;
    byte[] silence = new byte[silenceFrames * frameSize];
    audioLine.start();
    audioLine.write(silence, 0, silence.length);
    audioLine.drain();
    audioLine.flush();
  }

  @Override
  protected void shutdownComponent() {
    stopUtterance();
    rearmExecutor.shutdownNow();
    executor.shutdownNow();
    try {
      executor.awaitTermination(500, TimeUnit.MILLISECONDS);
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
    }
    if (backend != null) {
      backend.shutdown();
    }
    synchronized (lineLock) {
      if (audioLine != null) {
        try { audioLine.stop(); audioLine.close(); } catch (Exception ignored) {}
        audioLine = null;
      }
    }
  }

  private void disableMics() {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("disableMicrophone")).call(void.class);
    } catch (TRADEException e) {
      log.debug("[disableMics] disableMicrophone unavailable");
    }
  }

  private void enableMics() {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone")).call(void.class);
    } catch (TRADEException e) {
      log.debug("[enableMics] enableMicrophone unavailable");
    }
  }

  @Override
  public boolean sayText(String text) {
    return sayText(text, true);
  }

  @Override
  public boolean sayText(String text, boolean wait) {
    if (text == null || text.trim().isEmpty()) {
      log.debug("[sayText] empty input");
      return false;
    }
    log.info("[sayText] {}", text);
    if (backend == null || !backend.isReady()) {
      log.error("[sayText] backend not ready; cannot synthesize");
      return false;
    }
    log.debug("[sayText] enter text_len={} wait={}", text.length(), wait);

    if (muteMics) disableMics();

    final String finalText = text;
    audioJobs.incrementAndGet();
    Future<Boolean> future = executor.submit((Callable<Boolean>) () -> {
      try {
        return streamAndPlay(finalText, wait);
      } finally {
        audioJobs.decrementAndGet();
      }
    });

    boolean result = true;
    if (wait) {
      try {
        result = future.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[sayText] error waiting on future", e);
        result = false;
      }
    }

    if (muteMics) enableMics();
    return result;
  }

  private boolean streamAndPlay(String text, boolean wait) {
    long tStart = System.nanoTime();
    String kind = backendType.name().toLowerCase();
    log.debug("[lat] tts_request_send kind={} text_len={}", kind, text.length());

    StreamTarget target = new StreamTarget();
    SynthesisBackend.Session session = null;
    boolean ok = false;
    try {
      currentTarget = target;
      session = backend.openSession(target);

      // run the whole utterance through the same chunker the streamed path uses. sending it as
      // one phrase meant the backend had to synthesize all of it before any audio came back,
      // which is ~60ms per character of silence on a long reply
      StreamingTextBuffer buffer = new StreamingTextBuffer();
      buffer.push(text);
      buffer.complete();
      final SynthesisBackend.Session dispatchSession = session;
      final int[] sendsRef = new int[]{0};
      Thread dispatcher = new Thread(() -> dispatchLoop(buffer, dispatchSession, target, tStart, sendsRef), "tts-dispatch");
      dispatcher.setDaemon(true);
      dispatcher.start();

      ok = playFromTarget(target, wait, tStart, "tts");

      if (!target.aborted) {
        try {
          dispatcher.join(2000);
          if (dispatcher.isAlive()) {
            log.warn("[streamAndPlay] dispatcher did not exit within 2s; interrupting");
            dispatcher.interrupt();
            dispatcher.join(500);
          }
        } catch (InterruptedException ie) {
          Thread.currentThread().interrupt();
          dispatcher.interrupt();
        }
      }
      long tEnd = System.nanoTime();
      long ttfbMs = target.tFirstByte > 0 ? (target.tFirstByte - tStart) / 1_000_000 : -1;
      long ttfaMs = target.tFirstAudio > 0 ? (target.tFirstAudio - tStart) / 1_000_000 : -1;
      long totalMs = (tEnd - tStart) / 1_000_000;
      log.debug("[lat] tts_done kind={} ttfb={}ms ttfa={}ms total={}ms bytes={} text_len={}",
              kind, ttfbMs, ttfaMs, totalMs, target.totalBytes, text.length());
    } finally {
      // no-op after clean completion
      if (session != null) {
        session.abort();
      }
      currentTarget = null;
      speaking = false;
      // restart the quiet window so an arm placed mid-playback fires 500ms after the end
      if (pendingRearm.get()) {
        scheduleRearm(stopCount.get());
      }
    }
    return ok;
  }

  // TODO: is anything still calling this one? the provider passes a uid everywhere now
  @TRADEService
  public void pushStreamingSpeech(String fragment) {
    pushStreamingSpeech(fragment, NO_UID);
  }

  @TRADEService
  public void pushStreamingSpeech(String fragment, long uid) {
    log.debug("[pushStreamingSpeech] enter uid={} frag_len={}", uid, fragment == null ? 0 : fragment.length());
    synchronized (streamingLock) {
      removeStaleStreams();
      SpeechStream stream = streams.get(uid);
      if (stream == null) {
        stream = new SpeechStream(uid);
        // uid-less pushes predate buffering and play immediately, as before
        stream.released = (uid == NO_UID);
        streams.put(uid, stream);
      }
      if (stream.buffer.producerDone && !stream.cancelled) {
        // a push after end is the next stream for this uid, not a late fragment: llm
        // retries reuse the uid, and uid-less turns have no identity at all -- start
        // fresh, keeping the release state
        boolean wasReleased = stream.released;
        stream = new SpeechStream(uid);
        stream.released = wasReleased;
        streams.put(uid, stream);
      } else if (stream.cancelled) {
        if (stream.buffer.dropLogged.compareAndSet(false, true)) {
          log.warn("[pushStreamingSpeech] uid={} dropping fragments for cancelled stream; further drops at debug", uid);
        } else {
          log.debug("[pushStreamingSpeech] uid={} dropping fragment", uid);
        }
        return;
      }
      if (fragment != null && stream.bufferedChars + fragment.length() > MAX_BUFFERED_CHARS) {
        if (stream.buffer.dropLogged.compareAndSet(false, true)) {
          log.warn("[pushStreamingSpeech] uid={} buffered text over {} chars; dropping further fragments", uid, MAX_BUFFERED_CHARS);
        }
        return;
      }
      if (fragment != null) {
        stream.bufferedChars += fragment.length();
      }
      stream.buffer.push(fragment);
      if (stream.released && stream.job == null) {
        submitStream(stream);
      }
    }
  }

  private void submitStream(SpeechStream stream) {
    audioJobs.incrementAndGet();
    stream.job = executor.submit((Callable<Boolean>) () -> {
      try {
        // mute before micOff is visible to the cancel paths, so an unwind can never run
        // ahead of the mute it is undoing
        if (muteMics) {
          disableMics();
          boolean cancelled;
          synchronized (streamingLock) {
            cancelled = stream.cancelled;
            if (!cancelled) {
              stream.micOff = true;
            }
          }
          if (cancelled) {
            enableMics();
            return false;
          }
        }
        return streamAndPlayStreaming(stream, true);
      } finally {
        audioJobs.decrementAndGet();
      }
    });
  }

  @TRADEService
  public void unbufferStreamingTTSOutputForUID(long uid) {
    log.debug("[unbufferStreamingTTSOutputForUID] enter uid={}", uid);
    synchronized (streamingLock) {
      removeStaleStreams();
      SpeechStream stream = streams.get(uid);
      if (stream != null && stream.stopped) {
        // the user talked over this one, so unblocking it later should not start it up again
        log.debug("[unbufferStreamingTTSOutputForUID] uid={} was stopped, leaving it dropped", uid);
        return;
      }
      if (stream == null || stream.cancelled) {
        // release can arrive before any fragment, and a cancelled uid is reused by later
        // parse attempts -- either way a fresh released stream waits for the next push
        stream = new SpeechStream(uid);
        stream.released = true;
        streams.put(uid, stream);
        return;
      }
      stream.released = true;
      if (stream.job == null && !stream.buffer.isEmpty()) {
        submitStream(stream);
      } else if (stream.job == null && stream.buffer.producerDone) {
        // keep the release for the next stream under this uid instead of forgetting it
        SpeechStream fresh = new SpeechStream(uid);
        fresh.released = true;
        streams.put(uid, fresh);
      }
    }
  }

  @TRADEService
  public void cancelStreamingTTSOutputForUID(long uid) {
    log.debug("[cancelStreamingTTSOutputForUID] enter uid={}", uid);
    SpeechStream stream;
    StreamTarget t;
    synchronized (streamingLock) {
      removeStaleStreams();
      stream = streams.get(uid);
      if (stream == null) {
        // remember the cancel so late fragments for this uid are dropped
        SpeechStream marker = new SpeechStream(uid);
        marker.cancelled = true;
        streams.put(uid, marker);
        return;
      }
      stream.cancelled = true;
      stream.buffer.complete();
      t = stream.target;
    }
    if (t != null) {
      // same abort shape as stopUtterance, scoped to this stream's playback; the single
      // executor guarantees a stream with a live target owns the line
      t.aborted = true;
      t.done.complete(null);
      t.chunks.offer(END_SENTINEL);
      synchronized (lineLock) {
        if (audioLine != null) {
          audioLine.flush();
          audioLine.stop();
        }
      }
    }
    boolean unwindMic;
    synchronized (streamingLock) {
      unwindMic = stream.micOff;
      stream.micOff = false;
    }
    if (unwindMic) enableMics();
  }

  private void removeStaleStreams() {
    if (streams.isEmpty()) {
      return;
    }
    long now = System.nanoTime();
    Iterator<Map.Entry<Long, SpeechStream>> it = streams.entrySet().iterator();
    while (it.hasNext()) {
      SpeechStream stream = it.next().getValue();
      if (stream.job != null && !stream.job.isDone()) {
        continue;
      }
      long ageMs = (now - stream.createdAtNs) / 1_000_000;
      if (ageMs > STREAM_TTL_MS) {
        if (!stream.released && !stream.cancelled) {
          log.warn("[removeStaleStreams] uid={} never released or cancelled after {}ms; dropping", stream.uid, ageMs);
        }
        it.remove();
      }
    }
    if (streams.size() > MAX_STREAMS) {
      log.warn("[removeStaleStreams] {} live streams; something is leaking uids", streams.size());
      Iterator<Map.Entry<Long, SpeechStream>> evict = streams.entrySet().iterator();
      while (streams.size() > MAX_STREAMS && evict.hasNext()) {
        SpeechStream stream = evict.next().getValue();
        if (stream.job == null || stream.job.isDone()) {
          evict.remove();
        }
      }
    }
  }

  // TODO: same question as the uid-less push above -- still used anywhere?
  @TRADEService
  public boolean endStreamingSpeech(boolean wait) {
    return endStreamingSpeech(wait, NO_UID);
  }

  @TRADEService
  public boolean endStreamingSpeech(boolean wait, long uid) {
    log.debug("[endStreamingSpeech] enter uid={} wait={}", uid, wait);
    Future<Boolean> job;
    synchronized (streamingLock) {
      SpeechStream stream = streams.get(uid);
      if (stream == null) {
        return true;
      }
      stream.buffer.complete();
      job = stream.job;
    }
    boolean ok = true;
    if (wait && job != null) {
      try {
        ok = job.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[endStreamingSpeech] error waiting on future", e);
        ok = false;
      } catch (CancellationException e) {
        ok = false;
      }
    }
    return ok;
  }

  //TODO: Need to ensure this method isn't called before the stream is created?
  @TRADEService
  public void waitForStreamingSpeechCompletion(long uid) {
    SpeechStream stream;
    synchronized (streamingLock) {
      stream = streams.get(uid);
    }
    if (stream != null) {
      log.info("[waitForStreamingSpeechCompletion] waiting on job");
      try {
        stream.job.get();
      } catch (ExecutionException | InterruptedException e) {
        log.error("[waitForStreamingSpeechCompletion] error waiting on job", e);
      }
      log.info("[waitForStreamingSpeechCompletion] got job");
    }
  }

  private boolean streamAndPlayStreaming(SpeechStream stream, boolean wait) {
    long tStart = System.nanoTime();
    StreamTarget target = new StreamTarget();
    synchronized (streamingLock) {
      if (stream.cancelled) {
        return false;
      }
      stream.target = target;
      // set this inside the lock as well, or a stop landing between the two assignments
      // ends up missing one of them
      currentTarget = target;
    }
    log.debug("[lat] streamtts_begin");
    log.info("[streamAndPlayStreaming] enter");

    boolean ok = false;
    SynthesisBackend.Session session = null;
    try {
      if (backend == null || !backend.isReady()) {
        log.error("[streamAndPlayStreaming] backend not ready");
        return false;
      }
      session = backend.openSession(target);
      final SynthesisBackend.Session dispatchSession = session;

      final int[] sendsRef = new int[]{0};
      Thread dispatcher = new Thread(() -> dispatchLoop(stream.buffer, dispatchSession, target, tStart, sendsRef), "tts-dispatch");
      dispatcher.setDaemon(true);
      dispatcher.start();

      ok = playFromTarget(target, wait, tStart, "streamtts");

      // on abort skip the join: the dispatcher is a daemon that may be parked inside a synth
      // request, and waiting here would hold the shared single-thread executor and delay the
      // interrupt ding queued right behind us
      if (!target.aborted) {
        try {
          dispatcher.join(2000);
          if (dispatcher.isAlive()) {
            log.warn("[streamAndPlayStreaming] dispatcher did not exit within 2s; interrupting");
            dispatcher.interrupt();
            dispatcher.join(500);
          }
        } catch (InterruptedException ie) {
          Thread.currentThread().interrupt();
          dispatcher.interrupt();
        }
      }
      long tEnd = System.nanoTime();
      long ttfbMs = target.tFirstByte > 0 ? (target.tFirstByte - tStart) / 1_000_000 : -1;
      long ttfaMs = target.tFirstAudio > 0 ? (target.tFirstAudio - tStart) / 1_000_000 : -1;
      long totalMs = (tEnd - tStart) / 1_000_000;
      log.debug("[lat] streamtts_done ttfb={}ms ttfa={}ms total={}ms sends={} bytes={} max_gap={}ms underrun={}",
              ttfbMs, ttfaMs, totalMs, sendsRef[0], target.totalBytes, target.maxGapMs, target.underrun);
    } finally {
      // unblocks a stalled request and deregisters the sink; no-op after clean completion
      if (session != null) {
        session.abort();
      }
      currentTarget = null;
      speaking = false;
      finishStream(stream, target.aborted);
    }
    log.info("[streamAndPlayStreaming] exit aborted={}", target.aborted);
    return ok;
  }

  private void finishStream(SpeechStream stream, boolean aborted) {
    boolean unwindMic;
    boolean wasStopped;
    synchronized (streamingLock) {
      // a later parse attempt may have replaced this uid's entry; only swap out our own
      if (streams.get(stream.uid) == stream) {
        SpeechStream next = new SpeechStream(stream.uid);
        if (aborted) {
          // fragments that show up after this turn was cut off should keep getting dropped
          next.cancelled = true;
          next.stopped = stream.stopped;
        } else {
          // the next stream under this uid keeps the release, like a push after end
          next.released = stream.released;
        }
        streams.put(stream.uid, next);
      }
      unwindMic = stream.micOff;
      stream.micOff = false;
      wasStopped = stream.stopped;
    }
    if (unwindMic) enableMics();
    // when the user interrupts, they have already opened their own listen with the wake
    // word, so drop the retrigger rather than opening a second one. Cancelling a single
    // utterance is a different case and leaves it alone.
    if (aborted) {
      if (wasStopped) {
        cancelRearm();
        pendingRearm.set(false);
      }
    } else if (pendingRearm.get()) {
      scheduleRearm(stopCount.get());
    }
  }

  // the delay runs on its own thread; the audio executor must stay free for the interrupt
  // ding queued right behind a finished utterance
  private void scheduleRearm(long armedAtStop) {
    if (rearmExecutor.isShutdown()) {
      return;
    }
    ScheduledFuture<?> next = rearmExecutor.schedule(() -> fireRearm(armedAtStop), POST_DRAIN_REARM_DELAY_MS, TimeUnit.MILLISECONDS);
    ScheduledFuture<?> prev = rearmTask.getAndSet(next);
    if (prev != null) {
      prev.cancel(false);
    }
  }

  private void fireRearm(long armedAtStop) {
    if (stopCount.get() != armedAtStop) {
      return;
    }
    if (speaking || audioJobs.get() > 0) {
      // still talking or speech is queued; try again after the playback that finishes
      // last has had its quiet window
      scheduleRearm(armedAtStop);
      return;
    }
    if (!pendingRearm.compareAndSet(true, false)) {
      return;
    }
//    try {
//      TRADE.getAvailableService(new TRADEServiceConstraints().name("triggerWW")).call(void.class);
//    } catch (TRADEException e) {
//      log.debug("[fireRearm] triggerWW unavailable");
//    }
  }

  private void cancelRearm() {
    ScheduledFuture<?> t = rearmTask.getAndSet(null);
    if (t != null) {
      t.cancel(false);
    }
  }

  // Producer for the streaming path: split incoming fragments into phrases and send each to
  // the backend. Runs ahead of playback (RTF < 1), keeping the audio queue fed.
  private void dispatchLoop(StreamingTextBuffer source, SynthesisBackend.Session session, StreamTarget target, long tStart, int[] sendsRef) {
    StringBuilder buf = new StringBuilder();
    boolean firstChunk = true;
    try {
      while (true) {
        if (target.done.isDone()) {
          log.warn("[dispatchLoop] audio target completed early (likely backend error); aborting dispatch");
          return;
        }
        if (target.aborted) {
          return;
        }
        String fragment = source.pollFragment(DISPATCHER_POLL_TIMEOUT_MS);
        if (fragment != null) buf.append(fragment);
        while (true) {
          int split = findSplitPoint(buf, firstChunk);
          if (split <= 0) break;
          String chunkText = buf.substring(0, split);
          buf.delete(0, split);
          String phrase = sanitizeText(chunkText).trim();
          if (phrase.isEmpty()) continue;
          if (target.isFinished()) return;
          session.sendPhrase(phrase);
          sendsRef[0]++;
          log.debug("[lat] streamtts_send n={} chars={} elapsed={}ms",
                  sendsRef[0], phrase.length(), (System.nanoTime() - tStart) / 1_000_000);
          firstChunk = false;
        }
        if (source.isComplete()) {
          String remaining = sanitizeText(buf.toString()).trim();
          if (!remaining.isEmpty() && !target.isFinished()) {
            session.sendPhrase(remaining);
            sendsRef[0]++;
            log.debug("[lat] streamtts_send n={} chars={} elapsed={}ms (final)",
                    sendsRef[0], remaining.length(), (System.nanoTime() - tStart) / 1_000_000);
          }
          if (!target.isFinished()) {
            session.finish();
          }
          return;
        }
      }
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
      log.warn("[dispatchLoop] interrupted");
    } catch (Exception e) {
      log.error("[dispatchLoop] error", e);
      target.error(e.toString());
    }
  }

  // Audio consumer: drain target.chunks into the persistent line with a prime buffer and
  // frame-aligned writes (PCM byte[] chunks are not guaranteed frame-aligned; carry the tail).
  private boolean playFromTarget(StreamTarget target, boolean wait, long tStart, String label) {
    SourceDataLine line;
    synchronized (lineLock) {
      if (audioLine == null) {
        audioLine = openLine();
      }
      line = audioLine;
    }
    if (line == null) {
      log.error("[playFromTarget] no audio line available");
      return false;
    }

    speaking = true;
    playedSinceStop.set(true);
    line.start();
    int frameSize = (BITS_PER_SAMPLE / 8) * CHANNELS;
    byte[] tail = new byte[frameSize];
    int tailLen = 0;
    boolean started = false;
    long primeBytes = (long) (sampleRate * frameSize * PRIME_SECONDS);
    long tFirstByte = 0;
    long tFirstAudio = 0;
    long totalBytes = 0;
    long lastChunkArrivedAtNs = 0;
    long maxGapMs = 0;
    boolean underrun = false;
    ByteArrayOutputStream prime = new ByteArrayOutputStream(8192);
    try {
      while (true) {
        if (target.aborted) break;
        byte[] chunk = target.chunks.poll(CHUNK_POLL_TIMEOUT_MS, TimeUnit.MILLISECONDS);
        if (chunk == null) {
          log.error("[playFromTarget] timeout waiting for chunk after {}ms", CHUNK_POLL_TIMEOUT_MS);
          break;
        }
        if (chunk == END_SENTINEL) break;
        int n = chunk.length;
        if (n == 0) continue;
        long nowNs = System.nanoTime();
        if (tFirstByte == 0) tFirstByte = nowNs;
        if (lastChunkArrivedAtNs != 0) {
          long gapMs = (nowNs - lastChunkArrivedAtNs) / 1_000_000;
          if (gapMs > maxGapMs) maxGapMs = gapMs;
          if (started && !underrun) {
            long elapsedSinceFirstAudioMs = (nowNs - tFirstAudio) / 1_000_000;
            long bufferedAudioMs = totalBytes * 1000L / (sampleRate * frameSize);
            if (bufferedAudioMs + 30 < elapsedSinceFirstAudioMs) {
              underrun = true;
              log.warn("[lat] {}_underrun at={}ms audio_ms={} wall_ms={}",
                      label, (nowNs - tStart) / 1_000_000, bufferedAudioMs, elapsedSinceFirstAudioMs);
            }
          }
        }
        lastChunkArrivedAtNs = nowNs;
        totalBytes += n;
        if (!started) {
          prime.write(chunk, 0, n);
          if (prime.size() >= primeBytes) {
            byte[] head = prime.toByteArray();
            int aligned = (head.length / frameSize) * frameSize;
            int leftover = head.length - aligned;
            if (volume > 1.0f && aligned > 0) applySoftGain(head, 0, aligned);
            if (aligned > 0) line.write(head, 0, aligned);
            if (leftover > 0) {
              System.arraycopy(head, aligned, tail, 0, leftover);
              tailLen = leftover;
            }
            tFirstAudio = System.nanoTime();
            log.debug("[lat] {}_first_audio elapsed={}ms", label, (tFirstAudio - tStart) / 1_000_000);
            started = true;
          }
        } else {
          int total = tailLen + n;
          int aligned = (total / frameSize) * frameSize;
          int newTailLen = total - aligned;
          if (aligned > 0) {
            if (tailLen == 0) {
              if (volume > 1.0f) applySoftGain(chunk, 0, aligned);
              line.write(chunk, 0, aligned);
            } else {
              byte[] merged = new byte[aligned];
              System.arraycopy(tail, 0, merged, 0, tailLen);
              System.arraycopy(chunk, 0, merged, tailLen, aligned - tailLen);
              if (volume > 1.0f) applySoftGain(merged, 0, aligned);
              line.write(merged, 0, aligned);
            }
          }
          if (newTailLen > 0) {
            if (aligned < tailLen) {
              System.arraycopy(tail, aligned, tail, 0, tailLen - aligned);
              int srcLen = newTailLen - (tailLen - aligned);
              System.arraycopy(chunk, 0, tail, tailLen - aligned, srcLen);
            } else {
              int srcOff = aligned - tailLen;
              System.arraycopy(chunk, srcOff, tail, 0, newTailLen);
            }
          }
          tailLen = newTailLen;
        }
      }
      // if the user interrupts before the prime buffer fills, don't play what we primed
      if (!started && prime.size() > 0 && !target.aborted) {
        byte[] head = prime.toByteArray();
        int aligned = (head.length / frameSize) * frameSize;
        if (volume > 1.0f && aligned > 0) applySoftGain(head, 0, aligned);
        if (aligned > 0) line.write(head, 0, aligned);
        tFirstAudio = System.nanoTime();
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      log.warn("[playFromTarget] interrupted while polling chunks");
    }

    if (target.errorReason != null) {
      log.error("[playFromTarget] backend error: {}", target.errorReason);
    }
    if (wait) line.drain();

    target.tFirstByte = tFirstByte;
    target.tFirstAudio = tFirstAudio;
    target.totalBytes = totalBytes;
    target.maxGapMs = maxGapMs;
    target.underrun = underrun;
    speaking = false;
    return target.errorReason == null;
  }

  private String sanitizeText(String text) {
    if (text == null) return "";
    return text.replaceAll(":[a-zA-Z]\\w*", "").replace("_", " ").replace("\"", "")
            .replace("\u201c", "").replace("\u201d", "").replace("\u2018", "'").replace("\u2019", "'");
  }

  // Kokoro plans intonation per request, so a comma cut puts a sentence-final contour mid-sentence.
  // First piece: the first sentence when it ends within FIRST_CHUNK_MAX_CHARS, else the first clause
  // boundary past FIRST_SOFT_MIN_CHARS, else whitespace -- quick first audio without a bare "Sure,".
  // Later pieces: sentence ends only; a sentence running past MAX_CHUNK_CHARS is cut at the last clause
  // boundary after SOFT_SPLIT_MIN_CHARS so one long sentence cannot stall playback. A sentence end is
  // not taken while fewer than TAIL_MIN_CHARS follow it: a short tail like "Okay." folds into the
  // previous piece once the text is complete instead of getting its own contour.
  private static int findSplitPoint(CharSequence buf, boolean isFirst) {
    int len = buf.length();
    int limit = isFirst ? FIRST_CHUNK_MAX_CHARS : MAX_CHUNK_CHARS;
    int softMin = isFirst ? FIRST_SOFT_MIN_CHARS : SOFT_SPLIT_MIN_CHARS;
    int softCut = -1;
    for (int i = 0; i < len; i++) {
      char c = buf.charAt(i);
      int end = i + 1;
      boolean atBoundary = end < len && Character.isWhitespace(buf.charAt(end));
      if (atBoundary && (c == '.' || c == '!' || c == '?')) {
        if (c == '.' && (isAbbrevAt(buf, i) || isDecimalDot(buf, i))) continue;
        if (isFirst || len - end >= TAIL_MIN_CHARS) return end;
        return -1;
      }
      if (atBoundary && (c == ',' || c == ';' || c == ':' || c == '\u2014') && end >= softMin) {
        if (!isFirst || softCut < 0) softCut = end;
      }
      if (end >= limit) {
        if (softCut > 0) return softCut;
        if (Character.isWhitespace(c)) return end;
      }
    }
    return -1;
  }

  private static boolean isAbbrevAt(CharSequence s, int dotIdx) {
    int j = dotIdx - 1;
    while (j >= 0 && Character.isLetter(s.charAt(j))) j--;
    if (j == dotIdx - 1) return false;
    String word = s.subSequence(j + 1, dotIdx).toString().toLowerCase();
    switch (word) {
      case "mr": case "mrs": case "ms": case "dr": case "st":
      case "jr": case "sr": case "vs": case "etc":
        return true;
      default: return false;
    }
  }

  private static boolean isDecimalDot(CharSequence s, int dotIdx) {
    return dotIdx > 0 && dotIdx + 1 < s.length()
            && Character.isDigit(s.charAt(dotIdx - 1))
            && Character.isDigit(s.charAt(dotIdx + 1));
  }

  private void applySoftGain(byte[] pcm, int off, int len) {
    for (int i = off; i + 1 < off + len; i += 2) {
      short s = (short) ((pcm[i] & 0xff) | (pcm[i + 1] << 8));
      int scaled = Math.round(s * volume);
      if (scaled > Short.MAX_VALUE) scaled = Short.MAX_VALUE;
      if (scaled < Short.MIN_VALUE) scaled = Short.MIN_VALUE;
      pcm[i] = (byte) (scaled & 0xff);
      pcm[i + 1] = (byte) ((scaled >> 8) & 0xff);
    }
  }

  // below 1.0 the line's gain control does the attenuation; above it applySoftGain scales the samples
  private void applyLineGain(SourceDataLine line) {
    try {
      FloatControl gain = (FloatControl) line.getControl(FloatControl.Type.MASTER_GAIN);
      gain.setValue(volume < 1.0f ? 20f * (float) Math.log10(Math.max(volume, 0.0001f)) : 0f);
    } catch (IllegalArgumentException e) {
      log.debug("[applyLineGain] MASTER_GAIN not supported");
    }
  }

  private SourceDataLine openLine() {
    AudioFormat fmt = new AudioFormat(sampleRate, BITS_PER_SAMPLE, CHANNELS, true, false);
    DataLine.Info info = new DataLine.Info(SourceDataLine.class, fmt);
    try {
      SourceDataLine line = (mixer != null)
              ? (SourceDataLine) AudioSystem.getMixer(mixer).getLine(info)
              : (SourceDataLine) AudioSystem.getLine(info);
      line.open(fmt);
      applyLineGain(line);
      return line;
    } catch (LineUnavailableException ex) {
      log.error("[openLine] line unavailable", ex);
      return null;
    }
  }

  @TRADEService
  public boolean isStreamingSpeech(long uid) {
    SpeechStream stream = null;
    if (speaking) {
      synchronized (streamingLock) {
        removeStaleStreams();
        stream = streams.get(uid);
      }
    }
    return stream != null;
  }

  @Override
  public boolean isSpeaking() {
    return speaking;
  }

  @Override
  public boolean stopUtterance() {
    // Abort any in-flight streaming session: complete the text buffer, mark the playback
    // target aborted, and push END_SENTINEL so the playback lambda breaks out of its chunk
    // poll immediately and runs finishStream. This frees the shared single-thread
    // executor for the interrupt ding queued right behind us.
    // load-bearing -- without the abort the lambda drains the talked-over response to its
    // natural END_SENTINEL (~1s) and the ding can't play until then.
    int micUnwinds = 0;
    synchronized (streamingLock) {
      // the user talked over the robot: everything composed for this turn is stale,
      // buffered streams included
      for (SpeechStream stream : streams.values()) {
        stream.cancelled = true;
        stream.stopped = true;
        stream.buffer.complete();
        if (stream.micOff && stream.target == null) {
          stream.micOff = false;
          micUnwinds++;
        }
      }
      streams.remove(NO_UID);
    }
    for (int i = 0; i < micUnwinds; i++) {
      enableMics();
    }
    // the user is already talking; a queued rearm must not fire into their turn, and any
    // arm placed before this stop is dead even if its timer is already in flight
    stopCount.incrementAndGet();
    playedSinceStop.set(false);
    cancelRearm();
    pendingRearm.set(false);
    StreamTarget t = currentTarget;
    log.info("[stopUtterance] enter streamingActive={}", t != null);
    if (t != null) {
      t.aborted = true;
      t.done.complete(null);
      t.chunks.offer(END_SENTINEL);
    }
    synchronized (lineLock) {
      if (audioLine != null) {
        audioLine.flush();
        audioLine.stop();
      }
    }
    // Belt-and-suspenders: if finishStream is skipped, unwind enableMicrophone directly (AvaComponent clamps the refcount at 0, safe on double-enable).
    // load-bearing -- without this a flush-mid-stream leaves micMuteCount > 0 indefinitely and every subsequent ASR result is dropped.
    if (muteMics) enableMics();
    return true;
  }

  @TRADEService
  @Action
  public void armPostStreamRearm() {
    log.debug("[armPostStreamRearm] set");
    pendingRearm.set(true);
    // setting this again restarts the delay, so a filler and the answer behind it end up
    // as one retrigger after the last of the speech. If the user interrupted, wait until
    // we have actually played something again before setting up the timer.
    if (playedSinceStop.get()) {
      scheduleRearm(stopCount.get());
    }
  }

  @TRADEService
  public void playAudioClip(String path) {
    log.debug("[playAudioClip] enter path={}", path);
    if (sampleRate == 0) {
      log.warn("[playAudioClip] backend never initialized; dropping {}", path);
      return;
    }
    byte[] pcm = clipCache.computeIfAbsent(path, this::loadAndConvert);
    if (pcm == null || pcm.length == 0) {
      log.warn("[playAudioClip] no decoded audio for {}", path);
      return;
    }
    final long tSubmit = System.nanoTime();
    audioJobs.incrementAndGet();
    executor.submit(() -> {
      try {
        long tRun = System.nanoTime();
        SourceDataLine line;
        synchronized (lineLock) {
          if (audioLine == null) {
            audioLine = openLine();
          }
          line = audioLine;
        }
        if (line == null) return;
        line.start();
        line.write(pcm, 0, pcm.length);
        line.drain();
        log.info("[playAudioClip] {} queue_wait={}ms play={}ms", path,
                (tRun - tSubmit) / 1_000_000, (System.nanoTime() - tRun) / 1_000_000);
      } catch (Exception e) {
        log.error("[playAudioClip] {} failed", path, e);
      } finally {
        audioJobs.decrementAndGet();
      }
    });
  }

  private byte[] loadAndConvert(String path) {
    AudioFormat target = new AudioFormat(sampleRate, BITS_PER_SAMPLE, CHANNELS, true, false);
    try (AudioInputStream raw = AudioSystem.getAudioInputStream(new File(path));
         AudioInputStream conv = AudioSystem.getAudioInputStream(target, raw);
         ByteArrayOutputStream out = new ByteArrayOutputStream(8192)) {
      byte[] buf = new byte[4096];
      int n;
      while ((n = conv.read(buf)) > 0) out.write(buf, 0, n);
      byte[] pcm = out.toByteArray();
      if (clipVolume < 1.0f) {
        for (int i = 0; i + 1 < pcm.length; i += 2) {
          short s = (short) ((pcm[i] & 0xff) | (pcm[i + 1] << 8));
          int scaled = Math.round(s * clipVolume);
          if (scaled > Short.MAX_VALUE) scaled = Short.MAX_VALUE;
          if (scaled < Short.MIN_VALUE) scaled = Short.MIN_VALUE;
          pcm[i] = (byte) (scaled & 0xff);
          pcm[i + 1] = (byte) ((scaled >> 8) & 0xff);
        }
      }
      log.info("[loadAndConvert] cached {} -> {} bytes ({} ms audio, gain {})", path, pcm.length,
              pcm.length * 1000L / (sampleRate * (BITS_PER_SAMPLE / 8) * CHANNELS), clipVolume);
      return pcm;
    } catch (UnsupportedAudioFileException | IOException | IllegalArgumentException e) {
      log.error("[loadAndConvert] cannot decode {}", path, e);
      return null;
    }
  }

  @TRADEService
  @Action
  public void toggleVoiceOn() {
    log.debug("[toggleVoiceOn] (no-op for streaming TTS)");
  }

  @TRADEService
  @Action
  public void toggleVoiceOff() {
    log.debug("[toggleVoiceOff] (no-op for streaming TTS)");
  }

  @TRADEService
  public boolean setVoiceVolume(Symbol volumeSymbol) {
    try {
      return setVoiceVolume((int)Double.parseDouble(volumeSymbol.getName()));
    } catch (NumberFormatException e) {
      log.error("[setVoiceVolume] supplied volumeSymbol not parseable as int: {}", volumeSymbol, e);
      return false;
    }
  }

  // percent is relative to the launch -volume, so a site tuned above 1.0 keeps its boost as 100
  @TRADEService
  public boolean setVoiceVolume(int percent) {
    if (percent < 0 || percent > 100) {
      log.error("[setVoiceVolume] supplied volume is out of bounds [0,100]: {}", percent);
      return false;
    }
    volume = configuredVolume * percent / 100f;
    synchronized (lineLock) {
      if (audioLine != null) {
        applyLineGain(audioLine);
      }
    }
    return true;
  }

  @TRADEService
  public boolean lowerVoiceVolume() {
    return setVoiceVolume(Math.max(0, getVoiceVolume() - VOICE_VOLUME_STEP));
  }

  @TRADEService
  public boolean raiseVoiceVolume() {
    return setVoiceVolume(Math.min(100, getVoiceVolume() + VOICE_VOLUME_STEP));
  }

  @TRADEService
  public int getVoiceVolume() {
    return Math.round(volume / configuredVolume * 100f);
  }
}
