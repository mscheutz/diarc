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
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

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
  private static final int MIN_CHUNK_CHARS = 25;
  private static final int FORCE_FLUSH_CHARS = 100;
  private static final long DISPATCHER_POLL_TIMEOUT_MS = 50;
  private static final long POST_DRAIN_REARM_DELAY_MS = 500;

  private SynthesisBackend.Engine backendType = SynthesisBackend.Engine.KOKORO;
  private String voiceId = "";
  private String model = "";
  private String language = "";
  private String speed = "";
  private String ttsEndpoint = "";

  private int mixerID = -1;
  private Mixer.Info mixer;
  private boolean muteMics;
  private float volume = 1.0f;
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
  private StreamingTextBuffer activeBuffer;
  private Future<Boolean> activeStreamingFuture;
  private final AtomicBoolean pendingRearm = new AtomicBoolean(false);

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

    boolean isComplete() { return producerDone && queue.isEmpty(); }
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
  public boolean sayText(Utterance utterance) {
    return sayText(utterance, true);
  }

  @Override
  public boolean sayText(Utterance utterance, boolean wait) {
    try {
      boolean local = TRADE.getAvailableService(new TRADEServiceConstraints().name("isLocalAddressee").argTypes(Symbol.class))
              .call(Boolean.class, utterance.getAddressee());
      if (!local) {
        log.info("[sayText] non-local addressee {}; not speaking: {}", utterance.getAddressee(), utterance.getWordsAsString());
        return false;
      }
    } catch (TRADEException e) {
      log.debug("[sayText] isLocalAddressee unavailable");
    }
    return sayText(utterance.getWordsAsString(), wait);
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
    Future<Boolean> future = executor.submit((Callable<Boolean>) () -> streamAndPlay(finalText, wait));

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

      String phrase = sanitizeText(text).trim();
      if (!phrase.isEmpty()) {
        session.sendPhrase(phrase);
      }
      if (!target.isFinished()) {
        session.finish();
      }

      ok = playFromTarget(target, wait, tStart, "tts");
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
    }
    return ok;
  }

  @TRADEService
  public void pushStreamingSpeech(String fragment) {
    log.debug("[pushStreamingSpeech] enter frag_len={}", fragment == null ? 0 : fragment.length());
    boolean startedNewSession = false;
    synchronized (streamingLock) {
      if (activeBuffer == null) {
        activeBuffer = new StreamingTextBuffer();
        final StreamingTextBuffer src = activeBuffer;
        activeStreamingFuture = executor.submit((Callable<Boolean>) () -> streamAndPlayStreaming(src, true));
        startedNewSession = true;
      } else if (activeBuffer.producerDone) {
        if (activeBuffer.dropLogged.compareAndSet(false, true)) {
          log.warn("[pushStreamingSpeech] dropping post-completion fragments (producer marked done); further drops at debug");
        } else {
          log.debug("[pushStreamingSpeech] dropping fragment: producer already marked done");
        }
        return;
      }
      activeBuffer.push(fragment);
    }
    if (startedNewSession) {
      pendingRearm.set(false);
      if (muteMics) disableMics();
    }
  }

  @TRADEService
  public boolean endStreamingSpeech(boolean wait) {
    log.debug("[endStreamingSpeech] enter wait={}", wait);
    StreamingTextBuffer buf;
    Future<Boolean> fut;
    synchronized (streamingLock) {
      buf = activeBuffer;
      fut = activeStreamingFuture;
    }
    if (buf != null) buf.complete();
    boolean ok = true;
    if (wait && fut != null) {
      try {
        ok = fut.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[endStreamingSpeech] error waiting on future", e);
        ok = false;
      }
    }
    synchronized (streamingLock) {
      if (activeBuffer == buf) {
        activeBuffer = null;
        activeStreamingFuture = null;
      }
    }
    return ok;
  }

  private boolean streamAndPlayStreaming(StreamingTextBuffer source, boolean wait) {
    long tStart = System.nanoTime();
    log.debug("[lat] streamtts_begin");
    log.info("[streamAndPlayStreaming] enter");

    StreamTarget target = new StreamTarget();
    boolean ok = false;
    try {
      currentTarget = target;
      if (backend == null || !backend.isReady()) {
        log.error("[streamAndPlayStreaming] backend not ready");
        return false;
      }
      SynthesisBackend.Session session = backend.openSession(target);

      final int[] sendsRef = new int[]{0};
      Thread dispatcher = new Thread(() -> dispatchLoop(source, session, target, tStart, sendsRef), "tts-dispatch");
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
      // unblocks a stalled request and deregisters the sink; no-op after clean completion
      session.abort();
      long tEnd = System.nanoTime();
      long ttfbMs = target.tFirstByte > 0 ? (target.tFirstByte - tStart) / 1_000_000 : -1;
      long ttfaMs = target.tFirstAudio > 0 ? (target.tFirstAudio - tStart) / 1_000_000 : -1;
      long totalMs = (tEnd - tStart) / 1_000_000;
      log.debug("[lat] streamtts_done ttfb={}ms ttfa={}ms total={}ms sends={} bytes={} max_gap={}ms underrun={}",
              ttfbMs, ttfaMs, totalMs, sendsRef[0], target.totalBytes, target.maxGapMs, target.underrun);
    } finally {
      synchronized (streamingLock) {
        if (activeBuffer == source) {
          activeBuffer = null;
          activeStreamingFuture = null;
        }
      }
      currentTarget = null;
      speaking = false;
      finishStream(target.aborted);
    }
    log.info("[streamAndPlayStreaming] exit aborted={}", target.aborted);
    return ok;
  }

  private void finishStream(boolean aborted) {
    if (muteMics) enableMics();
    // when the user interrupts they have already opened their own listen with the wake word
    // and the indicator is up, so skip the rearm here. The sleep would otherwise hold the
    // shared single-thread executor and delay the ding queued right behind us.
    if (pendingRearm.compareAndSet(true, false) && !aborted) {
      try {
        Thread.sleep(POST_DRAIN_REARM_DELAY_MS);
      } catch (InterruptedException ie) {
        Thread.currentThread().interrupt();
        return;
      }
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("triggerWW")).call(void.class);
      } catch (TRADEException e) {
        log.debug("[finishStream] triggerWW unavailable");
      }
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
    return text.replaceAll(":[a-zA-Z]\\w*", "").replace("_", " ");
  }

  private static int findSplitPoint(CharSequence buf, boolean isFirst) {
    int hardLimit = buf.length();
    for (int i = 0; i < hardLimit; i++) {
      char c = buf.charAt(i);
      boolean hard = (c == '.' || c == '!' || c == '?');
      boolean soft = (c == ',' || c == ';' || c == ':' || c == '\u2014');
      boolean atBoundary = (i + 1 < hardLimit && Character.isWhitespace(buf.charAt(i + 1)));
      if (hard && atBoundary) {
        if (c == '.' && (isAbbrevAt(buf, i) || isDecimalDot(buf, i))) continue;
        if (isFirst || (i + 1) >= MIN_CHUNK_CHARS) return i + 1;
      }
      if (soft && atBoundary) {
        if (isFirst || (i + 1) >= MIN_CHUNK_CHARS) return i + 1;
      }
      if ((i + 1) >= FORCE_FLUSH_CHARS && Character.isWhitespace(c)) return i + 1;
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

  private SourceDataLine openLine() {
    AudioFormat fmt = new AudioFormat(sampleRate, BITS_PER_SAMPLE, CHANNELS, true, false);
    DataLine.Info info = new DataLine.Info(SourceDataLine.class, fmt);
    try {
      SourceDataLine line = (mixer != null)
              ? (SourceDataLine) AudioSystem.getMixer(mixer).getLine(info)
              : (SourceDataLine) AudioSystem.getLine(info);
      line.open(fmt);
      if (volume < 1.0f) {
        try {
          FloatControl gain = (FloatControl) line.getControl(FloatControl.Type.MASTER_GAIN);
          gain.setValue(20f * (float) Math.log10(Math.max(volume, 0.0001f)));
        } catch (IllegalArgumentException e) {
          log.debug("[openLine] MASTER_GAIN not supported");
        }
      }
      return line;
    } catch (LineUnavailableException ex) {
      log.error("[openLine] line unavailable", ex);
      return null;
    }
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
    StreamingTextBuffer buf;
    synchronized (streamingLock) {
      buf = activeBuffer;
    }
    if (buf != null) {
      buf.complete();
    }
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
  }

  @TRADEService
  public void playAudioClip(String path) {
    log.debug("[playAudioClip] enter path={}", path);
    byte[] pcm = clipCache.computeIfAbsent(path, this::loadAndConvert);
    if (pcm == null || pcm.length == 0) {
      log.warn("[playAudioClip] no decoded audio for {}", path);
      return;
    }
    final long tSubmit = System.nanoTime();
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
}
