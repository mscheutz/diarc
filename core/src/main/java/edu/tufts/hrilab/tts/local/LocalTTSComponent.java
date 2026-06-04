/*
 * Copyright Thinking Robots, Inc., Tufts University, and others 2026.
 */

package edu.tufts.hrilab.tts.local;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.slug.common.Utterance;
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
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
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
 * Self-hosted streaming TTS backed by a local synthesis sidecar (Kokoro by default).
 * 1-1 TRADE surface with CartesiaTTSComponent: the LLM streaming path pushes phrase
 * fragments via pushStreamingSpeech/endStreamingSpeech and is backend-agnostic. The only
 * difference vs Cartesia is the producer into the audio queue -- here each phrase is POSTed
 * to the sidecar and the returned PCM is queued, where Cartesia streamed it over a WebSocket.
 */
public class LocalTTSComponent extends DiarcComponent implements SpeechProductionInterface {

  private static final int SAMPLE_RATE = 24000;   // Kokoro native; sidecar returns PCM s16le mono @ 24k
  private static final int BITS_PER_SAMPLE = 16;
  private static final int CHANNELS = 1;
  private static final double PRIME_SECONDS = 0.04;
  private static final byte[] END_SENTINEL = new byte[0];
  private static final byte[] ERROR_SENTINEL = new byte[0];
  private static final long CHUNK_POLL_TIMEOUT_MS = 30000;
  private static final int MIN_CHUNK_CHARS = 25;
  private static final int FORCE_FLUSH_CHARS = 100;
  private static final long DISPATCHER_POLL_TIMEOUT_MS = 50;

  private String endpoint = "http://127.0.0.1:5002";
  private String voice = "am_onyx";
  private String speed = "1.0";

  private int mixerID = -1;
  private Mixer.Info mixer;
  private boolean muteMics;
  private float volume = 1.0f;
  private float clipVolume = 0.4f;
  private volatile boolean speaking = false;

  private HttpClient httpClient;
  private SourceDataLine audioLine;
  private final Object lineLock = new Object();
  private final Map<String, byte[]> clipCache = new ConcurrentHashMap<>();

  private final ExecutorService executor = Executors.newSingleThreadExecutor();

  private volatile StreamTarget currentTarget;

  private static final class StreamTarget {
    final LinkedBlockingQueue<byte[]> chunks = new LinkedBlockingQueue<>();
    final CompletableFuture<Void> done = new CompletableFuture<>();
    volatile boolean aborted = false;
  }

  public interface StreamingTextSource {
    String pollFragment(long timeoutMs) throws InterruptedException;
    boolean isComplete();
  }

  public static final class StreamingTextBuffer implements StreamingTextSource {
    private final LinkedBlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private volatile boolean producerDone = false;
    final AtomicBoolean dropLogged = new AtomicBoolean(false);
    public void push(String fragment) {
      if (fragment != null && !fragment.isEmpty()) queue.offer(fragment);
    }
    public void complete() { producerDone = true; }
    @Override
    public String pollFragment(long timeoutMs) throws InterruptedException {
      return queue.poll(timeoutMs, TimeUnit.MILLISECONDS);
    }
    @Override
    public boolean isComplete() { return producerDone && queue.isEmpty(); }
  }

  private final Object streamingLock = new Object();
  private StreamingTextBuffer activeBuffer;
  private Future<Boolean> activeStreamingFuture;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("ttsEndpoint").hasArg().desc("local TTS sidecar base URL (default http://127.0.0.1:5002)").build());
    options.add(Option.builder("voiceId").hasArg().desc("sidecar voice name (default am_onyx)").build());
    options.add(Option.builder("speed").hasArg().desc("speech speed float (default 1.0)").build());
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
    if (cmdLine.hasOption("ttsEndpoint")) {
      endpoint = cmdLine.getOptionValue("ttsEndpoint");
    }
    if (cmdLine.hasOption("voiceId")) {
      voice = cmdLine.getOptionValue("voiceId");
    }
    if (cmdLine.hasOption("speed")) {
      speed = cmdLine.getOptionValue("speed");
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
    httpClient = HttpClient.newBuilder()
            .version(HttpClient.Version.HTTP_1_1)
            .connectTimeout(Duration.ofSeconds(5))
            .build();

    audioLine = openLine();
    warmAudioLine();

    try {
      HttpResponse<String> r = httpClient.send(
              HttpRequest.newBuilder(URI.create(endpoint + "/health")).timeout(Duration.ofSeconds(2)).GET().build(),
              HttpResponse.BodyHandlers.ofString());
      log.info("[init] local TTS sidecar health: {} {}", r.statusCode(), r.body());
    } catch (Exception e) {
      log.warn("[init] local TTS sidecar not reachable at {} ({}); will retry per request", endpoint, e.toString());
    }

    log.info("[init] Local TTS ready. endpoint={} voice={} speed={} muteMics={} mixerID={}", endpoint, voice, speed, muteMics, mixerID);
  }

  private void warmAudioLine() {
    if (audioLine == null) return;
    int frameSize = (BITS_PER_SAMPLE / 8) * CHANNELS;
    int silenceFrames = SAMPLE_RATE / 20;
    byte[] silence = new byte[silenceFrames * frameSize];
    audioLine.start();
    audioLine.write(silence, 0, silence.length);
    audioLine.drain();
    audioLine.flush();
  }

  @Override
  protected void shutdownComponent() {
    synchronized (lineLock) {
      if (audioLine != null) {
        try { audioLine.stop(); audioLine.close(); } catch (Exception ignored) {}
        audioLine = null;
      }
    }
    executor.shutdownNow();
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
    return sayText(utterance.getWordsAsString(), true);
  }

  @Override
  public boolean sayText(Utterance utterance, boolean wait) {
    return sayText(utterance.getWordsAsString(), wait);
  }

  @Override
  public boolean sayText(String text, boolean wait) {
    if (text == null || text.trim().isEmpty()) {
      log.debug("[sayText] empty input");
      return false;
    }
    log.info("[sayText] {}", text);
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

  public boolean sayStreamingText(StreamingTextSource source, boolean wait) {
    if (source == null) {
      log.warn("[sayStreamingText] null source");
      return false;
    }
    if (muteMics) disableMics();

    Future<Boolean> future = executor.submit((Callable<Boolean>) () -> streamAndPlayStreaming(source, wait));

    boolean result = true;
    if (wait) {
      try {
        result = future.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[sayStreamingText] error waiting on future", e);
        result = false;
      }
    }

    if (muteMics) enableMics();
    return result;
  }

  @TRADEService
  public void pushStreamingSpeech(String fragment) {
    log.debug("[pushStreamingSpeech] enter frag_len={}", fragment == null ? 0 : fragment.length());
    boolean startedNewSession = false;
    synchronized (streamingLock) {
      if (activeBuffer == null) {
        activeBuffer = new StreamingTextBuffer();
        final StreamingTextSource src = activeBuffer;
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
      onStreamStarted();
      if (muteMics) disableMics();
    }
  }

  protected void onStreamStarted() {
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

  private boolean streamAndPlay(String text, boolean wait) {
    long tStart = System.nanoTime();
    log.debug("[lat] tts_request_send kind=local text_len={}", text != null ? text.length() : 0);

    StreamTarget target = new StreamTarget();
    currentTarget = target;

    byte[] pcm = httpSynth(text);
    if (target.aborted) {
      currentTarget = null;
      return true;
    }
    if (pcm == null) {
      log.error("[streamAndPlay] synth failed");
      currentTarget = null;
      return false;
    }
    target.chunks.offer(pcm);
    target.chunks.offer(END_SENTINEL);

    boolean ok = playFromTarget(target, wait, tStart, "tts");
    currentTarget = null;
    return ok;
  }

  private boolean streamAndPlayStreaming(StreamingTextSource source, boolean wait) {
    long tStart = System.nanoTime();
    log.debug("[lat] streamtts_begin");
    log.info("[streamAndPlayStreaming] enter");

    StreamTarget target = new StreamTarget();
    currentTarget = target;

    final int[] sendsRef = new int[]{0};
    Thread dispatcher = new Thread(() -> dispatchLoop(source, target, tStart, sendsRef), "local-tts-dispatch");
    dispatcher.setDaemon(true);
    dispatcher.start();

    boolean ok = playFromTarget(target, wait, tStart, "streamtts");

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

    synchronized (streamingLock) {
      if (activeBuffer == source) {
        activeBuffer = null;
        activeStreamingFuture = null;
      }
    }
    currentTarget = null;
    onStreamCompleted(target.aborted);
    log.info("[streamAndPlayStreaming] exit aborted={} sends={}", target.aborted, sendsRef[0]);
    return ok;
  }

  protected void onStreamCompleted(boolean aborted) {
    if (muteMics) enableMics();
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
    long primeBytes = (long) (SAMPLE_RATE * frameSize * PRIME_SECONDS);
    long tFirstAudio = 0;
    long totalBytes = 0;
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
        if (chunk == ERROR_SENTINEL) { log.error("[playFromTarget] error sentinel"); break; }
        int n = chunk.length;
        if (n == 0) continue;
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
      if (!started && prime.size() > 0) {
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

    if (wait) line.drain();

    long tEnd = System.nanoTime();
    log.debug("[lat] {}_done ttfa={}ms total={}ms bytes={}", label,
            tFirstAudio > 0 ? (tFirstAudio - tStart) / 1_000_000 : -1, (tEnd - tStart) / 1_000_000, totalBytes);
    speaking = false;
    return true;
  }

  // Producer for the streaming path: split incoming fragments into phrases, synth each via the
  // sidecar, and queue the returned PCM. Runs ahead of playback (RTF < 1), keeping the queue fed.
  private void dispatchLoop(StreamingTextSource source, StreamTarget target, long tStart, int[] sendsRef) {
    StringBuilder buf = new StringBuilder();
    boolean firstChunk = true;
    try {
      while (true) {
        if (target.done.isDone() || target.aborted) {
          return;
        }
        String fragment = source.pollFragment(DISPATCHER_POLL_TIMEOUT_MS);
        if (fragment != null) buf.append(fragment);
        while (true) {
          int split = findSplitPoint(buf, firstChunk);
          if (split <= 0) break;
          String chunkText = buf.substring(0, split).trim();
          buf.delete(0, split);
          if (chunkText.isEmpty()) continue;
          if (!synthAndQueue(chunkText, target)) return;
          sendsRef[0]++;
          log.debug("[lat] streamtts_send n={} chars={} elapsed={}ms", sendsRef[0], chunkText.length(),
                  (System.nanoTime() - tStart) / 1_000_000);
          firstChunk = false;
        }
        if (source.isComplete()) {
          String remaining = buf.toString().trim();
          if (!remaining.isEmpty()) {
            if (!synthAndQueue(remaining, target)) return;
            sendsRef[0]++;
            log.debug("[lat] streamtts_send n={} chars={} elapsed={}ms (final)", sendsRef[0], remaining.length(),
                    (System.nanoTime() - tStart) / 1_000_000);
          }
          target.chunks.offer(END_SENTINEL);
          target.done.complete(null);
          return;
        }
      }
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
      log.warn("[dispatchLoop] interrupted");
    } catch (Exception e) {
      log.error("[dispatchLoop] error", e);
      target.chunks.offer(ERROR_SENTINEL);
      target.done.complete(null);
    }
  }

  private boolean synthAndQueue(String text, StreamTarget target) {
    byte[] pcm = httpSynth(text);
    if (target.aborted) return false;
    if (pcm == null) {
      target.chunks.offer(ERROR_SENTINEL);
      target.done.complete(null);
      return false;
    }
    target.chunks.offer(pcm);
    return true;
  }

  private byte[] httpSynth(String text) {
    double sp = parseSpeed(speed);
    String body = "{\"text\":\"" + jsonEscape(sanitizeText(text)) + "\",\"voice\":\"" + voice + "\",\"speed\":" + sp + "}";
    try {
      HttpRequest req = HttpRequest.newBuilder(URI.create(endpoint + "/synth"))
              .header("Content-Type", "application/json")
              .timeout(Duration.ofSeconds(30))
              .POST(HttpRequest.BodyPublishers.ofString(body))
              .build();
      HttpResponse<byte[]> resp = httpClient.send(req, HttpResponse.BodyHandlers.ofByteArray());
      if (resp.statusCode() != 200) {
        log.error("[httpSynth] sidecar status {}", resp.statusCode());
        return null;
      }
      return resp.body();
    } catch (Exception e) {
      log.error("[httpSynth] request failed: {}", e.toString());
      return null;
    }
  }

  // Accept numeric speed, or Cartesia-style words (so -voiceSpeed fast works for both backends).
  private static double parseSpeed(String s) {
    if (s == null) return 1.0;
    try { return Double.parseDouble(s); } catch (NumberFormatException ignored) {}
    switch (s.toLowerCase()) {
      case "slowest": return 0.7;
      case "slow": return 0.85;
      case "fast": return 1.15;
      case "fastest": return 1.3;
      default: return 1.0;
    }
  }

  private String sanitizeText(String text) {
    if (text == null) return "";
    return text.replaceAll(":[a-zA-Z]\\w*", "").replace("_", " ");
  }

  private static String jsonEscape(String s) {
    return s.replace("\\", "\\\\").replace("\"", "\\\"")
            .replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t");
  }

  private static int findSplitPoint(CharSequence buf, boolean isFirst) {
    int hardLimit = buf.length();
    for (int i = 0; i < hardLimit; i++) {
      char c = buf.charAt(i);
      boolean hard = (c == '.' || c == '!' || c == '?');
      boolean soft = (c == ',' || c == ';' || c == ':' || c == '—');
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
    AudioFormat fmt = new AudioFormat(SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS, true, false);
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
    if (muteMics) enableMics();
    return true;
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
    AudioFormat target = new AudioFormat(SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS, true, false);
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
              pcm.length * 1000L / (SAMPLE_RATE * (BITS_PER_SAMPLE / 8) * CHANNELS), clipVolume);
      return pcm;
    } catch (UnsupportedAudioFileException | IOException | IllegalArgumentException e) {
      log.error("[loadAndConvert] cannot decode {}", path, e);
      return null;
    }
  }

  @TRADEService
  @Action
  public void toggleVoiceOn() {
    log.debug("[toggleVoiceOn] (no-op for local TTS)");
  }

  @TRADEService
  @Action
  public void toggleVoiceOff() {
    log.debug("[toggleVoiceOff] (no-op for local TTS)");
  }

}
