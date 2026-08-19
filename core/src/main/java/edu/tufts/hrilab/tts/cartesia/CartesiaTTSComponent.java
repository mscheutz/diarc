/*
 * Copyright Thinking Robots, Inc., Tufts University, and others 2026.
 */

package edu.tufts.hrilab.tts.cartesia;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import io.github.cdimascio.dotenv.Dotenv;
import io.github.cdimascio.dotenv.DotenvException;
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
import java.net.http.WebSocket;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class CartesiaTTSComponent extends DiarcComponent implements SpeechProductionInterface {

  private static final String CARTESIA_VERSION = "2026-03-01";
  private static final String WS_URL = "wss://api.cartesia.ai/tts/websocket?cartesia_version=" + CARTESIA_VERSION;
  private static final int SAMPLE_RATE = 22050;
  private static final int BITS_PER_SAMPLE = 16;
  private static final int CHANNELS = 1;
  private static final double PRIME_SECONDS = 0.04;
  private static final byte[] END_SENTINEL = new byte[0];
  private static final byte[] ERROR_SENTINEL = new byte[0];
  private static final long CHUNK_POLL_TIMEOUT_MS = 30000;
  private static final int MIN_CHUNK_CHARS = 25;
  private static final int FORCE_FLUSH_CHARS = 100;
  private static final long DISPATCHER_POLL_TIMEOUT_MS = 50;

  private String apiKey;
  private String voiceId = "3d808d23-cb09-4c39-8afd-528e209cba4f";
  private String model = "sonic-3";
  private String language = "en";
  private String speed = "normal";

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

  private final Object wsLock = new Object();
  private volatile WebSocket ws;
  private final CartesiaWsListener listener = new CartesiaWsListener();
  private volatile StreamTarget currentTarget;

  private static final class StreamTarget {
    final String contextId;
    final LinkedBlockingQueue<byte[]> chunks = new LinkedBlockingQueue<>();
    final CompletableFuture<Void> done = new CompletableFuture<>();
    volatile boolean aborted = false;
    StreamTarget(String contextId) { this.contextId = contextId; }
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
    options.add(Option.builder("voiceId").hasArg().desc("Cartesia voice UUID").build());
    options.add(Option.builder("model").hasArg().desc("Cartesia model id (default sonic-3)").build());
    options.add(Option.builder("language").hasArg().desc("language code (default en)").build());
    options.add(Option.builder("speed").hasArg().desc("speech speed: slowest|slow|normal|fast|fastest or float").build());
    options.add(Option.builder("volume").hasArg().desc("output volume 0.0-1.0").build());
    options.add(Option.builder("clipVolume").hasArg().desc("ding/cue clip volume 0.0-1.0 (default 0.4)").build());
    options.add(Option.builder("showMixers").desc("Show all mixers").build());
    options.add(Option.builder("matchMixer").hasArgs().desc("String to match for mixer. Do not quote argument if spaces needed.").build());
    options.add(Option.builder("mixer").hasArg().desc("ID of mixer for speaker output").build());
    options.add(Option.builder("muteMics").desc("Mute mics when speaking").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
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
    try {
      Dotenv dotenv = Dotenv.configure().directory(System.getProperty("user.home") + "/.diarc").filename("cartesia.env").load();
      apiKey = dotenv.get("CARTESIA_KEY");
      String envVoice = dotenv.get("CARTESIA_VOICE_ID");
      if (envVoice != null && !envVoice.isEmpty()) {
        voiceId = envVoice;
      }
    } catch (DotenvException ex) {
      log.error("[init] Cannot load ~/.diarc/cartesia.env", ex);
    }
    if (apiKey == null || apiKey.isEmpty()) {
      log.error("[init] CARTESIA_KEY missing from ~/.diarc/cartesia.env; component will not function.");
      return;
    }

    httpClient = HttpClient.newBuilder()
            .version(HttpClient.Version.HTTP_1_1)
            .connectTimeout(Duration.ofSeconds(5))
            .build();

    audioLine = openLine();
    warmAudioLine();

    executor.submit(() -> {
      try { ensureConnected(); } catch (Exception e) { log.warn("[init] ws prewarm failed: {}", e.toString()); }
    });

    log.info("[init] Cartesia TTS ready. voice={} model={} speed={} mixerID={}", voiceId, model, speed, mixerID);
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
    synchronized (wsLock) {
      if (ws != null) {
        try { ws.sendClose(WebSocket.NORMAL_CLOSURE, "shutdown"); } catch (Exception ignored) {}
        ws = null;
      }
    }
    executor.shutdownNow();
  }

  private WebSocket ensureConnected() throws IOException {
    synchronized (wsLock) {
      if (ws != null && !ws.isInputClosed() && !ws.isOutputClosed()) {
        return ws;
      }
      try {
        WebSocket newWs = httpClient.newWebSocketBuilder()
                .header("X-API-Key", apiKey)
                .connectTimeout(Duration.ofSeconds(10))
                .buildAsync(URI.create(WS_URL), listener)
                .get(10, TimeUnit.SECONDS);
        ws = newWs;
        log.info("[cartesia-ws] connected");
        return newWs;
      } catch (Exception e) {
        ws = null;
        throw new IOException("WS connect failed: " + e, e);
      }
    }
  }

  private class CartesiaWsListener implements WebSocket.Listener {
    private final StringBuilder buf = new StringBuilder();

    @Override
    public void onOpen(WebSocket webSocket) {
      webSocket.request(Long.MAX_VALUE);
    }

    @Override
    public CompletionStage<?> onText(WebSocket webSocket, CharSequence data, boolean last) {
      buf.append(data);
      if (last) {
        String msg = buf.toString();
        buf.setLength(0);
        handleMessage(msg);
      }
      return null;
    }

    @Override
    public CompletionStage<?> onClose(WebSocket webSocket, int statusCode, String reason) {
      log.info("[cartesia-ws] closed status={} reason={}", statusCode, reason);
      synchronized (wsLock) { if (ws == webSocket) ws = null; }
      StreamTarget t = currentTarget;
      if (t != null) {
        t.chunks.offer(ERROR_SENTINEL);
        t.done.complete(null);
      }
      return null;
    }

    @Override
    public void onError(WebSocket webSocket, Throwable error) {
      log.error("[cartesia-ws] error: {}", error.toString());
      synchronized (wsLock) { if (ws == webSocket) ws = null; }
      StreamTarget t = currentTarget;
      if (t != null) {
        t.chunks.offer(ERROR_SENTINEL);
        t.done.completeExceptionally(error);
      }
    }

    private void handleMessage(String msg) {
      StreamTarget t = currentTarget;
      if (t == null) return;
      try {
        JsonObject obj = JsonParser.parseString(msg).getAsJsonObject();
        JsonElement ctxEl = obj.get("context_id");
        if (ctxEl != null && !ctxEl.isJsonNull() && !t.contextId.equals(ctxEl.getAsString())) {
          return;
        }
        String type = obj.has("type") && !obj.get("type").isJsonNull() ? obj.get("type").getAsString() : "";
        if ("chunk".equals(type)) {
          JsonElement dataEl = obj.get("data");
          if (dataEl != null && !dataEl.isJsonNull()) {
            String b64 = dataEl.getAsString();
            if (!b64.isEmpty()) {
              byte[] pcm = Base64.getDecoder().decode(b64);
              t.chunks.offer(pcm);
            }
          }
          if (obj.has("done") && !obj.get("done").isJsonNull() && obj.get("done").getAsBoolean()) {
            t.chunks.offer(END_SENTINEL);
            t.done.complete(null);
          }
        } else if ("done".equals(type)) {
          t.chunks.offer(END_SENTINEL);
          t.done.complete(null);
        } else if ("error".equals(type)) {
          log.error("[cartesia-ws] error frame: {}", msg);
          t.chunks.offer(ERROR_SENTINEL);
          t.done.complete(null);
        }
      } catch (Exception e) {
        log.warn("[cartesia-ws] handleMessage parse failed: {} msg-head={}", e.toString(),
                msg.substring(0, Math.min(msg.length(), 200)));
      }
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
    if (apiKey == null || apiKey.isEmpty()) {
      log.error("[sayText] no API key; cannot synthesize");
      return false;
    }
    log.debug("[sayText] enter text_len={} wait={}", text.length(), wait);

    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("disableMicrophone"));
        svc.call(void.class);
      } catch (TRADEException e) {
        log.debug("[sayText] disableMicrophone unavailable");
      }
    }

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

    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone"));
        svc.call(void.class);
      } catch (TRADEException e) {
        log.debug("[sayText] enableMicrophone unavailable");
      }
    }
    return result;
  }

  public boolean sayStreamingText(StreamingTextSource source, boolean wait) {
    if (source == null) {
      log.warn("[sayStreamingText] null source");
      return false;
    }
    if (apiKey == null || apiKey.isEmpty()) {
      log.error("[sayStreamingText] no API key");
      return false;
    }
    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("disableMicrophone"));
        svc.call(void.class);
      } catch (TRADEException e) {
        log.debug("[sayStreamingText] disableMicrophone unavailable");
      }
    }

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

    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone"));
        svc.call(void.class);
      } catch (TRADEException e) {
        log.debug("[sayStreamingText] enableMicrophone unavailable");
      }
    }
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
      if (muteMics) {
        try {
          TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("disableMicrophone"));
          svc.call(void.class);
        } catch (TRADEException e) {
          log.debug("[pushStreamingSpeech] disableMicrophone unavailable");
        }
      }
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
    long tRequest = System.nanoTime();
    long tFirstByte = 0;
    long tFirstAudio = 0;
    long totalBytes = 0;
    log.debug("[lat] tts_request_send kind=cartesia text_len={}", text != null ? text.length() : 0);
    SourceDataLine line;

    synchronized (lineLock) {
      if (audioLine == null) {
        audioLine = openLine();
      }
      line = audioLine;
    }
    if (line == null) {
      log.error("[streamAndPlay] no audio line available");
      return false;
    }

    String contextId = UUID.randomUUID().toString();
    StreamTarget target = new StreamTarget(contextId);
    currentTarget = target;

    WebSocket conn;
    try {
      conn = ensureConnected();
    } catch (IOException e) {
      log.error("[streamAndPlay] ws connect failed", e);
      currentTarget = null;
      return false;
    }

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("diarcControlSayText").argTypes(String.class)).call(void.class,text);
    } catch (TRADEException e) {
      log.error("[sendStreamingChunk] error forwarding chunk to asr app");
    }
    String body = buildRequestBody(text, contextId);
    try {
      conn.sendText(body, true).get(5, TimeUnit.SECONDS);
    } catch (Exception e) {
      log.warn("[streamAndPlay] sendText failed, resetting WS and retrying: {}", e.toString());
      synchronized (wsLock) { ws = null; }
      try {
        conn = ensureConnected();
        conn.sendText(body, true).get(5, TimeUnit.SECONDS);
      } catch (Exception e2) {
        log.error("[streamAndPlay] sendText retry failed", e2);
        currentTarget = null;
        return false;
      }
    }

    speaking = true;
    line.start();
    int frameSize = (BITS_PER_SAMPLE / 8) * CHANNELS;
    byte[] tail = new byte[frameSize];
    int tailLen = 0;
    boolean started = false;
    long primeBytes = (long) (SAMPLE_RATE * frameSize * PRIME_SECONDS);
    ByteArrayOutputStream prime = new ByteArrayOutputStream(8192);
    try {
      while (true) {
        if (target.aborted) break;
        byte[] chunk = target.chunks.poll(CHUNK_POLL_TIMEOUT_MS, TimeUnit.MILLISECONDS);
        if (chunk == null) {
          log.error("[streamAndPlay] WS timeout waiting for chunk after {}ms", CHUNK_POLL_TIMEOUT_MS);
          break;
        }
        if (chunk == END_SENTINEL) break;
        if (chunk == ERROR_SENTINEL) { log.error("[streamAndPlay] WS error sentinel"); break; }
        int n = chunk.length;
        if (n == 0) continue;
        if (tFirstByte == 0) tFirstByte = System.nanoTime();
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
            int srcOff = (aligned >= tailLen) ? (aligned - tailLen) : 0;
            int srcLen = newTailLen;
            if (aligned < tailLen) {
              System.arraycopy(tail, aligned, tail, 0, tailLen - aligned);
              srcOff = 0;
              srcLen = newTailLen - (tailLen - aligned);
              System.arraycopy(chunk, 0, tail, tailLen - aligned, srcLen);
            } else {
              System.arraycopy(chunk, srcOff, tail, 0, srcLen);
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
      log.warn("[streamAndPlay] interrupted while polling chunks");
    } finally {
      currentTarget = null;
    }

    if (wait) line.drain();

    long tEnd = System.nanoTime();
    long ttfbMs = tFirstByte > 0 ? (tFirstByte - tRequest) / 1_000_000 : -1;
    long ttfaMs = tFirstAudio > 0 ? (tFirstAudio - tRequest) / 1_000_000 : -1;
    long totalMs = (tEnd - tRequest) / 1_000_000;
    log.debug("[lat] tts_done kind=cartesia ttfb={}ms ttfa={}ms total={}ms bytes={} text_len={}",
            ttfbMs, ttfaMs, totalMs, totalBytes, text != null ? text.length() : 0);
    speaking = false;
    return true;
  }

  private boolean streamAndPlayStreaming(StreamingTextSource source, boolean wait) {
    long tStart = System.nanoTime();
    long tFirstByte = 0;
    long tFirstAudio = 0;
    long totalBytes = 0;
    long lastChunkArrivedAtNs = 0;
    long maxGapMs = 0;
    boolean underrun = false;

    log.debug("[lat] streamtts_begin");
    log.info("[streamAndPlayStreaming] enter");
    SourceDataLine line;
    synchronized (lineLock) {
      if (audioLine == null) {
        audioLine = openLine();
      }
      line = audioLine;
    }
    if (line == null) {
      log.error("[streamAndPlayStreaming] no audio line available");
      return false;
    }

    String contextId = UUID.randomUUID().toString();
    StreamTarget target = new StreamTarget(contextId);
    currentTarget = target;

    WebSocket conn;
    try {
      conn = ensureConnected();
    } catch (IOException e) {
      log.error("[streamAndPlayStreaming] ws connect failed", e);
      currentTarget = null;
      return false;
    }

    final WebSocket dispConn = conn;
    final int[] sendsRef = new int[]{0};
    final long fTStart = tStart;
    final StreamTarget dispTarget = target;
    Thread dispatcher = new Thread(() -> dispatchLoop(source, dispConn, contextId, fTStart, sendsRef, dispTarget), "cartesia-stream-dispatch");
    dispatcher.setDaemon(true);
    dispatcher.start();

    speaking = true;
    line.start();
    int frameSize = (BITS_PER_SAMPLE / 8) * CHANNELS;
    byte[] tail = new byte[frameSize];
    int tailLen = 0;
    boolean started = false;
    long primeBytes = (long) (SAMPLE_RATE * frameSize * PRIME_SECONDS);
    ByteArrayOutputStream prime = new ByteArrayOutputStream(8192);
    try {
      while (true) {
        if (target.aborted) break;
        byte[] chunk = target.chunks.poll(CHUNK_POLL_TIMEOUT_MS, TimeUnit.MILLISECONDS);
        if (chunk == null) {
          log.error("[streamAndPlayStreaming] WS timeout waiting for chunk after {}ms", CHUNK_POLL_TIMEOUT_MS);
          break;
        }
        if (chunk == END_SENTINEL) break;
        if (chunk == ERROR_SENTINEL) { log.error("[streamAndPlayStreaming] WS error sentinel"); break; }
        int n = chunk.length;
        if (n == 0) continue;
        long nowNs = System.nanoTime();
        if (tFirstByte == 0) tFirstByte = nowNs;
        if (lastChunkArrivedAtNs != 0) {
          long gapMs = (nowNs - lastChunkArrivedAtNs) / 1_000_000;
          if (gapMs > maxGapMs) maxGapMs = gapMs;
          if (started && !underrun) {
            long elapsedSinceFirstAudioMs = (nowNs - tFirstAudio) / 1_000_000;
            long bufferedAudioMs = totalBytes * 1000L / (SAMPLE_RATE * frameSize);
            if (bufferedAudioMs + 30 < elapsedSinceFirstAudioMs) {
              underrun = true;
              log.warn("[lat] streamtts_underrun at={}ms audio_ms={} wall_ms={}",
                      (nowNs - tStart) / 1_000_000, bufferedAudioMs, elapsedSinceFirstAudioMs);
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
            log.debug("[lat] streamtts_first_audio elapsed={}ms", (tFirstAudio - tStart) / 1_000_000);
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
            int srcOff = (aligned >= tailLen) ? (aligned - tailLen) : 0;
            int srcLen = newTailLen;
            if (aligned < tailLen) {
              System.arraycopy(tail, aligned, tail, 0, tailLen - aligned);
              srcOff = 0;
              srcLen = newTailLen - (tailLen - aligned);
              System.arraycopy(chunk, 0, tail, tailLen - aligned, srcLen);
            } else {
              System.arraycopy(chunk, srcOff, tail, 0, srcLen);
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
      log.warn("[streamAndPlayStreaming] interrupted while polling chunks");
    } finally {
      currentTarget = null;
    }

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
    if (wait) line.drain();

    long tEnd = System.nanoTime();
    long ttfbMs = tFirstByte > 0 ? (tFirstByte - tStart) / 1_000_000 : -1;
    long ttfaMs = tFirstAudio > 0 ? (tFirstAudio - tStart) / 1_000_000 : -1;
    long totalMs = (tEnd - tStart) / 1_000_000;
    log.debug("[lat] streamtts_done ttfb={}ms ttfa={}ms total={}ms sends={} bytes={} max_gap={}ms underrun={}",
            ttfbMs, ttfaMs, totalMs, sendsRef[0], totalBytes, maxGapMs, underrun);
    synchronized (streamingLock) {
      if (activeBuffer == source) {
        activeBuffer = null;
        activeStreamingFuture = null;
      }
    }
    speaking = false;
    onStreamCompleted(target.aborted);
    log.info("[streamAndPlayStreaming] exit aborted={}", target.aborted);
    return true;
  }

  protected void onStreamCompleted(boolean aborted) {
    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone"));
        svc.call(void.class);
      } catch (TRADEException e) {
        log.debug("[onStreamCompleted] enableMicrophone unavailable");
      }
    }
  }

  private void dispatchLoop(StreamingTextSource source, WebSocket conn, String contextId, long tStart, int[] sendsRef, StreamTarget target) {
    StringBuilder buf = new StringBuilder();
    boolean firstChunk = true;
    try {
      while (true) {
        if (target.done.isDone()) {
          log.warn("[dispatchLoop] audio target completed early (likely WS close/error); aborting dispatch");
          return;
        }
        String fragment = source.pollFragment(DISPATCHER_POLL_TIMEOUT_MS);
        if (fragment != null) buf.append(fragment);
        while (true) {
          int split = findSplitPoint(buf, firstChunk);
          if (split <= 0) break;
          String chunkText = buf.substring(0, split);
          buf.delete(0, split);
          String trimmed = chunkText.trim();
          if (trimmed.isEmpty()) continue;
          sendStreamingChunk(conn, contextId, trimmed, true);
          sendsRef[0]++;
          log.debug("[lat] streamtts_send n={} chars={} elapsed={}ms continue=true",
                  sendsRef[0], trimmed.length(), (System.nanoTime() - tStart) / 1_000_000);
          firstChunk = false;
        }
        if (source.isComplete()) {
          String remaining = buf.toString().trim();
          sendStreamingChunk(conn, contextId, remaining, false);
          sendsRef[0]++;
          log.debug("[lat] streamtts_send n={} chars={} elapsed={}ms continue=false (final)",
                  sendsRef[0], remaining.length(), (System.nanoTime() - tStart) / 1_000_000);
          return;
        }
      }
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
      log.warn("[dispatchLoop] interrupted");
    } catch (Exception e) {
      log.error("[dispatchLoop] error", e);
    }
  }

  private void sendStreamingChunk(WebSocket conn, String contextId, String text, boolean keepContinue) {
    String body = buildStreamingRequestBody(text, contextId, keepContinue);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("diarcControlSayText").argTypes(String.class)).call(void.class,text);
    } catch (TRADEException e) {
      log.error("[sendStreamingChunk] error forwarding chunk to asr app");
    }
    try {
      conn.sendText(body, true);
    } catch (Exception e) {
      log.error("[sendStreamingChunk] sendText failed", e);
    }
  }

  private String buildStreamingRequestBody(String text, String contextId, boolean keepContinue) {
    String safe = text == null ? "" : text;
    String escaped = safe
            .replaceAll(":[a-zA-Z]\\w*", "")
            .replace("_", " ")
            .replace("\\", "\\\\")
            .replace("\"", "\\\"")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
            .replace("\t", "\\t");
    String speedJson;
    try {
      double v = Double.parseDouble(speed);
      speedJson = String.valueOf(v);
    } catch (NumberFormatException e) {
      speedJson = "\"" + speed + "\"";
    }
    return "{"
            + "\"model_id\":\"" + model + "\","
            + "\"transcript\":\"" + escaped + "\","
            + "\"voice\":{\"mode\":\"id\",\"id\":\"" + voiceId + "\"},"
            + "\"language\":\"" + language + "\","
            + "\"speed\":" + speedJson + ","
            + "\"context_id\":\"" + contextId + "\","
            // TODO(pete): would max_buffer_delay_ms>0 reduce mid-stream gaps without adding TTFA?
            + "\"max_buffer_delay_ms\":0,"
            + "\"continue\":" + keepContinue + ","
            + "\"output_format\":{\"container\":\"raw\",\"encoding\":\"pcm_s16le\",\"sample_rate\":" + SAMPLE_RATE + "}"
            + "}";
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

  private String buildRequestBody(String text, String contextId) {
    String escaped = text
            .replaceAll(":[a-zA-Z]\\w*", "")
            .replace("_", " ")
            .replace("\\", "\\\\")
            .replace("\"", "\\\"")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
            .replace("\t", "\\t");
    String speedJson;
    try {
      double v = Double.parseDouble(speed);
      speedJson = String.valueOf(v);
    } catch (NumberFormatException e) {
      speedJson = "\"" + speed + "\"";
    }
    return "{"
            + "\"model_id\":\"" + model + "\","
            + "\"transcript\":\"" + escaped + "\","
            + "\"voice\":{\"mode\":\"id\",\"id\":\"" + voiceId + "\"},"
            + "\"language\":\"" + language + "\","
            + "\"speed\":" + speedJson + ","
            + "\"context_id\":\"" + contextId + "\","
            // TODO(pete): would max_buffer_delay_ms>0 reduce mid-stream gaps without adding TTFA?
            + "\"max_buffer_delay_ms\":0,"
            + "\"continue\":false,"
            + "\"output_format\":{\"container\":\"raw\",\"encoding\":\"pcm_s16le\",\"sample_rate\":" + SAMPLE_RATE + "}"
            + "}";
  }

  @Override
  public boolean isSpeaking() {
    return speaking;
  }

  @Override
  public boolean stopUtterance() {
    // Abort any in-flight streaming session: complete the text buffer, mark the playback
    // target aborted, and push END_SENTINEL so the playback lambda breaks out of its chunk
    // poll immediately and runs onStreamCompleted. This frees the shared single-thread
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
    // Belt-and-suspenders: if onStreamCompleted is skipped, unwind enableMicrophone directly (AvaComponent clamps the refcount at 0, safe on double-enable).
    // load-bearing -- without this a flush-mid-stream leaves micMuteCount > 0 indefinitely and every subsequent ASR result is dropped.
    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone"));
        svc.call(void.class);
      } catch (TRADEException e) {
        log.debug("[stopUtterance] enableMicrophone unavailable");
      }
    }
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
    log.debug("[toggleVoiceOn] (no-op for Cartesia)");
  }

  @TRADEService
  @Action
  public void toggleVoiceOff() {
    log.debug("[toggleVoiceOff] (no-op for Cartesia)");
  }

}
