/*
 * Copyright Thinking Robots, Inc., Tufts University, and others 2026.
 */

package edu.tufts.hrilab.tts.cartesia;

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
import java.io.InputStream;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class CartesiaTTSComponent extends DiarcComponent implements SpeechProductionInterface {

  private static final String API_BASE = "https://api.cartesia.ai";
  private static final String TTS_BYTES_PATH = "/tts/bytes";
  private static final String VOICES_PATH = "/voices/?limit=1";
  private static final String CARTESIA_VERSION = "2026-03-01";
  private static final int SAMPLE_RATE = 22050;
  private static final int BITS_PER_SAMPLE = 16;
  private static final int CHANNELS = 1;
  private static final double PRIME_SECONDS = 0.04;

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
            .version(HttpClient.Version.HTTP_2)
            .connectTimeout(Duration.ofSeconds(5))
            .build();

    audioLine = openLine();
    warmAudioLine();

    prewarmAsync();

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
    executor.shutdownNow();
  }

  private void prewarmAsync() {
    HttpRequest req = HttpRequest.newBuilder()
            .uri(URI.create(API_BASE + VOICES_PATH))
            .header("X-API-Key", apiKey)
            .header("Cartesia-Version", CARTESIA_VERSION)
            .timeout(Duration.ofSeconds(10))
            .GET()
            .build();
    httpClient.sendAsync(req, HttpResponse.BodyHandlers.discarding())
            .whenComplete((resp, err) -> {
              if (err != null) {
                log.warn("[prewarm] {}", err.toString());
              } else {
                log.info("[prewarm] connection pool primed (HTTP {})", resp.statusCode());
              }
            });
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
    if (apiKey == null || apiKey.isEmpty()) {
      log.error("[sayText] no API key; cannot synthesize");
      return false;
    }

    if (muteMics) {
      try {
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("disableMicrophone").onLocalhost());
        svc.call(void.class);
      } catch (TRADEException e) {
        log.warn("[sayText] disableMicrophone failed", e);
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
        TRADEServiceInfo svc = TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone").onLocalhost());
        svc.call(void.class);
      } catch (TRADEException e) {
        log.warn("[sayText] enableMicrophone failed", e);
      }
    }
    return result;
  }

  private boolean streamAndPlay(String text, boolean wait) {
    long tRequest = System.nanoTime();
    long tFirstByte = 0;
    long tFirstAudio = 0;
    long totalBytes = 0;
    log.info("[lat] tts_request_send kind=cartesia text_len={}", text != null ? text.length() : 0);
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

    HttpRequest req = HttpRequest.newBuilder()
            .uri(URI.create(API_BASE + TTS_BYTES_PATH))
            .header("Content-Type", "application/json")
            .header("X-API-Key", apiKey)
            .header("Cartesia-Version", CARTESIA_VERSION)
            .timeout(Duration.ofSeconds(30))
            .POST(HttpRequest.BodyPublishers.ofString(buildRequestBody(text)))
            .build();

    HttpResponse<InputStream> resp;
    try {
      resp = httpClient.send(req, HttpResponse.BodyHandlers.ofInputStream());
    } catch (IOException | InterruptedException e) {
      log.error("[streamAndPlay] request failed", e);
      return false;
    }
    if (resp.statusCode() != 200) {
      String err = readStream(resp.body());
      log.error("[streamAndPlay] HTTP {} from Cartesia: {}", resp.statusCode(), err);
      return false;
    }

    speaking = true;
    line.start();
    int frameSize = (BITS_PER_SAMPLE / 8) * CHANNELS;
    byte[] tail = new byte[frameSize];
    int tailLen = 0;
    try (InputStream in = resp.body()) {
      byte[] chunk = new byte[4096];
      int n;
      boolean started = false;
      long primeBytes = (long) (SAMPLE_RATE * frameSize * PRIME_SECONDS);
      ByteArrayOutputStream prime = new ByteArrayOutputStream(8192);
      while ((n = in.read(chunk)) > 0) {
        if (tFirstByte == 0) {
          tFirstByte = System.nanoTime();
        }
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
    } catch (IOException e) {
      log.error("[streamAndPlay] error reading body", e);
      return false;
    }

    if (wait) line.drain();

    long tEnd = System.nanoTime();
    long ttfbMs = tFirstByte > 0 ? (tFirstByte - tRequest) / 1_000_000 : -1;
    long ttfaMs = tFirstAudio > 0 ? (tFirstAudio - tRequest) / 1_000_000 : -1;
    long totalMs = (tEnd - tRequest) / 1_000_000;
    log.info("[lat] tts_done kind=cartesia ttfb={}ms ttfa={}ms total={}ms bytes={} text_len={}",
            ttfbMs, ttfaMs, totalMs, totalBytes, text != null ? text.length() : 0);
    speaking = false;
    return true;
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

  private String buildRequestBody(String text) {
    String escaped = text
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
            + "\"output_format\":{\"container\":\"raw\",\"encoding\":\"pcm_s16le\",\"sample_rate\":" + SAMPLE_RATE + "}"
            + "}";
  }

  private String readStream(InputStream stream) {
    if (stream == null) return "";
    try (InputStream in = stream; ByteArrayOutputStream buf = new ByteArrayOutputStream()) {
      byte[] chunk = new byte[1024];
      int n;
      while ((n = in.read(chunk)) > 0) buf.write(chunk, 0, n);
      return buf.toString(StandardCharsets.UTF_8);
    } catch (IOException e) {
      return "";
    }
  }

  @Override
  public boolean isSpeaking() {
    return speaking;
  }

  @Override
  public boolean stopUtterance() {
    synchronized (lineLock) {
      if (audioLine != null) {
        audioLine.flush();
        audioLine.stop();
      }
    }
    return true;
  }

  @TRADEService
  public void playAudioClip(String path) {
    byte[] pcm = clipCache.computeIfAbsent(path, this::loadAndConvert);
    if (pcm == null || pcm.length == 0) {
      log.warn("[playAudioClip] no decoded audio for {}", path);
      return;
    }
    Future<?> f = executor.submit(() -> {
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
    });
    try {
      f.get();
    } catch (Exception e) {
      log.error("[playAudioClip] {} failed", path, e);
    }
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

  @TRADEService
  @Action
  public void setVoiceId(String id) {
    if (id != null && !id.isEmpty()) {
      voiceId = id;
      log.info("[setVoiceId] now using voice {}", voiceId);
    }
  }

  @TRADEService
  @Action
  public void setCartesiaVolume(String v) {
    try {
      volume = Math.max(0f, Math.min(1f, Float.parseFloat(v)));
    } catch (NumberFormatException e) {
      log.error("[setCartesiaVolume] cannot parse {}", v, e);
    }
  }
}
