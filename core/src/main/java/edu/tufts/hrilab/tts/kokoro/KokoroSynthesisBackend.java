/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.tts.kokoro;

import edu.tufts.hrilab.tts.SynthesisBackend;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

/**
 * Self-hosted synthesis via a local Kokoro sidecar. Each phrase is one POST to /synth
 * returning complete PCM s16le mono at 24000 Hz.
 */
public class KokoroSynthesisBackend implements SynthesisBackend {

  private static final Logger log = LoggerFactory.getLogger(KokoroSynthesisBackend.class);

  private static final int SAMPLE_RATE = 24000;
  private static final long SYNTH_TIMEOUT_SECONDS = 30;

  private String endpoint = "http://127.0.0.1:5002";
  private String voice = "";
  private double speed = 1.0;

  private HttpClient httpClient;

  public KokoroSynthesisBackend(String endpoint, String voice, String speed) {
    if (endpoint != null && !endpoint.isEmpty()) {
      this.endpoint = endpoint;
    }
    if (voice != null && !voice.isEmpty()) {
      this.voice = voice;
    }
    if (speed != null && !speed.isEmpty()) {
      this.speed = parseSpeed(speed);
    }
  }

  @Override
  public void init() {
    httpClient = HttpClient.newBuilder()
            .version(HttpClient.Version.HTTP_1_1)
            .connectTimeout(Duration.ofSeconds(5))
            .build();
    try {
      HttpResponse<String> r = httpClient.send(
              HttpRequest.newBuilder(URI.create(endpoint + "/health")).timeout(Duration.ofSeconds(2)).GET().build(),
              HttpResponse.BodyHandlers.ofString());
      log.info("[init] kokoro sidecar health: {} {}", r.statusCode(), r.body());
    } catch (Exception e) {
      log.warn("[init] kokoro sidecar not reachable at {} ({}); will retry per request", endpoint, e.toString());
    }
  }

  @Override
  public boolean isReady() {
    return httpClient != null;
  }

  @Override
  public int getSampleRate() {
    return SAMPLE_RATE;
  }

  @Override
  public Session openSession(Sink sink) {
    return new KokoroSession(sink);
  }

  @Override
  public void shutdown() {
  }

  private class KokoroSession implements Session {
    private final Sink sink;
    private volatile boolean aborted = false;
    private volatile CompletableFuture<HttpResponse<byte[]>> inFlight;

    KokoroSession(Sink sink) {
      this.sink = sink;
    }

    @Override
    public void sendPhrase(String phrase) {
      if (aborted) {
        return;
      }
      // no voice configured: leave the field out so the sidecar's own default applies
      String voiceField = voice.isEmpty() ? "" : ",\"voice\":\"" + voice + "\"";
      String body = "{\"text\":\"" + jsonEscape(phrase) + "\"" + voiceField + ",\"speed\":" + speed + "}";
      HttpRequest req = HttpRequest.newBuilder(URI.create(endpoint + "/synth"))
              .header("Content-Type", "application/json")
              .timeout(Duration.ofSeconds(SYNTH_TIMEOUT_SECONDS))
              .POST(HttpRequest.BodyPublishers.ofString(body))
              .build();
      // sendAsync so abort() can cancel a request the sidecar is sitting on; a plain send
      // holds the dispatch thread for the whole timeout, which delays the ding we play
      // when the user interrupts.
      CompletableFuture<HttpResponse<byte[]>> future = httpClient.sendAsync(req, HttpResponse.BodyHandlers.ofByteArray());
      inFlight = future;
      // abort() may have run between the send and the publish, in which case it saw a null inFlight
      if (aborted) {
        future.cancel(true);
      }
      HttpResponse<byte[]> resp;
      try {
        resp = future.get(SYNTH_TIMEOUT_SECONDS, TimeUnit.SECONDS);
      } catch (InterruptedException ie) {
        Thread.currentThread().interrupt();
        return;
      } catch (Exception e) {
        if (!aborted && !sink.isAborted()) {
          sink.error("synth request failed: " + e);
        }
        return;
      } finally {
        inFlight = null;
      }
      if (aborted || sink.isAborted()) {
        return;
      }
      if (resp.statusCode() != 200) {
        sink.error("sidecar status " + resp.statusCode());
        return;
      }
      sink.pushAudio(resp.body());
    }

    @Override
    public void finish() {
      if (!aborted && !sink.isAborted()) {
        sink.end();
      }
    }

    @Override
    public void abort() {
      aborted = true;
      CompletableFuture<HttpResponse<byte[]>> future = inFlight;
      if (future != null) {
        future.cancel(true);
      }
    }
  }

  // Accept numeric speed, or Cartesia-style words (so -speed fast works for both backends).
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

  private static String jsonEscape(String s) {
    return s.replace("\\", "\\\\").replace("\"", "\\\"")
            .replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t");
  }
}
