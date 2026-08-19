/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.tts.cartesia;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.tufts.hrilab.tts.SynthesisBackend;
import io.github.cdimascio.dotenv.Dotenv;
import io.github.cdimascio.dotenv.DotenvException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.time.Duration;
import java.util.Base64;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;

/**
 * Cartesia cloud TTS over a shared websocket. Phrases for one utterance are chained on a
 * context_id with continue=true; the server streams PCM chunks back and signals done after
 * the final continue=false send.
 */
public class CartesiaSynthesisBackend implements SynthesisBackend {

  private static final Logger log = LoggerFactory.getLogger(CartesiaSynthesisBackend.class);

  private static final String CARTESIA_VERSION = "2026-03-01";
  private static final String WS_URL = "wss://api.cartesia.ai/tts/websocket?cartesia_version=" + CARTESIA_VERSION;
  private static final int SAMPLE_RATE = 22050;

  private String apiKey;
  private String voiceId = "3d808d23-cb09-4c39-8afd-528e209cba4f";
  private String model = "sonic-3";
  private String language = "en";
  private String speed = "normal";

  private HttpClient httpClient;
  private final Object wsLock = new Object();
  private volatile WebSocket ws;
  private final CartesiaWsListener listener = new CartesiaWsListener();
  private final Map<String, Sink> sinks = new ConcurrentHashMap<>();

  public CartesiaSynthesisBackend(String voiceId, String model, String language, String speed) {
    if (voiceId != null && !voiceId.isEmpty()) {
      this.voiceId = voiceId;
    }
    if (model != null && !model.isEmpty()) {
      this.model = model;
    }
    if (language != null && !language.isEmpty()) {
      this.language = language;
    }
    if (speed != null && !speed.isEmpty()) {
      this.speed = speed;
    }
  }

  @Override
  public void init() {
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
      log.error("[init] CARTESIA_KEY missing from ~/.diarc/cartesia.env; backend will not function.");
      return;
    }
    httpClient = HttpClient.newBuilder()
            .version(HttpClient.Version.HTTP_1_1)
            .connectTimeout(Duration.ofSeconds(5))
            .build();
    Thread prewarm = new Thread(() -> {
      try {
        ensureConnected();
      } catch (Exception e) {
        log.warn("[init] ws prewarm failed: {}", e.toString());
      }
    }, "cartesia-ws-prewarm");
    prewarm.setDaemon(true);
    prewarm.start();
  }

  @Override
  public boolean isReady() {
    return apiKey != null && !apiKey.isEmpty();
  }

  @Override
  public int getSampleRate() {
    return SAMPLE_RATE;
  }

  @Override
  public Session openSession(Sink sink) {
    String contextId = UUID.randomUUID().toString();
    sinks.put(contextId, sink);
    return new CartesiaSession(contextId, sink);
  }

  @Override
  public void shutdown() {
    synchronized (wsLock) {
      if (ws != null) {
        try { ws.sendClose(WebSocket.NORMAL_CLOSURE, "shutdown"); } catch (Exception ignored) {}
        ws = null;
      }
    }
    sinks.clear();
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

  private void failAll(String reason) {
    for (Map.Entry<String, Sink> entry : sinks.entrySet()) {
      entry.getValue().error(reason);
      sinks.remove(entry.getKey());
    }
  }

  private class CartesiaSession implements Session {
    private final String contextId;
    private final Sink sink;
    private volatile boolean aborted = false;

    CartesiaSession(String contextId, Sink sink) {
      this.contextId = contextId;
      this.sink = sink;
    }

    @Override
    public void sendPhrase(String phrase) {
      send(phrase, true);
    }

    @Override
    public void finish() {
      // the empty continue=false send is what closes the context and produces the done frame
      send("", false);
    }

    @Override
    public void abort() {
      aborted = true;
      // no wire-level cancel; the server drains the context into a deregistered sink
      sinks.remove(contextId);
    }

    private void send(String text, boolean keepContinue) {
      if (aborted || sink.isAborted()) {
        return;
      }
      String body = buildRequestBody(text, contextId, keepContinue);
      try {
        WebSocket conn = ensureConnected();
        conn.sendText(body, true).get(5, TimeUnit.SECONDS);
      } catch (Exception e) {
        log.warn("[send] sendText failed, resetting WS and retrying: {}", e.toString());
        synchronized (wsLock) {
          ws = null;
        }
        try {
          WebSocket conn = ensureConnected();
          conn.sendText(body, true).get(5, TimeUnit.SECONDS);
        } catch (Exception e2) {
          log.error("[send] sendText retry failed", e2);
          sinks.remove(contextId);
          sink.error("sendText failed: " + e2);
        }
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
      failAll("ws closed: " + statusCode + " " + reason);
      return null;
    }

    @Override
    public void onError(WebSocket webSocket, Throwable error) {
      log.error("[cartesia-ws] error: {}", error.toString());
      synchronized (wsLock) { if (ws == webSocket) ws = null; }
      failAll("ws error: " + error);
    }

    private void handleMessage(String msg) {
      try {
        JsonObject obj = JsonParser.parseString(msg).getAsJsonObject();
        JsonElement ctxEl = obj.get("context_id");
        Sink sink;
        String contextId = null;
        if (ctxEl != null && !ctxEl.isJsonNull()) {
          contextId = ctxEl.getAsString();
          sink = sinks.get(contextId);
        } else if (sinks.size() == 1) {
          // context-less frames belong to the only live session
          Map.Entry<String, Sink> only = sinks.entrySet().iterator().next();
          contextId = only.getKey();
          sink = only.getValue();
        } else {
          sink = null;
        }
        String type = obj.has("type") && !obj.get("type").isJsonNull() ? obj.get("type").getAsString() : "";
        if (sink == null) {
          if ("error".equals(type)) {
            if (contextId == null) {
              log.error("[cartesia-ws] error frame with no context_id: {}", msg);
              failAll("error frame: " + msg);
            } else {
              // a drained or aborted context erroring late must not touch the live session
              log.debug("[cartesia-ws] error frame for deregistered context {}", contextId);
            }
          }
          return;
        }
        if ("chunk".equals(type)) {
          JsonElement dataEl = obj.get("data");
          if (dataEl != null && !dataEl.isJsonNull()) {
            String b64 = dataEl.getAsString();
            if (!b64.isEmpty() && !sink.isAborted()) {
              sink.pushAudio(Base64.getDecoder().decode(b64));
            }
          }
          if (obj.has("done") && !obj.get("done").isJsonNull() && obj.get("done").getAsBoolean()) {
            sinks.remove(contextId);
            sink.end();
          }
        } else if ("done".equals(type)) {
          sinks.remove(contextId);
          sink.end();
        } else if ("error".equals(type)) {
          log.error("[cartesia-ws] error frame: {}", msg);
          sinks.remove(contextId);
          sink.error("error frame");
        }
      } catch (Exception e) {
        log.warn("[cartesia-ws] handleMessage parse failed: {} msg-head={}", e.toString(),
                msg.substring(0, Math.min(msg.length(), 200)));
      }
    }
  }

  private String buildRequestBody(String text, String contextId, boolean keepContinue) {
    String safe = text == null ? "" : text;
    String escaped = safe
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
            // TODO: would max_buffer_delay_ms>0 reduce mid-stream gaps without adding TTFA?
            + "\"max_buffer_delay_ms\":0,"
            + "\"continue\":" + keepContinue + ","
            + "\"output_format\":{\"container\":\"raw\",\"encoding\":\"pcm_s16le\",\"sample_rate\":" + SAMPLE_RATE + "}"
            + "}";
  }
}
