/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.asr;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.util.Audio;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.java_websocket.WebSocket;
import org.java_websocket.drafts.Draft;
import org.java_websocket.drafts.Draft_6455;
import org.java_websocket.framing.CloseFrame;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

public class ASRWebsocketServer extends WebSocketServer {

  private Symbol speaker;
  private Symbol addressee;

  private int port = 8080;
  private Map<Integer, float[]> audioData = new HashMap<>();
  private Map<Integer, Integer> audioCursor = new HashMap<>();
  private Map<Integer, Integer> audioSampleRate = new HashMap<>();
  Gson gson = new Gson();
  static private Logger log = LoggerFactory.getLogger(ASRWebsocketServer.class);

  private class StartMessageConfig {
    public int sample_rate;
  }

  private class StartMessage {
    public StartMessageConfig config;
  }

  private class EndMessage {
    public int eof;
  }

  private class PartialMessage {
    public String partial;
    public PartialMessage (String partial) {
      this.partial = partial;
    }
  }

  private class TextMessage {
    public String text;
    public TextMessage(String text) {
      this.text = text;
    }
  }

  public ASRWebsocketServer(int port) throws UnknownHostException {
    super(new InetSocketAddress(port));
    this.port = port;
  }

  public ASRWebsocketServer(InetSocketAddress address) {
    super(address);
  }

  public ASRWebsocketServer(int port, Draft_6455 draft) {
    super(new InetSocketAddress(port), Collections.<Draft>singletonList(draft));
  }

  /**
   * Set the default speaker for the ASRWebsocketServer.
   * @param speaker Symbol of the speaker
   */
  public void setSpeaker (Symbol speaker) {
    this.speaker = speaker;
  }

  /**
   * Set the default addressee for the ASRWebsocketServer.
   * @param addressee Symbol of the addressee
   */
  public void setAddressee(Symbol addressee) {
    this.addressee = addressee;
  }

  /**
   * Callback for open event.
   *
   * @param conn      The <code>WebSocket</code> instance this event is occurring on.
   * @param handshake The handshake of the websocket instance
   */
  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {
    log.debug("Opened connection " + conn.hashCode());
  }

  /**
   * Callback for close event.
   *
   * @param conn   The <code>WebSocket</code> instance this event is occurring on.
   * @param code   The codes can be looked up here: {@link CloseFrame}
   * @param reason Additional information string
   * @param remote Returns whether or not the closing of the connection was initiated by the remote
   *               host.
   */
  @Override
  public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    log.debug("Closed connection " + conn.hashCode() + " with code " + code + ": " + reason);
    int id = conn.hashCode();
    if (audioData.containsKey(id)) {
      audioData.remove(id);
    }
    if (audioCursor.containsKey(id)) {
      audioCursor.remove(id);
    }
    if (audioSampleRate.containsKey(id)) {
      audioSampleRate.remove(id);
    }
  }

  /**
   * Callback for string message. Checks for opening and closing JSON objects.
   *
   * @param conn    The <code>WebSocket</code> instance this event is occurring on.
   * @param message The UTF-8 decoded message that was received.
   */
  @Override
  public void onMessage(WebSocket conn, String message) {
    log.debug("String message from " + conn + " " + message);
    if (message.contains("config")) {
      StartMessage startMessage = gson.fromJson(message, StartMessage.class);
      startRecording(conn, startMessage);
    } else if (message.contains("eof")) {
      EndMessage endMessage = gson.fromJson(message, EndMessage.class);
      endRecording(conn, endMessage);
    }
  }

  /**
   * Callback for byte array messages. Appends audio data to buffer.
   *
   * @param conn    The <code>WebSocket</code> instance this event is occurring on.
   * @param message The binary message that was received.
   */
  @Override
  public void onMessage(WebSocket conn, ByteBuffer message) {
    log.debug("Buffer message from " + conn + " " + message.toString());
    appendRecording(conn, message);
    //send blank partial transcription
    PartialMessage partial = new PartialMessage("");
    conn.send(gson.toJson(partial));
  }

  /**
   * Callback for error event.
   *
   * @param conn Can be null if there error does not belong to one specific websocket. For example
   *             if the servers port could not be bound.
   * @param ex   The exception causing this error
   */
  @Override
  public void onError (WebSocket conn, Exception ex) {
    log.error("Error on websocket server", ex);
    if (conn != null) {
      onClose(conn, 500, "Error in client", false);
    }
  }

  /**
   * Callback for start event.
   */
  @Override
  public void onStart() {
    log.info("ASRWebsocketServer started on port " + port);
    setConnectionLostTimeout(0);
    setConnectionLostTimeout(100);
  }

  /**
   * Wrapper for start method.
   *
   * @param args Server arguments
   * @throws InterruptedException
   * @throws IOException
   */
  public void start (String[] args) throws InterruptedException, IOException {
    ASRWebsocketServer s = new ASRWebsocketServer(port);
    s.start();
    log.info("ASRWebsocketServer started on port: " + s.getPort());
  }

  /**
   * Create audio buffer, cursor and stores sample rate of input audio.
   *
   * @param conn Websocket connection
   * @param startMessage Configuration message from client
   */
  private void startRecording (WebSocket conn, StartMessage startMessage) {
    int id = conn.hashCode();
    audioData.put(id, new float[startMessage.config.sample_rate * 30]);
    audioCursor.put(id, 0);
    audioSampleRate.put(id, startMessage.config.sample_rate);
    log.debug("Recording started on connection " + id + " with sample rate " + startMessage.config.sample_rate);
  }

  /**
   * Append audio data to buffer, advance cursor.
   *
   * @param conn Websocket connection
   * @param buffer Audio data to append
   */
  private void appendRecording (WebSocket conn, ByteBuffer buffer) {
    int id = conn.hashCode();
    if (audioData.containsKey(id)) {
      float[] samples = Audio.bytesToFloats(buffer);
      float[] savedAudio = audioData.get(id);
      int cursor = audioCursor.get(id);
      System.arraycopy(samples, 0, savedAudio, cursor, samples.length);
      audioData.put(id, savedAudio);
      audioCursor.put(id, cursor + samples.length);
      log.debug("Connection " + id + " added " + samples.length + " samples to recording at cursor " + cursor);
    }
  }

  /**
   * Close recording and pass audio data to transcribeAndReport.
   *
   * @param conn Websocket connection
   * @param endMessage EOF message from client
   */
  private void endRecording (WebSocket conn, EndMessage endMessage) {
    int id = conn.hashCode();
    if (audioData.containsKey(id)) {
      float[] samples = audioData.remove(id);
      int sampleRate = audioSampleRate.remove(id);
      int cursor = audioCursor.remove(id);
      if (sampleRate != 16000) {
        //TODO: resample here
      }
      transcribeAndReport(conn, Arrays.copyOfRange(samples, 0, cursor));
    }
  }

  /**
   * Transcribe audio using TRADEService transcribeSpeech and report to ListenerComponent
   * using reportRecognizeSpeech service if successful.
   *
   * @param conn Websocket connection
   * @param samples Audio data to transcribe
   */
  private void transcribeAndReport (WebSocket conn, float[] samples) {
    TRADEServiceInfo handle = null;
    String utt = null;
    TextMessage text = null;
    try {
      handle = TRADE.getAvailableService(new TRADEServiceConstraints().name("transcribeSpeech").argTypes(float[].class));
    } catch (TRADEException ex) {
      log.error("Error getting transcribeAndReport service handle", ex);
    }
    if (handle == null) {
      return;
    }
    try {
      utt = handle.call(String.class, samples);
    } catch (TRADEException ex) {
      log.error("Error calling transcribeAndReport service", ex);
    }

    if (utt == null) {
      log.warn("Transcribed speech was null");
      text = new TextMessage("");
      conn.send(gson.toJson(text));
      return;
    } else {
      text = new TextMessage(utt);
      conn.send(gson.toJson(text));
    }

    Utterance utterance = new Utterance.Builder()
        .setSpeaker(speaker)
        .setAddressee(addressee)
        .setWords(Arrays.asList(utt.split(" ")))
        .setUtteranceType(UtteranceType.UNKNOWN)
        .setIsInputUtterance(true)
        .build();

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
    } catch (TRADEException ex) {
      log.warn("Failed to report speech \"" + utt + "\"");
      return;
    }
    log.info("Reported utterance \"" + utt + "\"");
  }
}