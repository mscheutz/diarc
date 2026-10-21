/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.URI;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.drafts.Draft;
import org.java_websocket.handshake.ServerHandshake;

import edu.tufts.hrilab.ava.message.response.RMSGeneric;

public class AvaWebsocketClient extends WebSocketClient {

  public boolean connected = false;
  public boolean waiting = false;
  public String message = null;

  private Logger log = LoggerFactory.getLogger(AvaWebsocketClient.class);

  public AvaWebsocketClient(URI serverUri, Draft draft) {
    super(serverUri, draft);
  }

  public AvaWebsocketClient(URI serverURI) {
    super(serverURI);
  }

  public AvaWebsocketClient(URI serverUri, Map<String, String> httpHeaders) {
    super(serverUri, httpHeaders);
  }

  @Override
  public void onOpen(ServerHandshake handshakedata) {
    log.debug("Opened connection");
    connected = true;
  }

  @Override
  public void onMessage (String incomingMessage) {
    if (waiting && !RMSGeneric.isPush(incomingMessage)) {
      message = incomingMessage;
      log.debug("Received message: " + message);
      waiting = false;
    }
  }

  @Override
  public void onClose (int code, String reason, boolean remote) {
    // The close codes are documented in class org.java_websocket.framing.CloseFrame
    log.debug(
      "Connection closed by " + (remote ? "remote peer" : "us") + " Code: " + code + " Reason: "
        + reason);
    connected = false;
  }

  @Override
  public void onError (Exception ex) {
    log.error("Error in AvaClient websocket client");
    log.error(ex.toString());
  }
}