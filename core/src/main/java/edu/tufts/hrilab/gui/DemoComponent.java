/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import org.json.JSONException;
import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.io.IOException;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;


// @component gets pulled into the app
@Component
public class DemoComponent extends TextWebSocketHandler {

  @Override
  public void handleTextMessage(WebSocketSession session, TextMessage message)
          throws InterruptedException, IOException {

    String payload = message.getPayload();
    JSONObject jsonObject = null;
    try {
      jsonObject = new JSONObject(payload);
      session.sendMessage(new TextMessage("Hi " + jsonObject.get("user") + " how may we help you?"));
    } catch (JSONException e) {
      throw new RuntimeException(e);
    }

  }

  private final Set<WebSocketSession> sessions = new CopyOnWriteArraySet<>();

  @Override
  public void afterConnectionEstablished(WebSocketSession session) throws Exception {
    sessions.add(session);
  }

  @Override
  public void afterConnectionClosed(WebSocketSession session, CloseStatus status) throws Exception {
    sessions.remove(session);
  }

  public void broadcastUpdate(String message) {
    for (WebSocketSession session : sessions) {
      try {
        session.sendMessage(new TextMessage(message));
      } catch (IOException e) {
        // Log error, attempt to close session, etc.
      }
    }
  }


}