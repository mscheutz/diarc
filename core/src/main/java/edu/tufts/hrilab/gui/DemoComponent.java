/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import java.io.IOException;

//import org.json.JSONException;
//import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

@Component
public class DemoComponent extends TextWebSocketHandler {

  @Override
  public void handleTextMessage(WebSocketSession session, TextMessage message)
          throws InterruptedException, IOException {

//    String payload = message.getPayload();
//    JSONObject jsonObject = null;
//    try {
//      jsonObject = new JSONObject(payload);
//      session.sendMessage(new TextMessage("Hi " + jsonObject.get("user") + " how may we help you?"));
//    } catch (JSONException e) {
//      throw new RuntimeException(e);
//    }

  }

}