/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSPositionResponse {
  public String op;
  public String uri;
  public PositionResponse response;

  public static RMSPositionResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSPositionResponse.class);
  }
}