/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSGoToStatusResponse {
  public String op;
  public String uri;
  public GoToStatusResponse response;

  public static RMSGoToStatusResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSGoToStatusResponse.class);
  }
}