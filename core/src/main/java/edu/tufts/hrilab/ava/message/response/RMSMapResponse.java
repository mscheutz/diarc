/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSMapResponse {
  public String op;
  public String uri;
  public MapResponse response;

  public static RMSMapResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSMapResponse.class);
  }
}