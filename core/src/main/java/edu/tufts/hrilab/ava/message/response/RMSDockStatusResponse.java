/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSDockStatusResponse {
  public String op;
  public String uri;
  public DockStatusResponse response;

  public static RMSDockStatusResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSDockStatusResponse.class);
  }
}