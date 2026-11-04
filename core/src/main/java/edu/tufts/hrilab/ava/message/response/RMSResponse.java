/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSResponse {
  public String op;
  public String uri;
  public Object response;
  public String errorMessage;

  public static RMSResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSResponse.class);
  }
}