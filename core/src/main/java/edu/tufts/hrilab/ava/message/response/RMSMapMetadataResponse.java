/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSMapMetadataResponse {
  public String op;
  public String uri;
  public MapMetadataResponse response;

  public RMSMapMetadataResponse() {}

  public static RMSMapMetadataResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSMapMetadataResponse.class);
  }
}