/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

import java.util.Map;

public class RMSListTagsResponse {

  public String op;
  public String uri;
  public ListTagsResponse response;

  public static RMSListTagsResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSListTagsResponse.class);
  }
}
