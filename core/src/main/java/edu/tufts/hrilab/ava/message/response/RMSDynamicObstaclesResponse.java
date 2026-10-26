/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSDynamicObstaclesResponse {
  public DynamicObstaclesResponse response;

  public static RMSDynamicObstaclesResponse fromString (String responseString) {
    Gson gson = new Gson();
    return gson.fromJson(responseString, RMSDynamicObstaclesResponse.class);
  }
}