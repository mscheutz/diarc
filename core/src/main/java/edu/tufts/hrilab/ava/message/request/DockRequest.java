/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public class DockRequest {
  public String op = "request";
  public String uri;
  public String wait = "1";

  public DockRequest(String uriString) {
    uri = uriString;
  }

  public String toString () {
    Gson gson = new GsonBuilder().serializeNulls().create();
    return gson.toJson(this);
  }
}