/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public class RMSRequest {
  public String op = "request";
  public String uri;
  public Object args;

  public RMSRequest (String uriString) {
    uri = uriString;
    args = new Object();
  }

  public RMSRequest (String uriString, Object argsObject) {
    uri = uriString;
    args = argsObject;
  }

  public String toString () {
    Gson gson = new GsonBuilder().serializeNulls().create();
    return gson.toJson(this);
  }
}