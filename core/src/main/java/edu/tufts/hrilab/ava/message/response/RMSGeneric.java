/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSGeneric {
  public String op;

  public static boolean isPush (String responseString) {
    Gson gson = new Gson();
    RMSGeneric t = gson.fromJson(responseString, RMSGeneric.class);
    return t.op.equals("push");
  }
}
