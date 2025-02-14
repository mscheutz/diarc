/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSZLiftCameraTiltResponse {
    public String op;
    public String uri;
    public ZLiftCameraTiltResponse response;

    public static RMSZLiftCameraTiltResponse fromString (String responseString) {
        Gson gson = new Gson();
        return gson.fromJson(responseString, RMSZLiftCameraTiltResponse.class);
    }
}