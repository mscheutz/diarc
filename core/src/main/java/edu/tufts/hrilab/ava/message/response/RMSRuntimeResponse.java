/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSRuntimeResponse {
    public String op;
    public String uri;
    public RuntimeResponse response;

    public static RMSRuntimeResponse fromString (String responseString) {
        Gson gson = new Gson();
        return gson.fromJson(responseString, RMSRuntimeResponse.class);
    }
}