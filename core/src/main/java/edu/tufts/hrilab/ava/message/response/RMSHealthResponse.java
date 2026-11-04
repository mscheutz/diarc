/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSHealthResponse {
    public String op;
    public String uri;
    public HealthResponse response;

    public RMSHealthResponse() {}

    public static RMSHealthResponse fromString (String responseString) {
        Gson gson = new Gson();
        return gson.fromJson(responseString, RMSHealthResponse.class);
    }
}