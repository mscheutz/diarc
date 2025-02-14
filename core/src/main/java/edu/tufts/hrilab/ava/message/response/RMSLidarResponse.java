/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSLidarResponse {
    public String op;
    public String uri;
    public LidarResponse response;

    public RMSLidarResponse() {}

    public static RMSLidarResponse fromString (String responseString) {
        Gson gson = new Gson();
        return gson.fromJson(responseString, RMSLidarResponse.class);
    }
}