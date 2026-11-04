package edu.tufts.hrilab.ava.message.response;

import com.google.gson.Gson;

public class RMSDriveStatusResponse {
    public String op;
    public String uri;
    public DriveStatusResponse response;

    public static RMSDriveStatusResponse fromString(String responseString) {
        Gson gson = new Gson();
        return gson.fromJson(responseString, RMSDriveStatusResponse.class);
    }
}
