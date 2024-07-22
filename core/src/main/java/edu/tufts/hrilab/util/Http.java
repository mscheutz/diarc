/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.DataOutputStream;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.util.Map;
import java.util.HashMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Http {

    protected static Logger log = LoggerFactory.getLogger(Http.class);

    /**
     * Returns the JSON string response from an endpoint via GET.
     * @param endpointUrl URL, including path, to make request to
     * @param headers Headers for request in a map
     * @return Response from the endpoint as JSON string
     **/
    public static String sendGetRequest (String endpointUrl, Map<String, String> headers) {
        URL url;
        HttpURLConnection con;
        log.debug("GET " + endpointUrl);
        try {
            url = new URL(endpointUrl);
            con = (HttpURLConnection) url.openConnection();
            con.setRequestMethod("GET");
        } catch (Exception e) {
            log.error("[sendGetRequest] Error opening connection to " + endpointUrl, e);
            return null;
        }

        // Set headers if present
        if (headers != null) {
            for (Map.Entry<String, String> entry : headers.entrySet()) {
                con.setRequestProperty(entry.getKey(), entry.getValue());
            }
        }

        int responseCode;
        try {
            responseCode = con.getResponseCode();
        } catch (Exception e) {
            log.error("[sendGetRequest] Error reading response code", e);
            return null;
        }

        if (responseCode == HttpURLConnection.HTTP_OK) {
            InputStreamReader is = null;
            BufferedReader in = null;
            try {
                is = new InputStreamReader(con.getInputStream());
            } catch (IOException e) {
                log.error("[sendGetRequest] Error reading input stream from connection", e);
                is = new InputStreamReader(con.getErrorStream());
            }
            if (is == null) {
                return null;
            }
            try {
                in = new BufferedReader(is);
            } catch (Exception e) {
                log.error("[sendGetRequest] Error reading response", e);

                return null;
            }

            String inputLine;
            StringBuffer response = new StringBuffer();

            try {
                while ((inputLine = in.readLine()) != null) {
                    response.append(inputLine);
                }
                in.close();
            } catch (Exception e) {
                log.error("[sendGetRequest] Error reading response to StringBuffer", e);
                return null;
            }

            return response.toString();
        } else {
            log.error("Request failed with response code: " + responseCode);
        }
        return null;
    }

    /**
     * Returns the JSON string response from an endpoint via POST.
     * @param endpointUrl URL, including path, to make request to
     * @param requestBody Body payload of request
     * @param headers Headers for request in a map
     * @return Response from the endpoint as JSON string
     **/
    public static String sendPostRequest(String endpointUrl, Object requestBody, Map<String, String> headers) {
        URL url;
        HttpURLConnection con;
        int responseCode;
        log.debug("POST " + endpointUrl);
        try {
            url = new URL(endpointUrl);
            con = (HttpURLConnection) url.openConnection();
            con.setRequestMethod("POST");
            con.setRequestProperty("Content-Type", "application/json");
        } catch (Exception e) {
            log.error("[sendPostRequest] Error opening connection to " + endpointUrl, e);
            return null;
        }

        // Set headers if present
        if (headers != null) {
            for (Map.Entry<String, String> entry : headers.entrySet()) {
                con.setRequestProperty(entry.getKey(), entry.getValue());
            }
        }

        con.setDoOutput(true);

        // Convert request body to JSON string
        Gson gson = new GsonBuilder().create();
        String requestBodyJson = gson.toJson(requestBody);
        log.debug("Request: " + requestBodyJson);
        // Send request body
        DataOutputStream wr;
        try {
            wr = new DataOutputStream(con.getOutputStream());
            wr.writeBytes(requestBodyJson);
            wr.flush();
            wr.close();
        } catch (Exception e) {
            log.error("[sendPostRequest] Error writing output stream", e);
            return null;
        }

        try {
            responseCode = con.getResponseCode();
        } catch (Exception e) {
            log.error("[sendPostRequest] Error getting response code", e);
            return null;
        }

        InputStreamReader is = null;
        BufferedReader in = null;

        try {
            is = new InputStreamReader(con.getInputStream());
        } catch (IOException e) {
            log.error("[sendPostRequest] Error reading input stream from connection", e);
            is = new InputStreamReader(con.getErrorStream());
        }

        if (is == null) {
            return null;
        }

        try {
            in = new BufferedReader(is);
        } catch (Exception e) {
            log.error("[sendPostRequest] Error reading response", e);
            return null;
        }

        String inputLine;
        StringBuffer response = new StringBuffer();

        try {
            while ((inputLine = in.readLine()) != null) {
                response.append(inputLine);
            }
            in.close();
        } catch (Exception e) {
            log.error("[sendPostRequest] Error reading response to StringBuffer", e);
            return null;
        }

        if (responseCode == HttpURLConnection.HTTP_OK) {
            return response.toString();
        } else {
            log.error("Request failed with response code: " + responseCode + ".");
            log.error(response.toString());
        }

        return null;
    }
}