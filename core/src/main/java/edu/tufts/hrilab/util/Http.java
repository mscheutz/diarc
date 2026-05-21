/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;

import java.awt.image.BufferedImage;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.DataOutputStream;
import java.io.IOException;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.imageio.ImageIO;

public class Http {

    protected static Logger log = LoggerFactory.getLogger(Http.class);

    public static class HTTPRequestResponse {
        public String responseString;
        public int responseCode;

        HTTPRequestResponse(String responseString, int responseCode) {
            this.responseString = responseString;
            this.responseCode = responseCode;
        }
    }

    public static class HTTPRequestResponseStream {
        public BufferedReader responseStreamReader;
        public int responseCode;


        HTTPRequestResponseStream(BufferedReader responseStreamReader, int responseCode) {
            this.responseStreamReader = responseStreamReader;
            this.responseCode = responseCode;
        }
    }

    /**
     * Returns the JSON string response from an endpoint via GET.
     * @param endpointUrl URL, including path, to make request to
     * @param headers Headers for request in a map
     * @return Response from the endpoint as JSON string
     **/
    public static HTTPRequestResponse sendGetRequest (String endpointUrl, Map<String, String> headers) {
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
                return new HTTPRequestResponse(null, responseCode);
            }
            try {
                in = new BufferedReader(is);
            } catch (Exception e) {
                log.error("[sendGetRequest] Error reading response", e);
                return new HTTPRequestResponse(null, responseCode);
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

            return new HTTPRequestResponse(response.toString(),responseCode);
        } else {
            log.error("Request failed with response code: " + responseCode);
        }
        return new HTTPRequestResponse(null, responseCode);
    }

    public static HTTPRequestResponseStream sendPostRequestStreaming(String endpointUrl, Object requestBody, Map<String, String> headers) {
        HttpClient client = HttpClient.newHttpClient();
        HttpRequest request;
        Gson gson = new GsonBuilder().disableHtmlEscaping().create();
        String requestBodyJson = gson.toJson(requestBody);
        HttpRequest.Builder builder = HttpRequest.newBuilder()
                .uri(URI.create(endpointUrl))
                .POST(HttpRequest.BodyPublishers.ofString(requestBodyJson));

        if (headers != null) {
            for (Map.Entry<String,String> header : headers.entrySet()) {
                builder.header(header.getKey(), header.getValue());
            }
        }
        builder.header("Content-Type", "application/json");
        request = builder.build();
        log.debug("Request: " + requestBodyJson);

        HttpResponse<InputStream> response;
        // Send request and handle response as a stream
        try {
            response = client.send(request,
                    HttpResponse.BodyHandlers.ofInputStream());
        } catch (Exception e) {
            log.error("[sendPostRequestStreaming] exception sending request", e);
            return null;
        }

        if (response == null) {
            log.error("[sendPostRequestStreaming] got null response");
            return null;
        }

        int responseCode = response.statusCode();
        InputStream inputStream = response.body();
        InputStreamReader is = new InputStreamReader(inputStream);
        BufferedReader in;

        if (is == null) {
            return new HTTPRequestResponseStream(null, responseCode);
        }

        try {
            in = new BufferedReader(is);
        } catch (Exception e) {
            log.error("[sendPostRequest] Error reading response", e);
            return new HTTPRequestResponseStream(null, responseCode);
        }

        return new HTTPRequestResponseStream(in, responseCode);
    }
    /**
     * Returns the JSON string response from an endpoint via POST.
     * @param endpointUrl URL, including path, to make request to
     * @param requestBody Body payload of request
     * @param headers Headers for request in a map
     * @return Response from the endpoint as JSON string
     **/
    public static HTTPRequestResponse sendPostRequest(String endpointUrl, Object requestBody, Map<String, String> headers) {
        HTTPRequestResponseStream stream = sendPostRequestStreaming(endpointUrl, requestBody, headers);
        if (stream == null) {
            return null;
        } else if (stream.responseStreamReader == null) {
            return new HTTPRequestResponse(null, stream.responseCode);
        }

        BufferedReader in = stream.responseStreamReader;
        int responseCode = stream.responseCode;
        String inputLine;
        StringBuffer response = new StringBuffer();

        try {
            while ((inputLine = in.readLine()) != null) {
                response.append(inputLine);
            }
            in.close();
        } catch (Exception e) {
            log.error("[sendPostRequest] Error reading response to StringBuffer", e);
            return new HTTPRequestResponse(null, responseCode);
        }

        if (responseCode == HttpURLConnection.HTTP_OK) {
            return new HTTPRequestResponse(response.toString(), responseCode);
        } else {
            log.error("Request failed with response code: " + responseCode + ".");
            log.error(response.toString());
        }

        return new HTTPRequestResponse(null, responseCode);
    }

    /**
     * Get an image as a BufferedImage from a remote source.
     *
     * @param endpointUrl url of image to get
     * @return image data as BufferedImage
     */
    public static BufferedImage sendGetImage (String endpointUrl) {
        BufferedImage bufferedImage = null;
        URL url = null;
        try {
            url = new URL(endpointUrl);
        } catch (Exception e) {
            log.error("[sendGetImage] Error constructing URL", e);
        }

        if (url != null) {
            try {
                bufferedImage = ImageIO.read(url);
            } catch (IOException e) {
                log.error("[sendGetImage] Error getting image", e);
            }
        }

        return bufferedImage;
    }
}
