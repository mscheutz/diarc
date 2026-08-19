/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.net.ssl.HttpsURLConnection;
import javax.net.ssl.SSLContext;
import javax.net.ssl.TrustManager;
import javax.net.ssl.X509TrustManager;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.security.cert.X509Certificate;
import java.util.Base64;

public class CiscoCodecAPI {

  private static final Logger log = LoggerFactory.getLogger(CiscoCodecAPI.class);

  public enum DisplayMode {
    TELEPRESENCE,
    TRACS,
    CAMERA,
    NONE,
    UNKNOWN
  }

  private final String codecIp;
  private final String encodedAuth;

  // Connector IDs: 1=Camera, 2=PC1/Telepresence, 3=PC2/TRACS
  public static final int CONNECTOR_ID_CAMERA = 1;
  public static final int CONNECTOR_ID_TELEPRESENCE = 2;
  public static final int CONNECTOR_ID_TRACS = 3;

  public CiscoCodecAPI() {
    this("172.18.0.50", "admin", "avasupport");
  }

  public CiscoCodecAPI(String codecIp, String username, String password) {
    this.codecIp = codecIp;
    this.encodedAuth = Base64.getEncoder().encodeToString(
            (username + ":" + password).getBytes(StandardCharsets.UTF_8)
    );
    initTrustAllCerts();
  }

  // Cisco codecs use self-signed certs, so we need to trust all certs
  private void initTrustAllCerts() {
    try {
      TrustManager[] trustAllCerts = new TrustManager[]{
              new X509TrustManager() {
                public X509Certificate[] getAcceptedIssuers() { return null; }
                public void checkClientTrusted(X509Certificate[] certs, String authType) {}
                public void checkServerTrusted(X509Certificate[] certs, String authType) {}
              }
      };
      SSLContext sc = SSLContext.getInstance("TLS");
      sc.init(null, trustAllCerts, new java.security.SecureRandom());
      HttpsURLConnection.setDefaultSSLSocketFactory(sc.getSocketFactory());
      HttpsURLConnection.setDefaultHostnameVerifier((hostname, session) -> true);
    } catch (Exception e) {
      log.error("[initTrustAllCerts] Failed to init SSL context", e);
    }
  }

  public boolean switchToTRACS() {
    log.info("[switchToTRACS] Switching to PC 2");
    return startPresentation(CONNECTOR_ID_TRACS);
  }

  public boolean startPresentation(int connectorId) {
    String xmlCommand = String.format(
            "<Command><Presentation><Start><ConnectorId>%d</ConnectorId></Start></Presentation></Command>",
            connectorId
    );
    return sendCommand(xmlCommand);
  }

  //scoped to one source so a share we did not start is never touched. verified on
  //ce11.40.1.1: stops only that source's share, no-ops cleanly when nothing is
  //presenting, and leaves a different source's share up. never send an unscoped Stop.
  //Start takes ConnectorId and Stop takes PresentationSource by Cisco's API; both are
  //the video input number.
  public boolean stopPresentation(int connectorId) {
    log.info("[stopPresentation] Stopping presentation from connector {}", connectorId);
    String xmlCommand = String.format(
            "<Command><Presentation><Stop><PresentationSource>%d</PresentationSource></Stop></Presentation></Command>",
            connectorId
    );
    return sendCommand(xmlCommand);
  }

  public boolean isPresentationActive() {
    return getActiveConnectorId() != -1;
  }

  public int getActiveConnectorId() {
    String status = sendGetRequest("/getxml?location=/Status/Conference/Presentation");
    if (status == null) {
      log.warn("[getActiveConnectorId] Failed to get presentation status");
      return -1;
    }
    log.debug("[getActiveConnectorId] Status: {}", status);

    if (status.contains("<ghost>True</ghost>")) {
      return -1;
    }

    //a LocalInstance element exists exactly while a local presentation is up: verified on
    //ce11.40.1.1 that Presentation Stop removes it,
    //Start recreates it, and Mode stays Off throughout for LocalOnly shares -- so Mode
    //says nothing about local state. scope the Source read to the instance rather than
    //taking the first Source anywhere in the document.
    int instanceStart = status.indexOf("<LocalInstance");
    if (instanceStart == -1) {
      return -1;
    }
    int sourceStart = status.indexOf("<Source>", instanceStart);
    int sourceEnd = status.indexOf("</Source>", instanceStart);
    if (sourceStart != -1 && sourceEnd != -1 && sourceStart < sourceEnd) {
      try {
        return Integer.parseInt(status.substring(sourceStart + 8, sourceEnd).trim());
      } catch (NumberFormatException e) {
        log.warn("[getActiveConnectorId] Failed to parse connector ID", e);
      }
    }
    return -1;
  }

  public DisplayMode getCurrentDisplayMode() {
    int connectorId = getActiveConnectorId();
    switch (connectorId) {
      case CONNECTOR_ID_TELEPRESENCE: return DisplayMode.TELEPRESENCE;
      case CONNECTOR_ID_TRACS: return DisplayMode.TRACS;
      case CONNECTOR_ID_CAMERA: return DisplayMode.CAMERA;
      case -1: return DisplayMode.NONE;
      default: return DisplayMode.UNKNOWN;
    }
  }

  public boolean isTelepresenceActive() {
    return getActiveConnectorId() == CONNECTOR_ID_TELEPRESENCE;
  }

  public boolean isTRACSActive() {
    return getActiveConnectorId() == CONNECTOR_ID_TRACS;
  }

  public boolean isInCall() {
    String status = sendGetRequest("/getxml?location=/Status/Call");
    if (status == null) {
      return false;
    }
    return status.contains("<Call item");
  }

  private boolean sendCommand(String xmlCommand) {
    HttpsURLConnection connection = null;
    try {
      URL url = new URL("https://" + codecIp + "/putxml");
      connection = (HttpsURLConnection) url.openConnection();
      connection.setRequestMethod("POST");
      connection.setRequestProperty("Authorization", "Basic " + encodedAuth);
      connection.setRequestProperty("Content-Type", "text/xml");
      connection.setDoOutput(true);
      connection.setConnectTimeout(5000);
      connection.setReadTimeout(5000);

      try (OutputStream os = connection.getOutputStream()) {
        os.write(xmlCommand.getBytes(StandardCharsets.UTF_8));
        os.flush();
      }

      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        StringBuilder response = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(connection.getInputStream(), StandardCharsets.UTF_8))) {
          String line;
          while ((line = reader.readLine()) != null) {
            response.append(line);
          }
        }
        //the codec answers 200 with status="Error" in the body when it rejects a
        //command; the http code alone says nothing about acceptance
        if (response.toString().contains("status=\"Error\"")) {
          log.error("[sendCommand] codec rejected {}: {}", xmlCommand, response);
          return false;
        }
        log.debug("[sendCommand] Response: {}", response);
        return true;
      } else {
        log.error("[sendCommand] HTTP {}: {}", responseCode, xmlCommand);
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(connection.getErrorStream(), StandardCharsets.UTF_8))) {
          StringBuilder errorResponse = new StringBuilder();
          String line;
          while ((line = reader.readLine()) != null) {
            errorResponse.append(line);
          }
          log.error("[sendCommand] Error: {}", errorResponse);
        } catch (Exception ignored) {}
        return false;
      }
    } catch (Exception e) {
      log.error("[sendCommand] Exception: {}", xmlCommand, e);
      return false;
    } finally {
      if (connection != null) {
        connection.disconnect();
      }
    }
  }

  private String sendGetRequest(String path) {
    HttpsURLConnection connection = null;
    try {
      URL url = new URL("https://" + codecIp + path);
      connection = (HttpsURLConnection) url.openConnection();
      connection.setRequestMethod("GET");
      connection.setRequestProperty("Authorization", "Basic " + encodedAuth);
      connection.setConnectTimeout(5000);
      connection.setReadTimeout(5000);

      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(connection.getInputStream(), StandardCharsets.UTF_8))) {
          StringBuilder response = new StringBuilder();
          String line;
          while ((line = reader.readLine()) != null) {
            response.append(line).append("\n");
          }
          return response.toString();
        }
      } else {
        log.error("[sendGetRequest] HTTP {}: {}", responseCode, path);
        return null;
      }
    } catch (Exception e) {
      log.error("[sendGetRequest] Exception: {}", path, e);
      return null;
    } finally {
      if (connection != null) {
        connection.disconnect();
      }
    }
  }
}
