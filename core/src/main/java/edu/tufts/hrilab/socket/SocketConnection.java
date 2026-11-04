/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.socket;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.net.InetAddress;
import java.net.Socket;

public class SocketConnection {
  protected static int RESP_SLEEP = 50;
  protected static int MAX_RESP_TIME = 5000;

  protected static String DEFAULT_IP = "127.0.0.1";
  protected static int DEFAULT_PORT = 9000;

  protected Logger log = LoggerFactory.getLogger(SocketConnection.class);

  protected Socket sock;
  protected PrintWriter out;
  protected BufferedReader in;

  protected String ipAddress;
  protected int port;

  public SocketConnection() throws IOException {
    this(DEFAULT_IP, DEFAULT_PORT);
  }

  public SocketConnection(int port) throws IOException {
    this(DEFAULT_IP, port);
  }

  public SocketConnection(String ip, int port) throws IOException {
    this.ipAddress = ip;
    this.port = port;
    this.sock = new Socket(InetAddress.getByName(ip), port);
    this.sock.setKeepAlive(true);
    this.out = new PrintWriter(this.sock.getOutputStream(), true);
    this.in = new BufferedReader(new InputStreamReader(this.sock.getInputStream()));
  }

  public String waitedResponse() {
    return waitedResponse(MAX_RESP_TIME);
  }

  public String waitedResponse(int max_time) {
    String response;
    long startTime = System.currentTimeMillis();

    while (System.currentTimeMillis() - startTime < max_time) {
      log.debug("Waiting for response...");
      try {
        if (in.ready()) {  // Check if data is available
          log.debug("Reading line...");
          response = in.readLine();
          if (response == null) {
            log.warn("{}: Received null response.", port);
          }
          return response;
        }
      } catch (IOException e) {
        log.error("{}: Error reading message.", port, e);
        return null;
      }

      try {
        Thread.sleep(RESP_SLEEP);
      } catch (InterruptedException e) {
        log.error("Interrupted while sleeping.", e);
        return null;
      }
    }

    log.warn("{}: No message received within {} seconds.", port, max_time / 1000.0);
    return null;
  }


  public void sendCommand(String message) {
    // Clear out past responses if they exist
    StringBuilder readMsg = new StringBuilder();
    try {
      while (in.ready()) {
        String line = in.readLine();
        if (line != null) {
          readMsg.append("[").append(line).append("]  ");
        }
      }
    } catch (IOException e) {
      log.error("{}: Error reading available messages before sending.", port, e);
    }

    if (!readMsg.isEmpty()) {
      log.warn("{}: [sendCommand] Read stale messages before sending: {}", port, readMsg);
    }

    sendMessage(message);
  }

  public void sendMessage(String message) {
    out.println(message);
    out.flush();
  }

  public void closeSocket() {
    try {
      this.sock.close();
      this.in.close();
      this.out.close();
    } catch (IOException e) {
      log.error("{}: Error closing socket.", port, e);
    }
  }

  public boolean isConnected() {
    return sock != null && sock.isConnected() && !sock.isClosed();
  }

}
