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
    this.out = new PrintWriter(this.sock.getOutputStream(), true);
    this.in = new BufferedReader(new InputStreamReader(this.sock.getInputStream()));
  }

  public String waitedResponse() {
    return waitedResponse(MAX_RESP_TIME);
  }

  public String waitedResponse(int max_time) {
    String response = null;

    int cycles = (int) Math.ceil(((float) max_time) / RESP_SLEEP);
    int i;
    long startTime = System.currentTimeMillis();
    for (i = 0; i < cycles; i++) {
      if (this.messageAvailable()) {
        response = this.readMessage();
        if (response == null) {
          log.warn(port + ": Received null response.");
        }
        break;
      }
      try {
        Thread.sleep(RESP_SLEEP);
      } catch (InterruptedException e) {
        log.error("Interrupted while sleeping.", e);
        return null;
      }
    }
    long endTime = System.currentTimeMillis();

    if (i >= cycles) {
      log.warn(port + ": No msg available. Max wait time specified: " + (max_time/1000.0) + " Wall clock duration: " +
              ((endTime - startTime)/1000.0) + ". Returning null.");
    }
    log.debug("                                Socket wait time: "  + ((endTime - startTime)/1000.0));

    return response;
  }

  public void sendCommand(String message) {
    //Clear out past responses if they exist
    StringBuilder readMsg = new StringBuilder();
    while (this.messageAvailable()) {
      readMsg.append("[").append(this.readMessage()).append("]  ");
    }
    if (!readMsg.toString().isEmpty()) {
      log.warn(port + ": [sendCommand] reading available message before sending. Read message: " + readMsg);
    }

    this.sendMessage(message);
  }

  private String readMessage() {
    try {
      return this.in.readLine();
    } catch (IOException e) {
      log.error(port + ": Error reading message. Returning null.", e);
      return null;
    }
  }

  private boolean messageAvailable() {
    try {
      return this.in.ready();
    } catch (IOException e) {
      log.error(port + ": Error checking if message is available. Returning false.", e);
      return false;
    }
  }

  public void sendMessage(String message) {
    out.println(message);
  }

  public void closeSocket() {
    try {
      this.sock.close();
      this.in.close();
      this.out.close();
    } catch (IOException e) {
      log.error(port + ": Error closing socket.", e);
    }
  }

  public boolean isConnected() {
    return sock.isConnected();
  }
}
