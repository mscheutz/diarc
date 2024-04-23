/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Iterator;
import java.io.IOException;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.util.Util;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.socket.SocketConnection;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.unity.message.Message;

/**
 * Provides a socket-based connection to Unity for message passing
 * from and to TRADE services.
 * 
 */
public class UnityComponent extends DiarcComponent {
  
  private Logger log = LoggerFactory.getLogger(UnityComponent.class);
  private String msgStr = null;

  /**
   * Default Unity/DIARC socket server IP address and port.
   */

  private String ip = "127.0.0.1";
  private int port = 1755;
  private SocketConnection socketClient;
  private static List<Message> queue = new ArrayList<Message>();

  public UnityComponent() {
    super();
    this.shouldRunExecutionLoop = true;
  }

  @Override 
  protected void init () {
    boolean notFound = true;
    while (notFound) {
      try {
        this.socketClient = new SocketConnection(ip, port);
        notFound = false;
      } catch (IOException e) {
        notFound = true;
        log.error("Could not instantiate Unity Websocket Client: " + ip + ":" + port + " Retrying...");
        Util.Sleep(1000);
      }
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("unityport").hasArg().argName("port").desc("Set the Unity->DIARC socket server port. (Default=1755)").build());
    options.add(Option.builder("unityip").hasArg().argName("ip").desc("Set the Unity->DIARC socket server IP address. (Default=127.0.0.1)").build());
    return options;
  }

  @Override
  protected void parseArgs (CommandLine cmdLine) {
    if (cmdLine.hasOption("unityip")) {
      ip = cmdLine.getOptionValue("unityip");
    }
    if (cmdLine.hasOption("unityport")) {
      port = Integer.parseInt(cmdLine.getOptionValue("unityport"));
    }
  }

  @Override
  protected void executionLoop () {
    Message msg = null;
    msgStr = this.socketClient.waitedResponse();
    if (msgStr != null) {
      try {
        msg = parseMessage(msgStr);
      } catch (Exception e) {
        log.error("Error parsing message from Unity", e);
      }
      if (msg != null) {
        UnityComponent.queue.add(msg);
      }
    }
    for (int i = UnityComponent.queue.size() - 1; i > -1; --i) {
      Message imsg = UnityComponent.queue.get(i);
      if (imsg.response == null) {
        onReceive(imsg);
        UnityComponent.queue.remove(i);
      }
    }
  }

  /**
   * Method for passing messages received from the Unity
   * socket server to all Unity agents which register TRADE
   * services for receiving messages.
   *
   * @param msg - Message received from Unity.
   */
  public void onReceive (Message msg) {
    if (msg.response == null) {
      proxyMessage(msg, "unityReceiveMessage");
    }
  }

  /**
   * Method for proxying Message to specified TRADE service.
   * 
   * @param msg - Message from Unity to proxy.
   * @param tradeServiceName - TRADE service to receive message.
   */
  protected void proxyMessage (Message msg, String tradeServiceName) {

    TRADE.getAvailableServices(new TRADEServiceConstraints().name(tradeServiceName).argTypes(Message.class)).stream().forEach(handle -> {
      try {
        handle.call(Object.class, msg);
      } catch (TRADEException e) {
        log.error("Error calling " + tradeServiceName + ".", e);
      }
    });
  }

  /**
   * Parse JSON from string to Message type.
   *
   * @param str - Serialized string of JSON originating from Unity.
   */
  public Message parseMessage (String str) {
    return Message.fromString(str);
  }

  /**
   * Send message via local socket client.
   * 
   * @param msg - Message to send to Unity.
   **/
  public void sendMessage (Message msg) {
    String json = msg.toString();
    this.socketClient.sendMessage(json + "\n");
  }

  public Message getResponse (Message msg) {
    boolean resolved = false;
    String responseStr = null;
    Message responseMsg = null;

    sendMessage(msg);
    try {
      Thread.sleep(2);
    } catch (InterruptedException e) {
      //
    }  
    while (!resolved) {
      for (int i = UnityComponent.queue.size() - 1; i > -1; --i) {
        Message imsg = UnityComponent.queue.get(i);
        if (imsg.id.equals(msg.id)) {
          responseMsg = imsg;
          UnityComponent.queue.remove(i);
          resolved = true;
        } else if (imsg.response == null) {
          onReceive(imsg);
          UnityComponent.queue.remove(i);
        }
      }
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        // Handle the exception here
      }  
    }
    return responseMsg;
  }

  /**
   * Exposes sendMessage() to TRADE.
   * 
   * @param msg - Message to send to Unity.
   **/
  @TRADEService
  public synchronized boolean unitySendMessage (Message msg) {
    sendMessage(msg);
    return true;
  }

  /**
   * Exposes getResponse() to TRADE.
   * 
   * @param msg - Message to send to Unity.
   **/
  @TRADEService
  public synchronized List<String> unityGetResponse (Message msg) {
    return getResponse(msg).response;
  }

  /**
   * TRADE service which allows agent to speak within Unity.
   * Uses as Unity-based TTS to synthesize speech in simulation.
   * 
   * @param text - String of message for agent to speak in Unity
   **/
  @TRADEService
  @Action
  public synchronized boolean sayText (Symbol actor, String text) {
    Message msg = new Message(actor.toString(), "speak", text.toString());
    sendMessage(msg);
    return true;
  }
}
