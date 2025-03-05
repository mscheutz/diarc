/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import java.util.ArrayList;
import java.util.List;
import java.io.IOException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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
  private final List<Message> queue = new ArrayList<>();
  private final Lock queueLock = new ReentrantLock();
  private final Condition queueCondition = queueLock.newCondition();

  public UnityComponent() {
    super();
    this.shouldRunExecutionLoop = true;
  }

  @Override
  protected void init() {
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
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("unityip")) {
      ip = cmdLine.getOptionValue("unityip");
    }
    if (cmdLine.hasOption("unityport")) {
      port = Integer.parseInt(cmdLine.getOptionValue("unityport"));
    }
  }

  @Override
  protected void executionLoop() {
    Message msg = null;
    msgStr = this.socketClient.waitedResponse();
    log.debug("[executionLoop] waitedResponse returned");
    if (msgStr != null) {
      try {
        msg = parseMessage(msgStr);
      } catch (Exception e) {
        log.error("Error parsing message from Unity", e);
      }

      if (msg != null) {
        if (msg.response == null) {
          // new msg originating from unity
          log.debug("New unity msg received: {}", msg);
          onReceive(msg);
        } else {
          // response msg for msg send from diarc
          log.debug("Response msg from unity received: {}", msg);
          queueLock.lock();
          try {
            log.debug("Adding response msg to queue: {}", msg);
            queue.add(msg);
            queueCondition.signalAll();
          } finally {
            queueLock.unlock();
          }
        }
      } else {
        log.error("Null msg received from Unity socket.");
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
  public void onReceive(Message msg) {
    if (msg.response == null) {
      proxyMessage(msg, "unityReceiveMessage");
    }
  }

  /**
   * Method for proxying Message to specified TRADE service.
   *
   * @param msg              - Message from Unity to proxy.
   * @param tradeServiceName - TRADE service to receive message.
   */
  protected void proxyMessage(Message msg, String tradeServiceName) {

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
  public Message parseMessage(String str) {
    return Message.fromString(str);
  }

  /**
   * Send message via local socket client.
   *
   * @param msg - Message to send to Unity.
   **/
  public void sendMessage(Message msg) {
    String json = msg.toString();
    this.socketClient.sendMessage(json + "\n");
  }

  public Message getResponse(Message msg) {
    // send msg
    sendMessage(msg);
    Util.Sleep(2); // EAK: why?

    // wait for response
    Message responseMsg = null;
    log.debug("Waiting for unity response from sent msg: {}", msg);
    queueLock.lock();
    try {
      while (responseMsg == null) {
        // check already received msgs on the queue
        for (Message imsg : queue) {
          if (msg.id.equals(imsg.id)) {
            log.debug("Unity response received: {}", imsg);
            responseMsg = imsg;
            queue.remove(imsg);
            break;
          }
        }

        // if didn't get response, wait until notified to check again
        if (responseMsg == null) {
          queueCondition.await();
        }
      }
    } catch (InterruptedException e) {
      log.error("Error waiting for unity socket response.", e);
    } finally {
      queueLock.unlock();
    }

    return responseMsg;
  }

  /**
   * Exposes sendMessage() to TRADE.
   *
   * @param msg - Message to send to Unity.
   **/
  @TRADEService
  public synchronized boolean unitySendMessage(Message msg) {
    sendMessage(msg);
    return true;
  }

  /**
   * Exposes getResponse() to TRADE.
   *
   * @param msg - Message to send to Unity.
   **/
  @TRADEService
  public synchronized List<String> unityGetResponse(Message msg) {
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
  public synchronized boolean sayText(Symbol actor, String text) {
    log.debug("Unity sayText: \"" + text + "\"");
    Message msg = new Message(actor.toString(), "speak", text.toString());
    sendMessage(msg);
    return true;
  }
}
