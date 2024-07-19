/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;

import edu.tufts.hrilab.unity.message.Message;

/**
 * Routes agent-specific messages from UnityComponent to and from TRADE services.
 * 
 */
public class UnityAgent extends DiarcComponent {
	static private Logger log = LoggerFactory.getLogger(UnityAgent.class);
	
	public String agent = ""; //name of agent, must match Unity
  public boolean isConnected = false;

  private Symbol listener;

	public UnityAgent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("agent").hasArg().argName("agent").desc("Set the agent name.").build());
    options.add(Option.builder("nullListener").argName("bool").desc("Use a null listener.").build());
    return options;
  }

  @Override
  protected void parseArgs (CommandLine cmdLine) {
    if (cmdLine.hasOption("agent")) {
      agent = cmdLine.getOptionValue("agent");
    } else {
    	log.error("Agent name is required to route messages from Unity");
    }
    if (cmdLine.hasOption("nullListener")) {
      listener = Factory.createSymbol("unknown");
      log.debug("Agent " + agent + " is using a default null listener");
    } else {
      listener = Factory.createSymbol(this.agent);
    }
  }

  /**
   * Route a message received from the Unity socket server
   * to the appropriate method that matches the named action.
   *
   * @param msg - Message originating from Unity to route to method.
   */
  protected void route (Message msg) {
    switch (msg.action) {
      case "listen": listen(msg);
        break;
      case "connected": connected(msg);
        break;
    }
  }

  /**
   * Agent listens to a message spoken by an agent in Unity.
   * 
   * All "listen" messages will contain the following:
   * actor = listener
   * arguments[0] = speaker
   * arguments[1] = message
   * 
   * @param msg - Message with action "listen".
   */
  protected void listen (Message msg) {
    String text = msg.arguments.get(1);
    Symbol speaker = Factory.createSymbol(msg.arguments.get(0));
    Utterance.Builder utterance = new Utterance.Builder()
      .setWords(Arrays.asList(text.split(" ")))
      .setSpeaker(speaker)
      .addListener(listener)
      .setIsInputUtterance(true);
    listen(utterance.build());
  }

  protected void listen (Utterance input) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, input);
    } catch (TRADEException e) {
      log.error("Error calling reportRecognizedSpeech.", e);
    }
  }

  /**
   * Message received when an agent is connected to Unity to/from DIARC socket.
   * 
   * All "connected" messages will contain the following:
   * actor = connected agent
   * 
   * @param msg - Message with action "connected".
   **/
  protected void connected (Message msg) {
    String agent = msg.arguments.get(0);
    log.info("Connected " + agent + " to Unity");
    isConnected = true;
  }

  /**
   * Send a message to Unity via the unitySendMessage TRADE
   * service that proxies messages to the Unity socket server.
   * 
   * @param msg - Message to send to Unity as the agent.
   **/
  protected void sendMessage (Message msg) {
    log.info("Agent " + this.agent + " sending msg.action = " + msg.action);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("unitySendMessage").argTypes(Message.class)).call(Boolean.class, msg);
    } catch (TRADEException e) {
      log.error("Error calling unitySendMessage.", e);
    }
  }

  /**
   * Send a message to Unity and await a response.
   * All responses are stored in flat String Lists for now. 
   * 
   * @param msg - Message to send to Unity.
   **/
  public List<String> getResponse (Message msg) {
    List<String> response = null;
    try {
      response = TRADE.getAvailableService(new TRADEServiceConstraints().name("unityGetResponse").argTypes(Message.class)).call(List.class, msg);
    } catch (Exception e) {
      log.error("Error getting response ", e);
    }
    return response;
  }

  /**
   * TRADE service used by the UnityComponent when message received from
   * Unity socket server. If not intended for this instance of agent,
   * ignore message.
   * 
   * @param msg - Message received from Unity.
   **/
  @TRADEService
  public synchronized boolean unityReceiveMessage (Message msg) {
    if (msg.agent.equals(this.agent)) {
      log.info("Agent " + msg.agent + " received message action = " + msg.action);
      route(msg);
    }
    return true;
  }
}
