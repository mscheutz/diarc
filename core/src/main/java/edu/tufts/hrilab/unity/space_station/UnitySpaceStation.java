/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.unity.UnityAgent;
import edu.tufts.hrilab.unity.message.*;

public class UnitySpaceStation extends UnityAgent {

  static private Logger log = LoggerFactory.getLogger(UnitySpaceStation.class);

  public UnitySpaceStation() {
    super();
  }

  /**
   * Sends a health or tubeHealth message to Unity and
   * awaits the response.
   * 
   * Value is index 0 in response message
   * response : ["100"]
   */
  protected float healthResponse (Message msg) {
    List<String> res = null;
    try {
      res = getResponse(msg);
    } catch (Exception e) {
      log.error("Error sending " + msg.action + " message", e);
    }
    if (res == null || res.size() != 1) {
      log.error("Invalid response from space station re: " + msg.action);
      return 0f;
    }
    return Float.parseFloat(res.get(0));
  }
  protected float healthResponse () {
    Message msg = new Message(this.agent, "health");
    return healthResponse(msg);
  }
  protected float healthResponse (Symbol tube) {
    String tubeName = tube.toString();
    Message msg = new Message(this.agent, "tubeHealth", tubeName);
    return healthResponse(msg);
  }

  /**
   * Sends a tubesDamaged, tubesBroken or tubesOff message to Unity
   * and awaits the response.
   * 
   * Response is list of strings containing all matching tubes
   */
  protected List<Symbol> tubesResponse (Message msg) {
    List<Symbol> tubes = new ArrayList<Symbol>();
    List<String> res = null;
    Symbol tube;
    try {
      res = getResponse(msg);
    } catch (Exception e) {
      log.error("Error sending " + msg.action + " message", e);
    }
    if (res == null) {
      log.error("Invalid response from space station re: " + msg.action);
    } else {
      for (String tubeName : res) {
        tube = Factory.createSymbol(tubeName);
        tubes.add(tube);
      }
    }
    return tubes;
  }
  protected List<Symbol> tubesResponse (String action) {
    Message msg = new Message(this.agent, action);
    return tubesResponse(msg);
  }
  protected List<Symbol> tubesResponse (String action, Symbol area) {
    String areaName = area.toString();
    Message msg = new Message(this.agent, action, areaName);
    return tubesResponse(msg);
  }

  /**
   * Sends a tubeDamaged, tubeBroken or tubeOff message requesting
   * the state of a specific tube.
   * 
   * Response is in form ["name", "true"] where index 0 is the name of
   * the requested state and index 1 is the stringified boolean of the state.
   */
  protected boolean tubeStateResponse (String action, Symbol tube) {
    String tubeName = tube.toString();
    Message msg = new Message(this.agent, action, tubeName);
    List<String> res = null;
    try {
      res = getResponse(msg);
    } catch (Exception e) {
      log.error("Error sending " + action + " message", e);
    }
    if (res == null || res.size() != 2) {
      log.error("Invalid response from space station re: " + action);
      return false;
    }
    log.info(action + " " + tubeName + " " + res.get(1));
    return (res.get(1).toLowerCase()).equals("true");
  }

  /**
   * Gets the health of the space station as a float [0:100].
   */
  @TRADEService
  @Action
  public synchronized float getSpaceStationHealth () {
    return healthResponse();
  }

  /**
   * Gets a list of damaged tubes, optionally limited to the area.
   */
  @TRADEService
  @Action
  public synchronized List<Symbol> getSpaceStationTubesDamaged () {
    return tubesResponse("tubesDamaged");
  }
  @TRADEService
  @Action
  public synchronized List<Symbol> getSpaceStationTubesDamaged (Symbol area) {
    return tubesResponse("tubesDamaged", area);
  }

  /**
   * Gets a list of broken tubes, optionally limited to the area.
   */
  @TRADEService
  @Action
  public synchronized List<Symbol> getSpaceStationTubesBroken () {
    return tubesResponse("tubesBroken");
  }
  @TRADEService
  @Action
  public synchronized List<Symbol> getSpaceStationTubesBroken (Symbol area) {
    return tubesResponse("tubesBroken", area);
  }

  /**
   * Gets a list of off tubes, optionally limited to a area.
   */
  @TRADEService
  @Action
  public synchronized List<Symbol> getSpaceStationTubesOff () {
    return tubesResponse("tubesOff");
  }
  @TRADEService
  @Action
  public synchronized List<Symbol> getSpaceStationTubesOff (Symbol area) {
    return tubesResponse("tubesOff", area);
  }

  /**
   * Gets the health of a specific tube via name as a float [0:100].
   */
  @TRADEService
  @Action
  public synchronized float getSpaceStationTubeHealth (Symbol tube) {
    return healthResponse(tube);
  }

  /**
   * Gets the damaged state of a tube, true if damaged.
   */
  @TRADEService
  @Action
  public synchronized boolean getSpaceStationTubeDamaged (Symbol tube) {
    return tubeStateResponse("tubeDamaged", tube);
  }

  /**
   * Gets the broken state of a tube, true if broken.
   */
  @TRADEService
  @Action
  public synchronized boolean getSpaceStationTubeBroken (Symbol tube) {
    return tubeStateResponse("tubeBroken", tube);
  }

  /**
   * Gets the off state of a tube, true if off.
   */
  @TRADEService
  @Action
  public synchronized boolean getSpaceStationTubeOff (Symbol tube) {
    return tubeStateResponse("tubeOff", tube);
  }
}