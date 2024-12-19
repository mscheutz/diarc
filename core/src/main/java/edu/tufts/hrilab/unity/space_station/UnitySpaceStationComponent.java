/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.unity.UnityAgentComponent;
import edu.tufts.hrilab.unity.message.*;

public class UnitySpaceStationComponent extends UnityAgentComponent implements UnitySpaceStationInterface {

  static private Logger log = LoggerFactory.getLogger(UnitySpaceStationComponent.class);

  public UnitySpaceStationComponent() {
    super();
  }

  /**
   * Sends a health or tubeHealth message to Unity and
   * awaits the response.
   *
   * Value is index 0 in response message
   * response : ["100"]
   */
  protected float healthResponse(Message msg) {
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

  protected float healthResponse() {
    Message msg = new Message(this.agent, "health");
    return healthResponse(msg);
  }

  protected float healthResponse(Symbol tube) {
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
  protected List<Symbol> tubesResponse(Message msg) {
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

  protected List<Symbol> tubesResponse(String action) {
    Message msg = new Message(this.agent, action);
    return tubesResponse(msg);
  }

  protected List<Symbol> tubesResponse(String action, Symbol area) {
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
  protected boolean tubeStateResponse(String action, Symbol tube) {
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
   * {@inheritDoc}
   */
  @Override
  public synchronized float getSpaceStationHealth() {
    return healthResponse();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized List<Symbol> getSpaceStationTubesDamaged() {
    return tubesResponse("tubesDamaged");
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized List<Symbol> getSpaceStationTubesDamaged(Symbol area) {
    return tubesResponse("tubesDamaged", area);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized List<Symbol> getSpaceStationTubesBroken() {
    return tubesResponse("tubesBroken");
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized List<Symbol> getSpaceStationTubesBroken(Symbol area) {
    return tubesResponse("tubesBroken", area);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized List<Symbol> getSpaceStationTubesOff() {
    return tubesResponse("tubesOff");
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized List<Symbol> getSpaceStationTubesOff(Symbol area) {
    return tubesResponse("tubesOff", area);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized float getSpaceStationTubeHealth(Symbol tube) {
    return healthResponse(tube);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized boolean getSpaceStationTubeDamaged(Symbol tube) {
    return tubeStateResponse("tubeDamaged", tube);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized boolean getSpaceStationTubeBroken(Symbol tube) {
    return tubeStateResponse("tubeBroken", tube);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized boolean getSpaceStationTubeOff(Symbol tube) {
    return tubeStateResponse("tubeOff", tube);
  }
}