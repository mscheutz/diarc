/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Set;

public abstract class GameAction {
  protected Logger log;

  protected boolean receivedResponse;
  protected String rawResponse;
  protected int maxResponseWait;

  public GameAction() {
    log = LoggerFactory.getLogger(this.getClass());
    this.receivedResponse = false;
    this.maxResponseWait = 5000;
  }

  public abstract String getCommand();

  public abstract boolean getSuccess();

  public abstract double getStepCost();

  public abstract Set<Predicate> getAssertions();

  protected abstract void parseResponse();

  public abstract boolean canChangeGameState();

  public String getRawResponse() {
    if (this.receivedResponse) {
      return this.rawResponse;
    } else {
      throw new RuntimeException("Response Not Received Yet");
    }
  }

  public int getMaxResponseWait() {
    return this.maxResponseWait;
  }

  public void insertResponse(String response) {
    if (this.receivedResponse) {
      throw new RuntimeException("Response Already Received");
    }
    this.receivedResponse = true;
    this.rawResponse = response;
    this.parseResponse();
  }

  protected final void setGameOver(boolean goalAchieved) {
    log.info("Game Over in received Polycraft message.");
    TRADE.getAvailableServices(new TRADEServiceConstraints().name("setGameOver").argTypes(Boolean.class)).forEach(x -> {
      try {
        x.call(void.class,goalAchieved);
      } catch (TRADEException e) {
        log.error("[setGameOver] Error attempting to set game over.", e);
      }
    });
  }

}
