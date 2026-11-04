/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Goal;

import java.util.Set;

public abstract class Active extends GameAction {
  protected ParsedResponse parsedResponse;
  protected String failMessage = "";

  protected class ParsedResponse {
    Goal goal;
    CommandResult command_result;
    int step;
    boolean gameOver;
  }

  @Override
  public boolean getSuccess() {
    if (this.parsedResponse == null) {
      return false;
    } else {
      return this.parsedResponse.command_result.result.equals("SUCCESS");
    }
  }

  @Override
  public double getStepCost() {
    return this.parsedResponse.command_result.stepCost;
  }

  @Override
  public boolean canChangeGameState() {
    return true;
  }

  @Override
  public Set<Predicate> getAssertions() {
    return parsedResponse.command_result.generateAssertions();
  }

  @Override
  protected void parseResponse() {
    this.parsedResponse = new Gson().fromJson(this.rawResponse, ParsedResponse.class);

    if (this.parsedResponse == null) {
      log.error("Could not parse response. Raw response is: " + this.rawResponse);
      return;
    }

    if (this.parsedResponse.command_result.result.equals("FAIL")) {
      failMessage = this.parsedResponse.command_result.message;
      log.info("FAIL message: " + this.parsedResponse.command_result.message);
    }
    if (this.parsedResponse.gameOver) {
      setGameOver(this.parsedResponse.goal.goalAchieved);
    }

    String commandSent;
    if (this.parsedResponse.command_result.argument.equals("")) {
      commandSent = this.parsedResponse.command_result.command;
    } else {
      commandSent = this.parsedResponse.command_result.command + " " + this.parsedResponse.command_result.argument;
    }

    if (!getCommand().equals(commandSent)) {
      throw new RuntimeException("Wrong Response Received: " + this.rawResponse);
    }
  }

  public String getMessage() {
    return this.parsedResponse.command_result.message;
  }

  public String getFailMessage() {
    return failMessage;
  }
}
