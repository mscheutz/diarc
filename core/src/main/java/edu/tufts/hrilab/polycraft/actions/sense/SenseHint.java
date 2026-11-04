/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Goal;
import edu.tufts.hrilab.polycraft.msg.Hint;
import edu.tufts.hrilab.polycraft.msg.Recipe;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class SenseHint extends Sense {
  protected SenseHint.ParsedResponse parsedResponse;

  protected class ParsedResponse {
    Hint hint = new Hint();
    CommandResult command_result;
    Goal goal;
    int step;
    boolean gameOver;
  }

  public Hint getHint() {
    return parsedResponse.hint;
  }

  @Override
  public String getCommand() {
    return "HINT";
  }

  @Override
  public boolean getSuccess() {
    return this.parsedResponse.command_result.result.equals("SUCCESS");
  }

  @Override
  public double getStepCost() {
    return this.parsedResponse.command_result.stepCost;
  }

  @Override
  protected void parseResponse() {
    this.parsedResponse = new Gson().fromJson(this.rawResponse, SenseHint.ParsedResponse.class);

    if (this.parsedResponse.command_result.result.equals("FAIL")) {
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

  @Override
  public Set<Predicate> getAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    if (parsedResponse.hint!=null) {
      assertions.addAll(parsedResponse.hint.generateAssertions());
    }
    return assertions;
  }

}
