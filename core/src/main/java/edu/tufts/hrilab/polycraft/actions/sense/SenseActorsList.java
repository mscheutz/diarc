/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.ActorActionNoState;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Goal;
import edu.tufts.hrilab.polycraft.msg.WorldState;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class SenseActorsList extends Sense {
  protected SenseActorsList.ParsedResponse parsedResponse;

  protected class ParsedResponse {
    List<ActorActionNoState> actorActions = new ArrayList<>();
    Goal goal;
    CommandResult command_result;
    int step;
    boolean gameOver;
  }

  @Override
  public String getCommand() {
    return "SENSE_ACTOR_ACTIONS LIST";
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
    log.info(this.rawResponse);
    this.parsedResponse = new Gson().fromJson(this.rawResponse, SenseActorsList.ParsedResponse.class);
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
      throw new RuntimeException("Wrong Response Received");
    }
  }

  @Override
  public Set<Predicate> getAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    parsedResponse.actorActions.forEach(action -> assertions.add(new Predicate("actorActions",new Symbol(Integer.toString(action.entityID)),new Predicate("actions",new ArrayList<>(action.generateActorAssertions())))));
    //assertions.addAll(parsedResponse.goal.generateAssertions()); // novelty signal
    //assertions.addAll(parsedResponse.command_result.generateAssertions()); // action cost
    return assertions;
  }

}
