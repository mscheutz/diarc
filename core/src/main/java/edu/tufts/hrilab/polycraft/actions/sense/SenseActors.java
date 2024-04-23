/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.ActorAction;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Goal;
import edu.tufts.hrilab.polycraft.msg.WorldState;

import java.util.*;

public class SenseActors extends Sense {
  protected SenseActors.ParsedResponse parsedResponse;

  protected class ParsedResponse {
    List<ActorAction> actorActions = new ArrayList<>();
    Goal goal;
    CommandResult command_result;
    int step;
    boolean gameOver;
  }

  @Override
  public String getCommand() {
    return "SENSE_ACTOR_ACTIONS";
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
    this.parsedResponse = new Gson().fromJson(this.rawResponse, SenseActors.ParsedResponse.class);
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
    parsedResponse.actorActions.forEach(action -> assertions.addAll(action.generateActorAssertions())); // actions other agents took in last time step
    assertions.addAll(getAfterWorldState().generateAssertions()); // most up-to-date world state
    assertions.addAll(parsedResponse.goal.generateAssertions()); // novelty signal
    assertions.addAll(parsedResponse.command_result.generateAssertions()); // action cost
    return assertions;
  }

  @Override
  public Set<Predicate> getLocationAssertions(boolean includeAir) {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(getAfterWorldState().generateLocationAssertions(includeAir));
    return assertions;
  }

  public WorldState getBeforeWorldState() {
    for (ActorAction actorAction : parsedResponse.actorActions) {
      if (actorAction.preWorldState != null) {
        return actorAction.preWorldState;
      }
    }
    return null;
  }

  public WorldState getAfterWorldState() {
    for (ActorAction actorAction : parsedResponse.actorActions) {
      if (actorAction.postWorldState != null) {
        return actorAction.postWorldState;
      }
    }
    return null;
  }

}
