/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.BlockInFront;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Entities;
import edu.tufts.hrilab.polycraft.msg.Goal;
import edu.tufts.hrilab.polycraft.msg.InventoryItem;
import edu.tufts.hrilab.polycraft.msg.MapBlock;
import edu.tufts.hrilab.polycraft.msg.Player;

import java.util.*;

public class SenseLocation extends Sense {
  protected SenseLocation.ParsedResponse parsedResponse;

  protected class ParsedResponse {
    // world state
    BlockInFront blockInFront;
    Map<String, InventoryItem> inventory;
    Player player;
    int[] destinationPos;
    Map<String, Entities.Entity> entities;
    Map<String, MapBlock> map;

    Goal goal;
    CommandResult command_result;
    int step;
    boolean gameOver;
  }

  @Override
  public String getCommand() {
    return "SENSE_LOCATIONS";
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
    this.parsedResponse = new Gson().fromJson(this.rawResponse, SenseLocation.ParsedResponse.class);

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
    assertions.addAll(parsedResponse.player.generateAssertions());
//        assertions.addAll(generateBlockAssertions());  //item locations
    return assertions;
  }
}
