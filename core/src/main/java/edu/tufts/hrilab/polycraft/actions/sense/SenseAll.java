/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import java.util.*;

import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.BlockInFront;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Entities;
import edu.tufts.hrilab.polycraft.msg.Goal;
import edu.tufts.hrilab.polycraft.msg.Inventory;
import edu.tufts.hrilab.polycraft.msg.InventoryItem;
import edu.tufts.hrilab.polycraft.msg.MapBlock;
import edu.tufts.hrilab.polycraft.msg.Player;

public class SenseAll extends Sense {
  protected SenseAll.ParsedResponse parsedResponse;
  protected edu.tufts.hrilab.polycraft.msg.Map map;
  protected Entities entities;
  protected Inventory inventory;

  protected class ParsedResponse {
    // world state
    BlockInFront blockInFront;
    Map<String, InventoryItem> inventory;
    Player player;
    int[] destinationPos;
    Map<String, Entities.Entity> entities;
    Map<String, MapBlock> map;

    // supplemental game info
    Goal goal;
    CommandResult command_result;
    int step;
    boolean gameOver;
  }

  @Override
  public String getCommand() {
    return "SENSE_ALL NONAV";
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
    this.parsedResponse = new Gson().fromJson(this.rawResponse, SenseAll.ParsedResponse.class);

    // populate helper classes that can't be populated automatically by gson (e.g., classes that are just containers for Lists and Maps)
    map = new edu.tufts.hrilab.polycraft.msg.Map(parsedResponse.map);
    entities = new Entities(parsedResponse.entities);
    inventory = new Inventory(parsedResponse.inventory);

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
    assertions.addAll(parsedResponse.goal.generateAssertions()); //novelty signal
    assertions.addAll(parsedResponse.blockInFront.generateAssertions()); //block in front
    assertions.addAll(parsedResponse.player.generateAssertions()); //player location and direction
    assertions.addAll(inventory.generateAssertions()); // inventory
    assertions.addAll(map.generateAssertions());  //items on map
    assertions.addAll(entities.generateAssertions());  //entities
    assertions.addAll(parsedResponse.command_result.generateAssertions()); // action cost
    return assertions;
  }

  @Override
  public Set<Predicate> getLocationAssertions(boolean includeAir) {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(parsedResponse.player.generateAssertions()); //player location and direction
    assertions.addAll(map.generateMapAssertions(includeAir));  //items on map
    assertions.addAll(entities.generateAssertions());  //entities
    return assertions;
  }

}
