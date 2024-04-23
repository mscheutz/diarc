/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.CommandResult;
import edu.tufts.hrilab.polycraft.msg.Goal;
import edu.tufts.hrilab.polycraft.msg.RecipeItem;

import java.util.*;

public class Interact extends Active {
  protected Interact.ParsedResponse parsedResponse;
  protected String entityID;

  public Interact(String entityID) {
    this.entityID = entityID;
  }

  public class ParsedResponse {
    Trade trades;
    CommandResult command_result;
    Goal goal;
    int step;
    boolean gameOver;

    public class Trade {
      TradesAvailable[] trades;
    }

    public class TradesAvailable {
      RecipeItem[] inputs;
      RecipeItem[] outputs;
    }

  }

  public class TradeOption {
    public Map<String, Integer> input;
    public Map<String, Integer> output;
    public String traderID;

    public TradeOption(ParsedResponse.TradesAvailable t, String traderID) {
      input = new HashMap<>();
      output = new HashMap<>();
      for (RecipeItem i : t.inputs) {
        input.put(i.Item, i.stackSize);
      }
      for (RecipeItem i : t.outputs) {
        output.put(i.Item, i.stackSize);
      }
      this.traderID = traderID;
    }
  }

  public List<TradeOption> getTradeOption() {
    List<TradeOption> result = new ArrayList<>();
    if (this.parsedResponse.trades == null || this.parsedResponse.trades.trades == null) {
      return result;
    }
    for (ParsedResponse.TradesAvailable t : this.parsedResponse.trades.trades) {
      result.add(new TradeOption(t, this.parsedResponse.command_result.argument));
    }
    return result;
  }

  @Override
  public String getCommand() {
    return "INTERACT " + this.entityID; //Interact Trader_ID
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
    this.parsedResponse = new Gson().fromJson(this.rawResponse, Interact.ParsedResponse.class);
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
    assertions.addAll(parsedResponse.command_result.generateAssertions());
    return assertions;
  }

//    '{"trades":{"trades":[{"inputs":[{"Item":"minecraft:diamond","stackSize":18,"slot":0}],"outputs":[{"Item":"polycraft:block_of_platinum","stackSize":1,"slot":5}]},{"inputs":[{"Item":"minecraft:log","stackSize":10,"slot":0}],"outputs":[{"Item":"polycraft:block_of_titanium","stackSize":1,"slot":5}]}]},"goal":{"goalType":"ITEM","goalAchieved":false,"Distribution":"Uninformed"},"command_result":{"command":"INTERACT","argument":"54","result":"SUCCESS","message":"Returned available trades","stepCost":300.0},"step":2,"gameOver":false}\n'

}
