/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.planner.pddl.PddlGenerator;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.pddl.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import com.google.gson.Gson;
import edu.tufts.hrilab.pddl.Domain;
import edu.tufts.hrilab.pddl.Pddl;
import edu.tufts.hrilab.pddl.Type;
import edu.tufts.hrilab.polycraft.recovery.util.ExploreUtils;
import edu.tufts.hrilab.socket.SocketConnection;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class PolycraftRLComponent extends DiarcComponent {
  private SocketConnection rlSocket = null;
  private int port = 6013;
  /**
   * The set of actions that exist at startup (i.e., non-novel actions). Used to
   * identify novel actions during RL execution.
   */
  private Set<String> originalActionSet = new HashSet<>();
  /**
   * The last response returned by RL.
   */
  private String lastResponse;

  /**
   * Used to exit out of while loop in executeRL.
   */
  private volatile boolean isGameOver = false;

  /**
   * Used to prevent recursive RL episodes.
   */
  private volatile AtomicBoolean currentlyExploringRL = new AtomicBoolean(false);

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("port")) {
      port = Integer.parseInt(cmdLine.getOptionValue("port"));
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("port").hasArg().argName("port").desc("Set the RL agent port. (Default=" + port + ")").build());
    return options;
  }

  public void init() {
    log.info("Connecting to RL");
    try {
      rlSocket = new SocketConnection(port);
    } catch (IOException e) {
      log.warn("RL Socket failed to open.", e);
    }

    // start thread to wait for GM to be ready, and then collect initial action set
    new Thread(() -> collectOriginalActionSet()).start();
  }

  public void collectOriginalActionSet() {
    // wait for submitGoal to be available in the system
    log.info("Waiting for TRADE Service for submitGoal to be available ...");
    TRADEServiceInfo tsi = null;
    while (tsi == null) {
      try {
        tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal"));
      } catch (TRADEException e) {
        Util.Sleep(100);
      }
    }
    log.info("TRADE Service for submitGoal found. Starting game loop.");

    // create dummy goal in order to create PDDL domain and problem (only care about the domain here)
    Goal dummyGoal = new Goal(Factory.createSymbol("self"), Factory.createPredicate("plan()"), Observable.DEFAULT);
    List<Term> facts;
    try {
      facts = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFacts")).call(List.class);
    } catch (TRADEException e) {
      log.error("Error getting all facts from belief to seed RL actions.", e);
      return;
    }
    PddlGenerator pddlTest = new PddlGenerator(dummyGoal, new ActionConstraints(), facts, new ArrayList<>());
    Pddl pddl = pddlTest.build();

    Domain domain = pddl.getDomain();
    Domain nonparameterizedDomain = generateNonParameterizedDomain(domain);
    nonparameterizedDomain.getActions().forEach(action -> originalActionSet.add(action.getName()));
  }

  public class Initialize {
    GameState state;
    String domain;
    String plan;
    Set<String> novelActions;
  }

  public class GameState {
    Inventory inventory;
    Player player;
    Map<String, String> map;
    String action;
    boolean actionSuccess;
    String failedAction;
  }

  public class Inventory {
    Item[] slots;
    String selectedItem;
  }

  public class Item {
    String item;
    int count;
  }

  public class Player {
    int[] pos;
    String facing;
  }

  private Action createNewAction(Action a, String to_replace, Symbol item, String to_dist) {
    String name = a.getName();
    if (!item.getName().equals("none")) {
      name = name + "_" + item.getName();
    }
    if (to_dist.equals("two")) {
      name = name + "_bytwo";
    }
    Action.Builder builder = new Action.Builder(name);
    Map<Variable, Symbol> bindings = new HashMap<>();
    for (Symbol p : a.getParameters()) {
      if (p.getName().equals("?actor")) {
        builder.addParameter(p.getName(), p.getType());
      } else if (p.getName().equals(to_replace)) {
        bindings.put(new Variable(to_replace, p.getType()), item);
      } else if (p.getName().equals("?to_dist")) {
        bindings.put((Variable) p, new Symbol(to_dist));
      } else {
        try {
          //replace variable in preconditions and effects (adding more actions if necessary) unless variable starts with from or facing
          if (!p.getName().startsWith("?from") && !p.getName().startsWith("?facing")) {
            if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("object", p.getType(), p.getType()))) {
              bindings.put(new Variable(p.getName(), p.getType()), new Symbol(p.getType()));
              continue;
            }
          }
          builder.addParameter(p.getName(), p.getType());
        } catch (TRADEException e) {
          log.error("Error calling querySupport:", e);
        }
      }
    }
    for (Predicate pc : a.getPreconditions()) {
      builder.addPrecondition(pc.copyWithNewBindings(bindings));
    }
    for (Predicate e : a.getEffects()) {
      builder.addEffect(e.copyWithNewBindings(bindings));
    }
    return builder.build();
  }

  @TRADEService
  public String initializeRL(String failedAction) {
    Initialize init = new Initialize();

    // reset game over flag -- TODO: is there a better place for this?
    isGameOver = false;

    // set most recent plan (could be empty)
    try {
      init.plan = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrentPlan")).call(String.class);
    } catch (Exception e) {
      log.error("Error getting current plan: ", e);
      return "GIVEUP";
    }

    // set un-parameterized PDDL domain
    Domain parameterizedDomain;
    try {
      parameterizedDomain = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrentDomain")).call(Domain.class);
    } catch (Exception e) {
      log.error("Error getting current domain: ", e);
      return "GIVEUP";
    }
    Domain nonParameterizedDomain = generateNonParameterizedDomain(parameterizedDomain);
    init.domain = nonParameterizedDomain.generate("diarcdomain");
    //log.info(init.domain);

    // set novel actions
    Set<String> novelActions = new HashSet<>();
    nonParameterizedDomain.getActions().forEach(action -> {
      if (!originalActionSet.contains(action.getName())) {
        // HACK: never consider trades and interact_with as novel actions
        if (!action.getName().startsWith("trade") && !action.getName().startsWith("interact")) {
          novelActions.add(action.getName());
        }
      }
    });
    init.novelActions = novelActions;

    // set game state
    init.state = getState(failedAction, failedAction, false);

    // generate JSON and send to RL
    String json = new Gson().toJson(init);
    //log.info(json);
    if (rlSocket != null && rlSocket.isConnected()) {
      rlSocket.sendCommand(json);
      while (true) {
        if (!rlSocket.isConnected()) {
          log.warn("Socket closed unexpectedly.");
          return "GIVEUP";
        }
        //Rl should catch any gameOver in the response json, so there's no need
        //  to track trial time here. If RL crashes, the socket will go down
        //  and we will exit.
        long maxTime = 6000;
        String res = rlSocket.waitedResponse((int) maxTime);
        //log.info("response: "+res);
        lastResponse = processResponse(res);

        //log.info(lastResponse);
        return lastResponse;
      }
    }

    return "GIVEUP";
  }

  /**
   * Convert a PDDL Domain, possibly containing parameterized actions, into a PDDL Domain
   * that contains only non-parameterized actions.
   *
   * @param parameterizedDomain
   * @return
   */
  private Domain generateNonParameterizedDomain(Domain parameterizedDomain) {
    try {
      List<Action> actions = parameterizedDomain.getActions();
      List<Symbol> heldArgs = new ArrayList<>();
      for (Action a : actions) {
        List<Predicate> preconditions = a.getPreconditions();
        //if parameter is holding X, add to list of holding parameters
        for (Predicate p : preconditions) {
          if (p.getName().equals("holding")) {
            Symbol item = p.get(1);
            if (!heldArgs.contains(item) && !item.getName().equals("air") && !item.isVariable()) {
              heldArgs.add(item);
            }
          } else if (p.getName().equals("not")) {
            Term arg = (Term) p.get(0);
            if (arg.getName().equals("holding")) {
              Symbol item = arg.get(1);
              if (!heldArgs.contains(item) && !item.getName().equals("air") && !item.isVariable()) {
                heldArgs.add(item);
              }
            }
          }
        }
      }

      List<Action> newactions = new ArrayList<>();
      for (Action a : actions) {
        //if parameter is not actor, remove it (unless deselect or break_and_pickup)
        //for select, create new action for each holding parameter
        if (a.getName().equals("select")) {
          for (Symbol item : heldArgs) {
            Term query = Factory.createPredicate("object", item, item);
            if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, query)) {
              query = Factory.createPredicate("typeobject", new Variable("X"), item);
              List<Map<Variable, Symbol>> results = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
              for (Map<Variable, Symbol> m : results) {
                newactions.add(createNewAction(a, "?obj", m.get("X"), "one"));
              }
            } else {
              if (!item.getName().startsWith("?") && !item.equals("air")) {
                newactions.add(createNewAction(a, "?obj", item, "one"));
              }
            }
          }
        } else if (!a.getName().equals("place") && !a.getName().equals("delete_item")) {
          //get rid of place
          if (!a.getName().startsWith("approach")) {
            if (a.getName().startsWith("deselect") || a.getName().startsWith("pickup")) {
              newactions.add(createNewAction(a, "placeholder", new Symbol("none"), "one"));
            } else {
            /*
            //replace to_dist with one for objects and two for actors (but also two for log)
            List<Map<Variable, Symbol>> argoptions;
            if (a.getName().equals("approach_actor")) {
              Term query = Factory.createPredicate("typeobject(X,actor)");
              argoptions = (List<Map<Variable, Symbol>>) TRADE.callThe("queryBelief", query);
              for (Map<Variable, Symbol> option : argoptions) {
                if (!option.get(new Variable("X")).getName().equals("self")) {
                  newactions.add(createNewAction(a, "?to_actor", option.get(new Variable("X")), "two"));
                }
              }
            } else if (a.getName().equals("approach_object")) {
              Term query = Factory.createPredicate("typeobject(X,physobj)");
              argoptions = (List<Map<Variable, Symbol>>) TRADE.callThe("queryBelief", query);
              HashSet<Symbol> args = new HashSet<>();
              for (Map<Variable, Symbol> option : argoptions) {
                Symbol item = option.get(new Variable("X"));
                args.add(item);
              }
              for (Symbol arg : args) {
                // EAK: want to create approach_object actions for all known object types, not just
                // objects that have count >=1 in the world
//                query = Factory.createPredicate("fluent_geq(world," + arg + ",1)");
//                if ((boolean) TRADE.callThe("querySupport", query)) {
                  if ((boolean) TRADE.callThe("querySupport", Factory.createPredicate("type", arg, new Symbol("log")))) {
                    newactions.add(createNewAction(a, "?to_obj", arg, "two"));
                  }
                  newactions.add(createNewAction(a, "?to_obj", arg, "one"));
//                }
              }
            }
          } else {
             */
              List<Variable> parameters = a.getParameters();
              Map<String, List<Symbol>> map = new HashMap<>();
              List<Predicate> preconditions = a.getPreconditions();


                for (Symbol symbol : parameters) {
                  List<Symbol> args = new ArrayList<>();
                  if (!symbol.getName().equals("?actor") && !symbol.getName().startsWith("?facing")) {
                    //for each remainign parameter, create new action adding argument to name (based on type)
                    Term query;
                    //if (symbol.getType().equals("breakable") || symbol.getType().equals("placeable")) {
                    //  query = Factory.createPredicate("typeobject(X,physobj)");
                    //} else {
                    boolean appendArgs = true;
                    for (Predicate p : preconditions) {
                      if (p.getName().equals("equals")) {
                        Symbol arg = p.getArgs().get(1);
                        //log.info("Additional arg is "+arg.toString());
                        if (a.getName().endsWith(arg.toString())) {
                          //log.info("Not appending additional args");
                          query = Factory.createPredicate("type", arg.toString(), symbol.getType());
                          //argument in name is of parameter type
                          //log.info(query);
                          boolean correctType = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, query);
                          if (correctType) {
                            //log.info("adding "+arg+" to end of name");
                            appendArgs = false;
                            args.add(arg);
                            break;
                          }
                        }
                      }
                    }
                    if (appendArgs) {
                    query = Factory.createPredicate("typeobject(X," + symbol.getType() + ")");
                    List<Map<Variable, Symbol>> argoptions = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
                    if (argoptions.size() > 1) {
                      for (Map<Variable, Symbol> option : argoptions) {
                        Symbol item = option.get(new Variable("X"));
                        if (!args.contains(item) && !item.equals("air")) {
                          args.add(item);
                        }
                      }
                    }
                  }
                  if (!args.isEmpty()) {
                    map.put(symbol.getName(), args);
                  }
                }
              }
              if (map.keySet().isEmpty()) {
                newactions.add(createNewAction(a, "placeholder", new Symbol("none"), "one"));
              } else {
                for (String s : map.keySet()) {
                  for (Symbol sym : map.get(s)) {
                    newactions.add(createNewAction(a, s, sym, "one"));
                  }
                }
              }
            }
          }
        }
      }

      Term query = Factory.createPredicate("typeobject(X,agent)");
      List<Map<Variable, Symbol>> results = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query, MemoryLevel.EPISODIC);
      for (Map<Variable, Symbol> m : results) {
        Symbol actor = m.get(new Variable("X"));
        if (!actor.getName().equals("self")) {
          Action.Builder builder = new Action.Builder("interact_with_" + actor.getName());
          builder.addParameter("?actor", "agent");
          builder.addPrecondition(Factory.createPredicate("facing_obj", "?actor", actor.getName(), "two"));
          builder.addEffect(Factory.createPredicate("facing_obj", "?actor", actor.getName(), "two"));
          newactions.add(builder.build());
        }
      }

      Domain nonparamDomain = new Domain();
      for (Predicate func : parameterizedDomain.getFunctions().values()) {
        nonparamDomain.addFunction(func);
      }
      for (String str : parameterizedDomain.getRequirements()) {
        nonparamDomain.addRequirement(str);
      }
      for (Type t : parameterizedDomain.getTypes()) {
        nonparamDomain.addType(t);
      }
      for (Action a : newactions) {
        if (!nonparamDomain.getActions().contains(a)) {
          nonparamDomain.addAction(a);
        }
      }

      return nonparamDomain;

    } catch (Exception e) {
      log.error("Error creating non-parameterized PDDL Domain.", e);
      return null;
    }
  }

  @TRADEService
  public String executeRL(String failedAction, String lastAction, boolean actionResult) {
    GameState state = getState(failedAction, lastAction, actionResult);
    String json = new Gson().toJson(state);
    if (rlSocket != null && rlSocket.isConnected()) {
      rlSocket.sendCommand(json);
      while (true) {
        if (!rlSocket.isConnected()) {
          log.warn("Socket closed unexpectedly.");
          return "GIVEUP";
        }
        //Rl should catch any gameOver in the response json, so there's no need
        //  to track trial time here. If RL crashes, the socket will go down
        //  and we will exit.
        long maxTime = 6000;

        String res = rlSocket.waitedResponse((int) maxTime);
        log.info(res);
        lastResponse = processResponse(res);
        log.info("returning response");
        return lastResponse;
      }
    }
    return "GIVEUP";
  }

  private String processResponse(String response) {
    String res;
    if (response == null) {
      return null;
    } else if (response.equalsIgnoreCase("GIVEUP") || response.equalsIgnoreCase("REPLAN") || response.equalsIgnoreCase("DONE")) {
      return response.toUpperCase();
    } else {
      // construct DIARC action
      boolean bytwo = false;
      String arg = "";
      res = response;
      if (res.endsWith("bytwo")) {
        int index = res.lastIndexOf('_');
        arg = response.substring(index + 1);
        res = res.substring(0, index);
        bytwo = true;
      }

      ActionDBEntry a = Database.getActionDB().getAction(res);
      while (a == null) {
        //no action matches the response we got
        int index = res.lastIndexOf('_');
        if (!bytwo) {
          arg = response.substring(index + 1);
        } else {
          arg = response.substring(index + 1, response.lastIndexOf('_'));
        }
        res = res.substring(0, index);
        a = Database.getActionDB().getAction(res);
      }
      try {
        res = TRADE.getAvailableService(new TRADEServiceConstraints().name("fillAction")).call(String.class, a, arg, bytwo);
      } catch (TRADEException e) {
        log.error("Error filling action",e);
      }
    }
    return res;
  }

  public GameState getState(String failedAction, String action, boolean actionResult) {
    GameState state = new GameState();
    state.actionSuccess = actionResult;
    state.action = action;
    state.failedAction = failedAction;
    Term playerQuery = Factory.createPredicate("at(self,X,Y)");
    Term facingQuery = Factory.createPredicate("facing(self,X)");
    Term inventoryQuery = Factory.createPredicate("fluent_equals(inventory(self,I),C)");
    Term holdingQuery = Factory.createPredicate("holding(self,X)");
    Term atQuery = Factory.createPredicate("at(I,X,Y)");
    try {
      List<Map<Variable, Symbol>> player = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, playerQuery);
      List<Map<Variable, Symbol>> facing = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, facingQuery);
      List<Map<Variable, Symbol>> inventory = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, inventoryQuery);
      List<Map<Variable, Symbol>> holding = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, holdingQuery);
      List<Map<Variable, Symbol>> map = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, atQuery);

      state.player = new Player();
      state.player.pos = new int[2];
      if (!player.isEmpty()) {
        state.player.pos[0] = Integer.parseInt(player.get(0).get(new Variable("X")).toString());
        state.player.pos[1] = Integer.parseInt(player.get(0).get(new Variable("Y")).toString());
      }
      if (!facing.isEmpty()) {
        state.player.facing = facing.get(0).get(new Variable("X")).toString();
      }
      state.inventory = new Inventory();
      if (!holding.isEmpty()) {
        state.inventory.selectedItem = holding.get(0).get(new Variable("X")).toString();
      }
      state.inventory.slots = new Item[inventory.size()];

      for (int i = 0; i < inventory.size(); i++) {
        Item item = new Item();
        item.item = inventory.get(i).get(new Variable("I")).toString();
        item.count = (int) Double.parseDouble(inventory.get(i).get(new Variable("C")).toString());
        state.inventory.slots[i] = item;
      }

      state.map = new HashMap<>();
      for (Map<Variable, Symbol> m : map) {
        String loc = m.get(new Variable("X")).toString();
        loc = loc + "," + m.get(new Variable("Y")).toString();
        String item = m.get(new Variable("I")).getName();
        state.map.put(loc, item);
      }
    } catch (TRADEException e) {
      log.error("Error calling belief to initialize RL Component:", e);
    }
    return state;
  }

  /**
   * Notify RL when a game is over, and also set isGameOver flag to
   * exit out of local RL execution loop.
   *
   * @param goalAchieved
   */
  @TRADEService
  public void setGameOver(boolean goalAchieved) {
    isGameOver = true;
    String message_request = "RESET " + goalAchieved;

    if (rlSocket != null) {
      rlSocket.sendCommand(message_request);
      rlSocket.waitedResponse(5000);
    }
  }

  /**
   * Main RL entry point for DIARC.
   * <p>
   * NOTE: this should ALWAYS return a TRUE justification to indicate that
   * RL was successfully attempted. Otherwise, goal recovery can enter infinitely nested
   * RL episodes (i.e., executeRL(executeRL(exuteRL(...))).
   */
  @TRADEService
  @edu.tufts.hrilab.action.annotations.Action
  public Justification executeRL(Symbol actor, Predicate brokenActionPredicate, Symbol brokenActionStatus, Predicate failurePredicate) {
    if (!currentlyExploringRL.compareAndSet(false, true)) {
      log.warn("Not executing nested RL.");
      return new ConditionJustification(true);
    }

    try {
      // don't enter RL for actions that failed as part of symbolic exploration
      if (TRADE.getAvailableService(new TRADEServiceConstraints().name("isCurrentlyExploring")).call(boolean.class)) {
        log.warn("Not executing RL while performing symbolic exploration.");
        return new ConditionJustification(true);
      }

      // report novelty every time we enter RL. This is bc we only go to RL when we're out of symbol options.
      Predicate novelty = Factory.createPredicate("unknown", "startingRL");
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportNovelties")).call(Object.class, Stream.of(novelty).collect(Collectors.toSet()));

      // get action status
      ActionStatus actionStatus = ActionStatus.fromString(brokenActionStatus.toString().toUpperCase());

      // get failedAction
      String failedAction = null;
      if (actionStatus == ActionStatus.FAIL_NOTFOUND) {
        // i.e., can't plan
        failedAction = "cannotplan";
      } else if (actionStatus == ActionStatus.FAIL_RETURNVALUE) {
        // primitive action failed
        failedAction = brokenActionPredicate.toString();
      }

      if (failedAction != null) {
        // call RL
        log.warn("Attempting RL to solve broken action: " + brokenActionPredicate);
        String response = initializeRL(failedAction);
        int checkGameOverCounter = 0;
        while (!isGameOver && response != null && !response.equals("GIVEUP") && !response.equals("DONE")) {
          if (response.equals("REPLAN")) {
            boolean canPlan = TRADE.getAvailableService(new TRADEServiceConstraints().name("canPlan")).call(boolean.class, actor, brokenActionPredicate);
            response = executeRL(failedAction, response, canPlan);

            // TODO: REMOVE this termination criteria once RL returns DONE
            if (actionStatus == ActionStatus.FAIL_RETURNVALUE) {
              break;
            } else if (actionStatus == ActionStatus.FAIL_NOTFOUND && canPlan) {
              break;
            }
          } else {
            // must be next action to take
            Justification rljustification = ExploreUtils.doGoal(Factory.createPredicate(response));
            response = executeRL(failedAction, response, rljustification.getValue());
          }

          // occasionally check if the game is over. this is necessary when no action is ever executed (e.g., can never plan)
          if (++checkGameOverCounter > 30) {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("nop")).call(Object.class);
            checkGameOverCounter = 0;
          }
        }

        log.warn("Done attempting RL to solve broken action: " + brokenActionPredicate);
      } else {
        log.warn("Not executing RL for actionStatus: " + actionStatus);
      }
    } catch (TRADEException e) {
      log.error("Error executing RL.", e);
    } finally {
      currentlyExploringRL.set(false);
      return new ConditionJustification(true);
    }
  }

}


