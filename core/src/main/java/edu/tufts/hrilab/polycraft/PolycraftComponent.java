/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.polycraft;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.belief.provers.prolog.Prolog;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.polycraft.actions.*;
import edu.tufts.hrilab.polycraft.actions.sense.*;
import edu.tufts.hrilab.polycraft.recovery.util.ExploreUtils;
import edu.tufts.hrilab.polycraft.msg.Player;
import edu.tufts.hrilab.polycraft.util.Movement;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;
import edu.tufts.hrilab.polycraft.util.TradeRecipe;
import edu.tufts.hrilab.socket.SocketConnection;
import edu.tufts.hrilab.polycraft.actions.Interact;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;

public class PolycraftComponent extends DiarcComponent {
  /**
   * Connection to polycraft/novelgridworlds environment.
   */
  private GamePlay game;
  /**
   * Default polycraft/novelgridworlds port.
   */
  private int gamePort = 9000;

  private boolean useVisionModule = false;
  private int visionNullCounter = 0;
  private SocketConnection visSocket;
  private boolean useGrapModule = false;
  private SocketConnection graphSocket;
  private double episodeNoveltyScore;
  private boolean hints = false;

  /**
   * Local flag to know when to start/stop sending data to graph and vision modules.
   */
  private volatile boolean isGameOver = false;
  /**
   * Local flag to know if we want to use vision to locate novelties
   */
  private boolean localization = false;

  /**
   * To create unique trade_num actions.
   */
  private int tradeCount = 0;

  private String characterize = "";

  public PolycraftComponent() {
    super();
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("gameport")) {
      gamePort = Integer.parseInt(cmdLine.getOptionValue("gameport"));
    }
    if (cmdLine.hasOption("vision")) {
      log.debug("activating vision");
      useVisionModule = true;
      int port = Integer.parseInt(cmdLine.getOptionValue("vision", "6011"));
      try {
        visSocket = new SocketConnection(port);
      } catch (IOException e) {
        log.error("Error connecting to vision port.", e);
      }
    }
    if (cmdLine.hasOption("visionloc")) {
      log.debug("activating vision with localization");
      useVisionModule = true;
      int port = Integer.parseInt(cmdLine.getOptionValue("vision", "6011"));
      try {
        visSocket = new SocketConnection(port);
        localization = true;
      } catch (IOException e) {
        log.error("Error connecting to vision port.", e);
      }
    }
    if (cmdLine.hasOption("graph")) {
      useGrapModule = true;
      log.debug("activating graph");
      int port = Integer.parseInt(cmdLine.getOptionValue("graph", "6012"));
      try {
        graphSocket = new SocketConnection(port);
      } catch (IOException e) {
        log.error("Error connecting to graph port.", e);
      }
    }
    if (cmdLine.hasOption("hint")) {
      log.debug("Looking for hints now");
      hints = true;
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("gameport").hasArg().argName("port").desc("Set the polycraft/novelgridworlds port. (Default=9000)").build());
    options.add(Option.builder("visionloc").hasArg().argName("port").optionalArg(true).desc("Look for visual novelties with localization. Optional port arg.").build());
    options.add(Option.builder("vision").hasArg().argName("port").optionalArg(true).desc("Look for visual novelties. Optional port arg.").build());
    options.add(Option.builder("graph").hasArg().argName("port").optionalArg(true).desc("Look for graph novelties. Optional port arg.").build());
    options.add(Option.builder("hint").hasArg().argName("hint").optionalArg(true).desc("Look for hints. Optional port arg.").build());
    return options;

  }

  @Override
  protected void init() {
    game = new edu.tufts.hrilab.polycraft.GamePlay(gamePort);
  }

  /**
   * This initializes the map and belief based on the initial state of the game. Also detects and returns
   * a set of detected pre-game novelties.
   *
   * @return
   */
  @TRADEService
  public Set<Term> initializeBeliefs() {
    // reset local game state
    isGameOver = false;
    game.reset();
    if (localization && episodeNoveltyScore > 0) {
      if (episodeNoveltyScore > 0.006) {
        //selecting the highest novel score of the episode to determine novelty
        Predicate novelty = Factory.createPredicate("vision", "object");
        reportNovelties(Stream.of(novelty).collect(Collectors.toSet()));
      }
    }
    episodeNoveltyScore = 0;

    // remove the previous trade actions.
    for (int count = 0; count < tradeCount; ++count) {
      String actionName = "trade_" + count;
      ActionDBEntry action = Database.getActionDB().getAction(actionName);
      if (action != null) {
        Database.getInstance().removeActionDBEntry(action);
      }
    }
    tradeCount = 0; // reset the counter to make new trade actions.

    // reset beliefs
    try {
      // clears WORKING and EPISODIC memory
      TRADE.getAvailableService(new TRADEServiceConstraints().name("clear")).call(Object.class, MemoryLevel.WORKING);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("clear")).call(Object.class, MemoryLevel.EPISODIC);
    } catch (TRADEException e) {
      log.error("Cannot reset beliefs.", e);
    }

    // get senseall assertions
    getCurrentGameState(); // internally calls SenseAll and populates gamePlay's prover and lastSenseAll
    Set<Predicate> gameState = new HashSet<>();
    Sense senseAction = game.getLastSenseAction();
    gameState.addAll(senseAction.getAssertions());

    // TODO: remove this once PAL init bug is fixed
    if (senseAction.getAssertions().isEmpty()) {
      String fileprefix = "emptyInit";
      String outputFile = writeSenseAllJsonToFile(fileprefix);
      log.error("Error it initial SENSE_ALL JSON. Wrote JSON to file: " + outputFile);
    }

    // get senserecipe assertions
    SenseRecipe senseRecipe = new SenseRecipe();
    game.perform(senseRecipe);
    gameState.addAll(senseRecipe.getAssertions());

    // check for novelties (before updating belief state with game state)
    Set<Term> novelties = checkForPreGameNovelties(gameState);

    // TODO: initialize properties for novel objects

    // clear old map and initialize with new environment (after initializing properties for novel objects)
    populateMap(senseAction.getLocationAssertions(true), true);

    // update belief with game state
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, gameState, MemoryLevel.EPISODIC);
    } catch (TRADEException e) {
      log.error("Error calling assertBeliefs.", e);
    }

    // initialize fluents for known physobj object types to 0
    Set<Predicate> initPreds = new HashSet<>();
    try {
      Predicate objPred = Factory.createPredicate("constant", "X", "Y");
      List<Map<Variable, Symbol>> objects = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, objPred);
      for (Map<Variable, Symbol> m : objects) {
        Symbol s = m.get(new Variable("X"));

        // only want objects that are physobj type
        Predicate physobjPred = Factory.createPredicate("type", s.getName(), "physobj");
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, physobjPred, MemoryLevel.EPISODIC)) {
          // if condition to check whether the predicated below exists or not. X
          if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("fluent_equals(inventory(self," + s + "), X)"), MemoryLevel.EPISODIC)) {
            initPreds.add(Factory.createPredicate("fluent_equals(inventory(self," + s + "), 0)"));
          }
          if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("fluent_equals(world(" + s + "), X)"), MemoryLevel.EPISODIC)) {
            initPreds.add(Factory.createPredicate("fluent_equals(world(" + s + "), 0)"));
          }
        }
      }
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, initPreds, MemoryLevel.EPISODIC);
    } catch (TRADEException e) {
      log.error("Cannot initialize beliefs.", e);
    }

    if (hints) {
      try {
        if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("novelty(X,Y)"))) {
          SenseHint senseHint = new SenseHint();
          game.perform(senseHint);
          if (senseHint.getSuccess()) {
            Set<Predicate> hints = senseHint.getAssertions();
            if (!hints.isEmpty()) {
              reportNovelty(0, "Hint revealed novelty");
              TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, hints, MemoryLevel.EPISODIC);
            }
          }
        }
      } catch (TRADEException e) {
        log.error("Error getting hints", e);
      }
    }

    return novelties;
  }

  @Action
  @TRADEService
  public void senseNewMap() {
    // get all location information
    Sense senseAll = new SenseAll();
    game.perform(senseAll);

    Set<Predicate> gameStateWithAir = senseAll.getLocationAssertions(true);
    populateMap(gameStateWithAir, false);

    // update belief with game state
    try {
      Set<Predicate> gameStateWithoutAir = senseAll.getLocationAssertions(false);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, gameStateWithoutAir, MemoryLevel.EPISODIC);
    } catch (TRADEException e) {
      log.error("Error calling assertBeliefs.", e);
    }
  }

  private void populateMap(Set<Predicate> worldState, boolean clearOldMap) {
    if (clearOldMap) {
      try {
        // reset to empty map
        TRADE.getAvailableService(new TRADEServiceConstraints().name("resetMap")).call(Object.class);
      } catch (TRADEException e) {
        log.error("Cannot reset map.", e);
      }
    }

    Variable objVar = Factory.createVariable("OBJ");
    Variable xVar = Factory.createVariable("X");
    Variable yVar = Factory.createVariable("Y");
    List<Map<Variable, Symbol>> atBindings = new ArrayList<>();

    // add all at(obj,x,y) predicate binding options
    Predicate atPredicate = Factory.createPredicate("at", objVar, xVar, yVar);
    for (Predicate state : worldState) {
      if (state.instanceOf(atPredicate)) {
        Map<Variable, Symbol> atBindingOption = new HashMap<>();
        atBindingOption.put(objVar, state.get(0));
        atBindingOption.put(xVar, state.get(1));
        atBindingOption.put(yVar, state.get(2));
        atBindings.add(atBindingOption);
      }
    }

    // after processing all map info, pass it to the PathPlannerComponent
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("updateMap")).call(Object.class, atPredicate, atBindings);
    } catch (TRADEException e) {
      log.error("Error calling updateMap.", e);
    }
  }

  /**
   * Checks for pre-game novelties. This only includes a limited set of novelties that
   * can be detected before game play starts.
   *
   * @param gameStateAssertions
   * @return
   */
  private Set<Term> checkForPreGameNovelties(Set<Predicate> gameStateAssertions) {

    Set<Term> novelties = new HashSet<>();

    // get beliefs
    List<Term> beliefState;
    try {
      beliefState = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFacts")).call(List.class);
    } catch (TRADEException e) {
      log.error("Error calling getFacts while checking for novelties during initialization.", e);
      return novelties;
    }

    // get world state
    Prover gameStateProver = new Prolog();
    gameStateAssertions.forEach(predicate -> gameStateProver.assertBelief(predicate));

    // Check if the world/inventory counts are the same as no-novelty environments
    for (Term bState : beliefState) {
      if (bState.toString().startsWith("init")) {
        Term fluent = (Term) bState.get(0);
        if (bState.toString().contains("container") || bState.toString().contains("totalcost")) {
          // TODO: this is hacky -- generalize this
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, fluent, MemoryLevel.EPISODIC);
          } catch (TRADEException e) {
            log.error("Error asserting expected container fluents while checking for novelties during initialization.", e);
          }
          continue;
        }

        // check if the world state count is the same as the belief state amount
        Variable quantityVar = Factory.createVariable("Q");
        Term fluentQuery = Factory.createPredicate(fluent.getName(), fluent.get(0), quantityVar);
        List<Map<Variable, Symbol>> fluentBindings = gameStateProver.queryBelief(fluentQuery);
        double worldCount;
        if (fluentBindings.isEmpty()) {
          // no explicit fluent_equals gameState --> assume zero count
          worldCount = 0;
          Map<Variable, Symbol> zeroCount = new HashMap<>();
          zeroCount.put(quantityVar, Factory.createSymbol("0"));
          fluentBindings.add(zeroCount);
        } else {
          worldCount = Double.parseDouble(fluentBindings.get(0).get(quantityVar).getName());
        }

        double beliefCount = Double.parseDouble(fluent.get(1).getName());
        if (beliefCount != worldCount) {
          // novelty -- counts don't match
          novelties.add(fluentQuery.copyWithNewBindings(fluentBindings.get(0)));
        }
      }
    }

    for (Predicate gameState : gameStateAssertions) {
      if (!beliefState.contains(gameState)) {
        String gameStateName = gameState.getName();
        String gameStateFirstArg = gameState.getArgs().get(0).toString();
        if (gameStateName.equals("constant")) {
          // object type novelties -- renamed constant for pddl compatibility
          if (gameStateFirstArg.contains("trader") || gameStateFirstArg.contains("pogoist")) {
            // filter out unique identifiers for traders and pogoist e.g., object(trader_XXX, trader_XXX) and object(pogoist_XXX, pogoist_XXX)
            continue;
          }

          // object novelty
          novelties.add(gameState);
        } else if (gameStateName.equals("subtype")) {
          if (gameStateFirstArg.contains("trader") || gameStateFirstArg.contains("pogoist")) {
            // filter out unique identifiers for traders and pogoist e.g., subtype(trader_XXX, trader) and subtype(pogoist_XXX, pogoist)
            continue;
          }
          try {
            if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type", gameState.getArgs()))) {
              // even though subtype isn't explicitly in belief, it's true through the type hierarchy
              continue;
            }
          } catch (TRADEException e) {
            log.error("Exception querying belief type hierarchy.", e);
          }

          // subtype novelty
          novelties.add(gameState);
        } else if (gameStateName.equals("type") || gameStateName.equals("recipe")) {
          novelties.add(gameState);
        } else if (gameStateName.equals("fluent_equals") && gameState.get(0).getName().startsWith("cost_")) {
          try {
            Predicate oldstate = new Predicate("fluent_equals", gameState.get(0), Factory.createVariable("X"));
            List<Map<Variable, Symbol>> believed = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, oldstate);
            if (!believed.isEmpty()) {
              if (!believed.get(0).isEmpty()) {
                log.debug("Oldstate: " + oldstate);
                novelties.add(new Predicate("discrepancy", oldstate.copyWithNewBindings(believed.get(0)), gameState));
              }
            } else {
              novelties.add(gameState);
            }
          } catch (TRADEException e) {
            log.error("Error calling queryBelief while checking for discrepancies during intialization.", e);
          }
        }
      }
    }

    // filter out novelties that have already been reported by (at any level)
    Iterator<Term> noveltyItr = novelties.iterator();
    while (noveltyItr.hasNext()) {
      Term novelty = noveltyItr.next();
      Variable levelVar = Factory.createVariable("X");
      Predicate noveltyQuery = Factory.createPredicate("novelty", novelty, levelVar);
      try {
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(Boolean.class, noveltyQuery)) {
          noveltyItr.remove();
        }
      } catch (TRADEException e) {
        log.error("Error checking if novelty has already been reported.");
      }
    }
    return novelties;
  }

  /**
   * Checks for inventory novelties and at(object,x,y) novelties in agent's local area.
   *
   * @return
   */
  @TRADEService
  @Observes({"discrepancy(X,Y)"})
  public List<Map<Variable, Symbol>> observeDiscrepancies(Term state) {
    // assumes state predicate does not have anything bound to variables in advertised predicate
    Variable beliefStateVar = (Variable) state.get(0);
    Variable gameStateVar = (Variable) state.get(1);

    ////////////////////////////////////////////////////////////////////////////////////
    // get inventory discrepancies
    ////////////////////////////////////////////////////////////////////////////////////
    Variable itemVar = Factory.createVariable("ITEM");
    Variable amountVar = Factory.createVariable("AMOUNT");
    Predicate inventoryQuery = Factory.createPredicate("fluent_equals(inventory(self,ITEM),AMOUNT)");

    // get all inventory beliefs
    List<Map<Variable, Symbol>> beliefInventory;
    try {
      beliefInventory =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, inventoryQuery);
    } catch (TRADEException e) {
      log.error("Error calling getFacts while checking for inventory novelties.", e);
      return new ArrayList<>();
    }

    // check inventory beliefs against game state
    Prover gameState = getCurrentGameState();
    List<Map<Variable, Symbol>> discrepancyResults = new ArrayList<>();
    for (Map<Variable, Symbol> beliefInventoryForItem : beliefInventory) {
      Predicate beliefFluentEquals = inventoryQuery.copyWithNewBindings(beliefInventoryForItem);
      if (!gameState.querySupport(beliefFluentEquals)) {
        // discrepancy -- get discrepancy amounts
        Map<Variable, Symbol> dicrepancyBindings = new HashMap<>();

        // get belief amount
        Symbol beliefAmount = beliefInventoryForItem.get(amountVar);

        // get amount in game
        Map<Variable, Symbol> gameBinding = new HashMap<>();
        gameBinding.put(itemVar, beliefInventoryForItem.get(itemVar));
        Predicate gameQuery = inventoryQuery.copyWithNewBindings(gameBinding); // bind item to make game query
        List<Map<Variable, Symbol>> gameInventoryResults = gameState.queryBelief(gameQuery);
        if (!gameInventoryResults.isEmpty()) {
          // add game amount
          Symbol gameAmount = gameInventoryResults.get(0).get(amountVar);
          gameBinding.put(amountVar, gameAmount);
        } else {
          // polycraft does not return state information for zero inventory items
          if (Double.parseDouble(beliefAmount.getName()) > 0) {
            log.warn("No game state results for inventoryQuery: " + beliefInventoryForItem.get(itemVar));
          }
          continue;
        }

        // TODO: add check for fluent_equals(inventory,...) preds that exist in game state but not belief ??

        // add inventory discrepancy to results
        dicrepancyBindings.put(beliefStateVar, beliefFluentEquals);
        dicrepancyBindings.put(gameStateVar, inventoryQuery.copyWithNewBindings(gameBinding));
        discrepancyResults.add(dicrepancyBindings);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // checking around for unexpected changes to the immediate environment (i.e., location discrepancies at(X,Y,Z))
    ////////////////////////////////////////////////////////////////////////////////////
    Predicate selfQuery = Factory.createPredicate("at(self,X,Y)");
    Predicate directionQuery = Factory.createPredicate("facing(self,X)");
    List<Map<Variable, Symbol>> beliefBindings;

    try {
      beliefBindings = gameState.queryBelief(selfQuery);
      Term self = selfQuery.copyWithNewBindings(beliefBindings.get(0));
      log.debug("self is at " + self); //at(self,X,Y)
      beliefBindings = gameState.queryBelief(directionQuery);
      Term facing = directionQuery.copyWithNewBindings(beliefBindings.get(0));
      log.debug("self is facing: " + facing);

      //figure out relevant locations
      int x = Integer.parseInt(self.getArgs().get(1).toString());
      int y = Integer.parseInt(self.getArgs().get(2).toString());

      Set<Predicate> locations = new HashSet<>();
      int range = 2;
      for (int x_off = -range; x_off <= range; ++x_off) {
        for (int y_off = -range; y_off <= range; ++y_off) {
          int at_x = x + x_off;
          int at_y = y + y_off;
          if (at_x >= 0 && at_y >= 0 && !(at_x == x && at_y == y)) { // greater than zero and not agent location
            locations.add(Factory.createPredicate("at", "A", Integer.toString(at_x), Integer.toString(at_y)));
          }
        }
      }

      locations = filterPointsNotInCurrentRoom(locations);
      log.trace("locations: " + locations);

      List<Map<Variable, Symbol>> observedBindings;
      for (Term location : locations) {
        beliefBindings =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, location);
        observedBindings = gameState.queryBelief(location);
        if (!observedBindings.isEmpty() || !beliefBindings.isEmpty()) {
          Set<Term> beliefs = location.copyWithNewBindings(beliefBindings);
          Set<Term> observations = location.copyWithNewBindings(observedBindings);

          Iterator<Term> beliefItr = beliefs.iterator();
          while (beliefItr.hasNext()) {
            Term belief = beliefItr.next();
            if (observations.contains(belief)) {
              beliefItr.remove();
              observations.remove(belief);
            } else {
              // there's a discrepancy
              String actor = belief.get(0).getName();
              if (!actor.startsWith("novelitem")) {
                Map<Variable, Symbol> dicrepancyBindings = new HashMap<>();
                dicrepancyBindings.put(beliefStateVar, belief);
                dicrepancyBindings.put(gameStateVar, Factory.createPredicate("not", belief));
                discrepancyResults.add(dicrepancyBindings);
                //observation does not contain this item at this location. better retract the novelitem as well
                Symbol lx = belief.get(1);
                Symbol ly = belief.get(2);
                Predicate novelLocQuery = Factory.createPredicate("at", new Variable("N"), lx, ly);
                List<Map<Variable, Symbol>> novelBindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, novelLocQuery);
                for (Map<Variable, Symbol> b : novelBindings) {
                  Symbol novelID = b.get(new Variable("N"));
                  if (novelID.toString().startsWith("novelitem")) {
                    Term novelLoc = novelLocQuery.copyWithNewBindings(b);
                    dicrepancyBindings = new HashMap<>();
                    dicrepancyBindings.put(beliefStateVar, novelLoc);
                    dicrepancyBindings.put(gameStateVar, Factory.createPredicate("not", novelLoc));
                    discrepancyResults.add(dicrepancyBindings);
                  }
                }

                // if at() discrepancy is for actor (has unique id) -- add actual location of actor
                try {
                  if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type(" + actor + ",agent)"))) {
                    Predicate actorLocQuery = Factory.createPredicate("at(" + actor + ",X,Y)");
                    List<Map<Variable, Symbol>> actorLocResults = gameState.queryBelief(actorLocQuery);
                    if (actorLocResults.size() == 1) {
                      Predicate actorLoc = actorLocQuery.copyWithNewBindings(actorLocResults.get(0));
                      dicrepancyBindings = new HashMap<>();
                      dicrepancyBindings.put(beliefStateVar, Factory.createPredicate("not", actorLoc));
                      dicrepancyBindings.put(gameStateVar, actorLoc);
                      discrepancyResults.add(dicrepancyBindings);
                    } else {
                      log.warn("Cannot determine location of actor: " + actor);
                    }
                  }
                } catch (TRADEException e) {
                  log.error("Error checking if location discrepancy is for an actor.", e);
                }
              }
            }
          }

          Iterator<Term> observationItr = observations.iterator();
          while (observationItr.hasNext()) {
            Term observation = observationItr.next();
            if (beliefs.contains(observation)) {
              observationItr.remove();
              beliefs.remove(observation);
            } else {
              // there's a discrepancy
              Map<Variable, Symbol> dicrepancyBindings = new HashMap<>();
              dicrepancyBindings.put(beliefStateVar, Factory.createPredicate("not", observation));
              dicrepancyBindings.put(gameStateVar, observation);
              discrepancyResults.add(dicrepancyBindings);

              // if at() discrepancy is for actor -- remove where we thought actor was
              String actor = observation.get(0).getName();
              try {
                if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type(" + actor + ",agent)"))) {
                  Predicate prevActorLocQuery = Factory.createPredicate("at(" + actor + ",X,Y)");
                  List<Map<Variable, Symbol>> prevActorLocResults = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, prevActorLocQuery);
                  if (prevActorLocResults.size() == 1) {
                    Predicate prevActorLoc = prevActorLocQuery.copyWithNewBindings(prevActorLocResults.get(0));
                    dicrepancyBindings = new HashMap<>();
                    dicrepancyBindings.put(beliefStateVar, prevActorLoc);
                    dicrepancyBindings.put(gameStateVar, Factory.createPredicate("not", prevActorLoc));
                    discrepancyResults.add(dicrepancyBindings);
                  } else {
                    log.warn("Cannot determine location of actor: " + actor);
                  }
                }
              } catch (TRADEException e) {
                log.error("Error checking if location discrepancy is for an actor.", e);
              }
            }
          }
        }
      }

      ////////////////////////////////////////////////////////////////////////////////////
      // check for holding discrepancy
      ////////////////////////////////////////////////////////////////////////////////////
      // NOTE: adding a holding discrepancy check requires substantial changes to the action condition/effects bc
      // lots of actions change the state of holding, even though most of the time what the agent is holding doesn't matter
      // as it will be changed automatically without having to perform an explicit (de)select action
//      Predicate holdingQuery = Factory.createPredicate("holding(self,X)");
//      beliefBindings = (List<Map<Variable, Symbol>>) TRADE.callThe("queryBelief", holdingQuery);
//      observedBindings = gameState.queryBelief(holdingQuery);
//      if (beliefBindings.size() == 1 && observedBindings.size() == 1) {
//        Predicate holdingBelief = holdingQuery.copyWithNewBindings(beliefBindings.get(0));
//        Predicate holdingObservation = holdingQuery.copyWithNewBindings(observedBindings.get(0));
//        if (!holdingBelief.equals(holdingObservation)) {
//          Map<Variable, Symbol> holdingDiscrepancy = new HashMap<>();
//          holdingDiscrepancy.put(beliefStateVar, holdingBelief);
//          holdingDiscrepancy.put(gameStateVar, holdingObservation);
//          discrepancyResults.add(holdingDiscrepancy);
//        }
//      } else {
//        log.error("Could not check holding discrepancy.");
//      }

    } catch (TRADEException e) {
      log.error("Error calling queryBelief while checking for location novelties.", e);
      return discrepancyResults;
    }

    try {
      //constant or subtype discrepancies
      List<Predicate> constants = new ArrayList<>();
      Predicate p = Factory.createPredicate("constant(X,Y)");
      List<Map<Variable, Symbol>> map = gameState.queryBelief(p);
      for (Map<Variable, Symbol> bindings : map) {
        constants.add(p.copyWithNewBindings(bindings));
      }

      for (Predicate c : constants) {
        boolean b = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, c);
        if (!b) {
          Map<Variable, Symbol> dicrepancyBindings = new HashMap<>();
          dicrepancyBindings.put(beliefStateVar, Factory.createPredicate("not", c));
          dicrepancyBindings.put(gameStateVar, c);
          discrepancyResults.add(dicrepancyBindings);
        }
      }

      List<Predicate> subtypes = new ArrayList<>();
      p = Factory.createPredicate("subtype(X,Y)");
      map = gameState.queryBelief(p);
      for (Map<Variable, Symbol> bindings : map) {
        subtypes.add(p.copyWithNewBindings(bindings));
      }

      for (Predicate c : subtypes) {
        boolean b = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, new Predicate("type",c.getArgs()));
        if (!b) {
          Map<Variable, Symbol> dicrepancyBindings = new HashMap<>();
          dicrepancyBindings.put(beliefStateVar, Factory.createPredicate("not", c));
          dicrepancyBindings.put(gameStateVar, c);
          discrepancyResults.add(dicrepancyBindings);
        }
      }
    } catch (TRADEException e){
      log.error("Error calling queryBelief while checking for constant novelties.", e);
    }

    log.debug("discrepancies: " + discrepancyResults);

    for (Predicate p : getVisionAssertions()) {
      Map<Variable, Symbol> dicrepancyBindings = new HashMap<>();
      dicrepancyBindings.put(beliefStateVar, Factory.createPredicate("not", p));
      dicrepancyBindings.put(gameStateVar, p);
      discrepancyResults.add(dicrepancyBindings);
    }
    return discrepancyResults;
  }

  /**
   * Convenience method to filterPointsNotInCurrentRoom when only a single location needs to be checked.
   * Location predicate needs to be of form: at(O,locX,locY)
   *
   * @param location
   * @return
   */
  public boolean isInCurrentRoom(Predicate location) {
    Set<Predicate> locations = new HashSet<>();
    locations.add(location);
    return !filterPointsNotInCurrentRoom(locations).isEmpty();
  }

  /**
   * Helper method to filter out locations that are not in the same room as our agent. Useful for checking for at()
   * discrepancies, since our agent can't observe outside its current room.
   * Location predicates needs to be of form: at(O,locX,locY)
   *
   * @param locations
   * @return
   */
  @TRADEService
  @Action
  public Set<Predicate> filterPointsNotInCurrentRoom(Set<Predicate> locations) {
    // get wall locations of current room
    Prover gameState = getCurrentGameState();
    Variable wall = Factory.createVariable("W");
    Variable xVar = Factory.createVariable("X");
    Variable yVar = Factory.createVariable("Y");
    Predicate wallQuery = Factory.createPredicate("at", wall, xVar, yVar);
    List<Map<Variable, Symbol>> wallBindings = gameState.queryBelief(wallQuery);

    // calculate current room boundaries based on wall locations
    int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE;
    int maxX = 0, maxY = 0;
    Set<String> entryTypes = new HashSet<>();
    entryTypes.add("door");
    entryTypes.add("open_door");
    for (Map<Variable, Symbol> wallBinding : wallBindings) {
      if (!entryTypes.contains(wallBinding.get(wall).getName())) {
        int x = Integer.parseInt(wallBinding.get(xVar).getName());
        int y = Integer.parseInt(wallBinding.get(yVar).getName());
        if (x < minX) {
          minX = x;
        }
        if (x > maxX) {
          maxX = x;
        }
        if (y < minY) {
          minY = y;
        }
        if (y > maxY) {
          maxY = y;
        }
      }
    }
    // inflate room bb by 1 height and width so that "contains" calls return true for points on bb boundary (e.g., walls, doors)
    Rectangle roomBB = new Rectangle(minX, minY, (maxX - minX) + 1, (maxY - minY) + 1);

    // filter out locations not in room bounding box
    Set<Predicate> filteredLocations = locations.stream().filter(location -> {
      int size = location.size();
      int x = Integer.parseInt(location.get(size - 2).getName());
      int y = Integer.parseInt(location.get(size - 1).getName());
      return roomBB.contains(x, y);
    }).collect(Collectors.toSet());

    return filteredLocations;
  }

  /**
   * Main entry point for reporting novelties to Polycraft. This checks if the novelty has
   * already been reported, gets the novelty level, reports new unreported novelties to Polycraft,
   * and updates Belief of any reported novelties.
   *
   * @param novelties
   */
  @TRADEService
  public void reportNovelties(Set<? extends Term> novelties) {
    for (Term novelty : novelties) {

      // if using vision and/or graph module, only use reported novelties by those modules
//      if (useVisionModule || useGrapModule) {
//        String firsArg = novelty.get(0).getName();
//        if (!firsArg.equals("graph") && !firsArg.equals("vision")) {
//          continue;
//        }
//      }

      try {
        // check if novelty has already been reported (at any level)
        Variable levelVar = Factory.createVariable("X");
        Predicate noveltyQuery = Factory.createPredicate("novelty", novelty, levelVar);
        boolean noveltyReported = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(Boolean.class, noveltyQuery);

        if (!noveltyReported) {
          // get novelty level info and report the novelty to polycraft
          Pair<Integer, String> levelInfo = getNoveltyLevel(novelty);
          reportNovelty(levelInfo.getLeft(), levelInfo.getRight());

          // add reported novelty to belief
          Predicate noveltyAssertion = Factory.createPredicate("novelty", novelty, Factory.createSymbol(levelInfo.getLeft().toString()));
          log.warn("Reported novelty: " + noveltyAssertion);
          TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, noveltyAssertion, MemoryLevel.UNIVERSAL);
        }
      } catch (TRADEException e) {
        log.error("Error reporting novelty: " + novelty, e);
      }
    }
  }

  /**
   * Gets the novelty level (0-3) (and associated message) for a novelty Predicate.
   *
   * @param novelty
   * @return
   */
  private Pair<Integer, String> getNoveltyLevel(Term novelty) {
    if (novelty.getName().contains("recipe")) {
      characterize("Environment");
      return Pair.of(3, "New recipe");
    } else if (novelty.getName().contains("cost")) {
      //characterize("Relations");
      //characterize("Actions");
      characterize("Event");
      return Pair.of(3, "Updated action cost");
    } else if (novelty.getName().contains("discrepancy")) {
      //characterize("Relations");
      //characterize("Interactions");
      characterize("Event");
      return Pair.of(4, "Discrepancy found: " + novelty);
    } else if (novelty.getName().contains("constant") || novelty.getName().contains("subtype")) {
      //characterize("Objects");
      //need to check if it's actually in the world right now
      Symbol obj = novelty.get(0);
      Predicate p = Factory.createPredicate("fluent_equals(world("+obj.toString()+"),0)");
      try {
        boolean b = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class,p);
        log.info(p + " is " + b);
        if (b){
          characterize("Event");
        } else {
          characterize("Environment");
        }
      } catch (TRADEException e) {
        log.error("Couldn't call belief:",e);
      }
      return Pair.of(1, "New object detected: " + novelty);
    } else if (novelty.getName().contains("vision")) {
      characterize("Event");
      return Pair.of(10, "Visual novelty detected: " + novelty);
    } else if (novelty.getName().contains("agent")) {
      characterize("Goal");
      return Pair.of(9, "Agent novelty detected: " + novelty);
    } else {
      return Pair.of(0, "Unknown novelty (could be any level L0 and higher)");
    }
  }

  private Prover getCurrentGameState() {
    // if game state needs updated, update it, and send updated info to graph and vision modules
    if (game.hasGameStateChanged()) {
      game.updateGameState();
      Sense senseAction = game.getLastSenseAction();
      if (graphSocket != null && !isGameOver) {
        //retrieve actor actions
        Sense senseActors = new SenseActors();
        game.perform(senseActors);
        graphSocket.sendCommand(senseActors.getRawResponse());
        String result = graphSocket.waitedResponse(8000);

        if (result == null) {
          log.error("Graph module returned null. It likely crashed.");
        } else {
          try {
            parseActorInfo(result);
          } catch (Exception e) {
            log.error("Error parsing actor report: "+result);
          }
        }
      }
      if (visSocket != null && !isGameOver) {
        SenseScreen senseScreen = new SenseScreen();
        game.perform(senseScreen);
        sendScreen(senseScreen.getRawResponse(), senseAction.getRawResponse());
      }
      if (graphSocket != null || visSocket != null) {
        // check graph and/or vision for novelties
        // this also reports novelties to PAL
        //getGraphVisionAssertions();
      }
    }

    return game.getGameState();
  }

  @TRADEService
  public Justification senseRecipe() {
    GameAction action = new SenseRecipe();
    game.perform(action);
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  public Justification senseHint() {
    GameAction action = new SenseHint();
    game.perform(action);
    return new ConditionJustification(action.getSuccess());
  }

  public Justification characterize(String identifier) {
    if (characterize == "Environment") {
      //if we've already characterized the episode as an environmental novelty, don't change that
      identifier = characterize;
    } else if (characterize == "Event") {
      //otherwise, if we've already characterized the episode as an event novelty, don't change it to goal
      identifier = characterize;
    }
    characterize = identifier;
    Active action = new Characterization(identifier);
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[characterize] failed with message: " + action.getFailMessage());
    }
    //return new ConditionJustification(action.getSuccess());
    return new ConditionJustification(true);
  }

  public Justification reportNovelty(Integer level, String message) {
    Active action = new ReportNovelty(level, message);
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[reportNovelty] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification giveUp() {
    Active action = new GiveUp();
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[giveUp] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification smoothTurn(Symbol turnAngle) {
    Active action = new SmoothTurn(Integer.parseInt(turnAngle.toString()));
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[smoothTurn] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification smoothMove(Symbol direction) {
    log.debug("smoothMove direction: " + direction);
    Active action = new SmoothMove(direction.toString());
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[smoothMove] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification breakBlock() {
    Active action = new BreakBlock();
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[breakBlock] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification placeItem(Symbol itemSymbol) {
    String item = itemSymbol.getName();
    String resolvedItem = SymbolResolver.toPolyItem(item);
    log.debug("Placing " + item);
    Active action = new Place(resolvedItem);
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[placeItem] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification craft(Term recipe) {
    List<String> ingredients = new ArrayList<>();

    for (int i = 0; i < 9; i++) {
      String ing = recipe.get(i).getName();
      if (ing.equals("0")) {
        ingredients.add("0");
      } else {
        ingredients.add(SymbolResolver.toPolyItem(ing));
      }
    }

    boolean on_table = false;
    int[] twoxtwo = {0, 1, 3, 4};
    int[] nontwoxtwo = {2, 5, 6, 7, 8};

    for (int i : nontwoxtwo) {
      if (!ingredients.get(i).equals("0")) {
        on_table = true;
        break;
      }
    }

    int order[];
    if (on_table) {
      order = new int[]{0, 1, 2, 3, 4, 5, 6, 7, 8};
    } else {
      order = twoxtwo;
    }

    String arguments = "1";

    for (int o : order) {
      arguments += " " + ingredients.get(o);
    }

    Active action = new Craft(arguments);
    game.perform(action);

    if (!action.getSuccess()) {
      log.error("[craft] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification teleport(Symbol targetX, Symbol targetY, Symbol distance) {
    // add the z coordinate in the target loc
    String targetLoc = targetX.getName() + "_" + targetY.getName();
    String d = "";
    if (distance.getName().equals("two")) {
      d = " 2";
    }
    targetLoc = SymbolResolver.toPolycraftLocation(targetLoc) + d;
    Active action = new Teleport(targetLoc);
    game.perform(action);

    if (!action.getSuccess()) {
      log.error("[teleport] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification selectItem(Symbol item) {
    Active action;
    if (item.getName().equals("air") || item.getName().equals("null")) {
      action = new Select();
    } else {
      action = new Select(item.toString());
    }
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[selectItem] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }
  // New actions in phase 2

  @TRADEService
  @Action
  public Justification use() {
    Active action = new Use();
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[use] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification collect() {
    Active action = new Collect();
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[collect] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification delete(Symbol itemSymbol) {
    String item = itemSymbol.getName();
    String resolvedItem = SymbolResolver.toPolyItem(item);
    log.debug("Deleting " + item);
    Active action = new Delete(resolvedItem);
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[delete] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification nop() {
    Active action = new Nop();
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[nop] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification interact(Symbol otherActor, boolean expectTrades) {
    String actorName = otherActor.getName(); // e.g., trader_21
    String actorID = actorName.substring(actorName.indexOf('_') + 1); // e.g., 21
    Interact action = new Interact(actorID);

    game.perform(action);

    if (!action.getSuccess()) {
      log.error("[interact] failed with message: " + action.getFailMessage());
      return new ConditionJustification(false);
    }

    // check for novelty when you dont get a trade.
    if (expectTrades && action.getTradeOption().isEmpty()) {
      Predicate emptyTrade = Factory.createPredicate("trade(empty)");
      Set<Predicate> emptyTradeSet = new HashSet<>();
      emptyTradeSet.add(emptyTrade);
      this.reportNovelties(emptyTradeSet);
      return new ConditionJustification(false);
    } else {
      TradeRecipe.createTradeActions(action.getTradeOption(), actorName, actorID, tradeCount);
      tradeCount += action.getTradeOption().size();
    }


    return new ConditionJustification(action.getSuccess());
  }

  @TRADEService
  @Action
  public Justification trade(Integer traderID, Term tradeRecipe) {
    log.debug("Making trade with trader: " + traderID + " recipe: " + tradeRecipe);

    Predicate tradeInputPredicate = (Predicate) tradeRecipe.get(0);
    StringBuilder inputArgs = new StringBuilder();
    tradeInputPredicate.getArgs().forEach(arg -> inputArgs.append(SymbolResolver.toPolyItem(arg.toString())).append(" "));

    Active action = new Trade(traderID.toString(), inputArgs.toString().trim());
    game.perform(action);
    if (!action.getSuccess()) {
      log.error("[trade] failed with message: " + action.getFailMessage());
    }
    return new ConditionJustification(action.getSuccess());
  }

  /**
   * Try to observe the specified state in the world. This method doesn't currently handle unbound variables in the
   * next_to predicate.
   *
   * @param state
   * @return
   */
  @TRADEService
  @Observes({"next_to(X,Y)"})
  public List<Map<Variable, Symbol>> observeNextTo(Term state) {
    log.debug("Observing: " + state);
    if (!state.getVars().isEmpty()) {
      log.error("This observer can't observe state with unbound variables: " + state);
      return null;
    }

    Symbol objOne = state.get(0);
    Symbol objTwo = state.get(1);
    Prover gameState = getCurrentGameState();
    List<Map<Variable, Symbol>> results = new ArrayList<>();

    // get locations of all objOnes and objTwos
    Variable xVar = Factory.createVariable("X");
    Variable yVar = Factory.createVariable("Y");
    Predicate objOneQuery = Factory.createPredicate("at", objOne, xVar, yVar);
    List<Map<Variable, Symbol>> objOneLocs = gameState.queryBelief(objOneQuery);
    Predicate objTwoQuery = Factory.createPredicate("at", objTwo, xVar, yVar);
    List<Map<Variable, Symbol>> objTwoLocs = gameState.queryBelief(objTwoQuery);
    for (Map<Variable, Symbol> objOneLoc : objOneLocs) {
      int x1 = Integer.parseInt(objOneLoc.get(xVar).toString());
      int y1 = Integer.parseInt(objOneLoc.get(yVar).toString());

      for (Map<Variable, Symbol> objTwoLoc : objTwoLocs) {
        int x2 = Integer.parseInt(objTwoLoc.get(xVar).toString());
        int y2 = Integer.parseInt(objTwoLoc.get(yVar).toString());

        if (Movement.getDistance(x1, y1, x2, y2) == 1) {
          // next_to observed
          results.add(new HashMap<>());
          return results;
        }
      }
    }

    // observation not made
    return results;
  }

  /**
   * Try to observe if facing_obj(self,X,one) or facing_obj(self,X,two) is true.
   *
   * @param state
   * @return
   */
  @TRADEService
  @Observes({"facing_obj(self,X,one)", "facing_obj(self,X,two)"})
  public List<Map<Variable, Symbol>> observeFacingObject(Term state) {
    log.debug("Observing: " + state);

    Prover gameState = getCurrentGameState();

    // get agent location and direction from game
    Symbol self = state.get(0);
    Variable xVar = Factory.createVariable("X");
    Variable yVar = Factory.createVariable("Y");
    Variable dirVar = Factory.createVariable("Dir");
    Predicate atQuery = Factory.createPredicate("at", self, xVar, yVar);
    Predicate directionQuery = Factory.createPredicate("facing", self, dirVar);
    List<Map<Variable, Symbol>> atBindings = gameState.queryBelief(atQuery);
    List<Map<Variable, Symbol>> dirBindings = gameState.queryBelief(directionQuery);

    // get item(s) directly (one block) in front
    Predicate inFrontPred = Movement.getInFront(atBindings.get(0).get(xVar), atBindings.get(0).get(yVar), dirBindings.get(0).get(dirVar));
    List<Map<Variable, Symbol>> inFrontBindings;
    if (isInCurrentRoom(inFrontPred)) {
      // query game state
      inFrontBindings = gameState.queryBelief(inFrontPred);
    } else {
      // query belief
      try {
        inFrontBindings =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, inFrontPred);
      } catch (TRADEException e) {
        log.error("Error querying belief for location predicate: " + inFrontPred, e);
        inFrontBindings = new ArrayList<>();
      }
    }

    List<Map<Variable, Symbol>> results = new ArrayList<>();
    Symbol distance = state.get(2);
    if (distance.getName().equals("one")) {
      // check distance "one" case
      if (state.get(1).isVariable()) {
        // free-variable
        if (inFrontBindings.isEmpty()) {
          Map<Variable, Symbol> air = new HashMap<>();
          air.put((Variable) state.get(1), Factory.createSymbol("air"));
          results.add(air);
        } else {
          Map<Variable, Symbol> inFront = new HashMap<>();
          inFront.put((Variable) state.get(1), inFrontBindings.get(0).get((Variable) inFrontPred.get(0)));
          results.add(inFront);
        }
      } else if (state.get(1).getName().endsWith("air")) {
        // check air variants -- not explicitly keeping track of at(air,X,Y) for air variants, so no results for at(air,X,Y) means air at X,Y
        if (inFrontBindings.isEmpty()) {
          results.add(new HashMap<>());
        }
      } else {
        // check all non-air objects
        for (Map<Variable, Symbol> m : inFrontBindings) {
          if (inFrontPred.copyWithNewBindings(m).get(0).equals(state.get(1))) {
            results.add(new HashMap<>());
          }
        }
      }
    } else if (distance.getName().equals("two")) {
      // check distance "two" case
      boolean somethingInFront = !inFrontBindings.isEmpty();
      if (!somethingInFront) {
        // now check target object is two steps in front (i.e., that it's in front of what's directly in front of the agent)
        Predicate twoInFrontPred = Movement.getInFront(inFrontPred.get(1), inFrontPred.get(2), dirBindings.get(0).get(dirVar));
        List<Map<Variable, Symbol>> twoInFrontBindings;
        if (isInCurrentRoom(twoInFrontPred)) {
          // query game state
          twoInFrontBindings = gameState.queryBelief(twoInFrontPred);
        } else {
          // query belief
          try {
            twoInFrontBindings =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, twoInFrontPred);
          } catch (TRADEException e) {
            log.error("Error querying belief for location predicate: " + twoInFrontPred, e);
            twoInFrontBindings = new ArrayList<>();
          }
        }
        for (Map<Variable, Symbol> m : twoInFrontBindings) {
          if (twoInFrontPred.copyWithNewBindings(m).get(0).equals(state.get(1))) {
            results.add(new HashMap<>());
          }
        }
      }
    }

    // HACK: to make sure there's always one of these states in belief
    if (results.isEmpty()) {
      log.debug("State not observed: " + state);
      try {
        Predicate facingObjPred;
        if (inFrontBindings.isEmpty()) {
          // assume air is in front
          facingObjPred = Factory.createPredicate("facing_obj(self,air,one)");
        } else {
          facingObjPred = inFrontPred.copyWithNewBindings(inFrontBindings.get(0));
        }
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, facingObjPred, MemoryLevel.EPISODIC);
      } catch (TRADEException e) {
        log.error("Error updating belief with observed state.", e);
      }
    }

    return results;
  }

  @TRADEService
  @Observes({"holding(X,Y)", "at(X,Y,Z)", "facing(X,Y)"})
  public List<Map<Variable, Symbol>> observeState(Term state) {
    log.debug("Observing: " + state);
    Prover gameState = getCurrentGameState();

    List<Map<Variable, Symbol>> results = gameState.queryBelief(state);
    if (results.isEmpty()) {
      log.debug("State not observed: " + state);

      // HACK: to make sure there's always one of these states in belief
      try {
        if (state.getName().equals("holding")) {
          Predicate queryPred = Factory.createPredicate("holding(X,Y)");
          List<Map<Variable, Symbol>> gameStateResults = gameState.queryBelief(queryPred);
          TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, queryPred.copyWithNewBindings(gameStateResults), MemoryLevel.EPISODIC);
        }
      } catch (TRADEException e) {
        log.error("Error updating belief with observed state.", e);
      }
    }
    return results;
  }

  @TRADEService
  @Observes({"fluent_equals(X,Y)"})
  public List<Map<Variable, Symbol>> observeEquality(Term state) {
    log.debug("Observing: " + state);

    if (state.get(0).getName().startsWith("cost")) {
      return observeCost(state);
    }

    Prover gameState = getCurrentGameState();
    Term numericFunction = (Term) state.get(0);
    Symbol queryValue = state.get(1);
    List<Map<Variable, Symbol>> results = new ArrayList<>();
    if (queryValue.isVariable()) {
      // free-variable in state query, just query gameState
      results = gameState.queryBelief(state);
      if (results.isEmpty()) {
        // assume zero
        Map<Variable, Symbol> zeroResults = new HashMap<>();
        zeroResults.put((Variable) queryValue, Factory.createSymbol("0"));
        results.add(zeroResults);
      }
    } else {
      // numeric value in state query, get game state value and compare
      // not just querying game state so that int and double values can be correctly compared
      // in prolog "1" != "1.0"
      Variable valueVar = Factory.createVariable("V");
      Predicate query = Factory.createPredicate(state.getName(), numericFunction, valueVar);
      List<Map<Variable, Symbol>> tmpResults = gameState.queryBelief(query);
      if (tmpResults.isEmpty()) {
        log.debug("State not observed: " + state);
      } else {
        if (tmpResults.size() > 1) {
          log.warn("More than one value found for query: " + state);
        }
        Symbol worldValue = tmpResults.get(0).get(valueVar);
        if (Double.parseDouble(worldValue.getName()) == Double.parseDouble(queryValue.getName())) {
          results.add(new HashMap<>()); // i.e., true
        }
      }
    }
    return results;
  }


  private List<Map<Variable, Symbol>> observeCost(Term observation) {
    log.debug("Observing cost: " + observation);
    Term costFunction = (Term) observation.get(0);
    Symbol costValue = observation.get(1);
    GameAction lastAction = game.getLastActionTaken(costFunction.get(0).toString().toUpperCase());
    List<Map<Variable, Symbol>> results = new ArrayList<>();

    if (lastAction != null && lastAction.getSuccess()) {
      Double stepcost = lastAction.getStepCost();
      String command = lastAction.getCommand();
      log.debug("stepcost = " + stepcost + ". command = " + command);

      Map<Variable, Symbol> bindings = new HashMap<>();

      if (costValue.isVariable()) {
        bindings.put((Variable) costValue, new Symbol(stepcost.toString()));
        results.add(bindings);
      } else if (Double.parseDouble(costValue.getName()) == Double.parseDouble(stepcost.toString())) {
        results.add(bindings);
      }
    } else {
      log.error("Unable to observe action cost: " + observation);
      return null;
    }
    log.debug("results = " + results);
    return results;
  }

  /**
   * Debug method to make it easy to log the raw SENSE_ALL JSON from anywhere in the system.
   * The output JSON will be written to {@code /tmp/<filePreFix><rdn>.json} and the filepath
   * is the return value.
   *
   * @param filePreFix
   * @return output file path
   */
  @TRADEService
  public String writeSenseAllJsonToFile(String filePreFix) {
    if (game.getLastSenseAction() == null) {
      log.warn("Game's lastSenseAll is null. Calling getCurrentGameState to update lastSenseAll.");
      getCurrentGameState();
    }

    String rawJson = game.getLastSenseAction().getRawResponse();
    try {
      File file = File.createTempFile(filePreFix, ".json");
      FileWriter writer = new FileWriter(file);
      writer.write(rawJson);
      writer.close();
      return file.getAbsolutePath();
    } catch (IOException e) {
      log.error("Error writing SenseAll JSON to file: " + filePreFix, e);
      return "N/A";
    }
  }

  ////////////////////////////////////////////////////////////////
  //////////// Methods for graph and vision modules //////////////
  ////////////////////////////////////////////////////////////////

  private void sendScreen(String screenJson, String senseAllJson) {
    if (visSocket != null) {
      if (screenJson == null || senseAllJson == null) {
        log.error("screenJson or senseAllJson null. Not calling vision module.");
        return;
      }

      // combine jsons
      StringBuilder combinedJsons = new StringBuilder();
      combinedJsons.append("[").append(screenJson).append(",").append(senseAllJson).append("]");

      // first combined json
      visSocket.sendCommand(combinedJsons.toString());
      String res = visSocket.waitedResponse(5000);
      log.trace(res);
    }
  }

  public Set<Predicate> createVisionGraphAssertions(String model_output) {
    Set<Predicate> assertions = new HashSet<>();

    if (model_output.equals("None")) {
      return assertions;
    }

    ModelResult model_result = new ModelResult(model_output);
    return model_result.getAssertions();
  }

  @TRADEService
  public void setGameOver(boolean goalAchieved) {
    if (!isGameOver) {
      isGameOver = true;

      getGraphAssertions();

      if (graphSocket != null) {
        String message_request = "RESET";
        graphSocket.sendCommand(message_request);
        String response = graphSocket.waitedResponse(8000);
        log.info("Agent Model response received after RESET: " + response);
      }
    } else {
      log.warn("setGameOver being called more than once per episode.");
    }
  }

  protected class ParsedVisionResponse {
    Player player;
    int nov_dir;
    boolean is_novel;
    float novel_score;
  }

  protected class ParsedActorsResponse {
    Map<String, ParsedActorInfo> actors;
  }

  protected class ParsedActorInfo {
    boolean novelty;
    AgentJustification justification;
  }

  protected class AgentJustification {
    AgentType agent_type;
    Map<String, String> first_unlikely_action;
    List<String> new_actions;
    Map<String, String> changepoint;
  }

  protected class AgentType {
    String reported_type;
    String belief_type;
    Map<String, Double> acting_ratio;
  }

  @TRADEService
  public List<Predicate> getGraphAssertions() {
    String message_request = "SEND_DETECTIONS";
    List<Predicate> module_results = new ArrayList<>();

    log.debug("NNMODEL - SENDING RESPONSE REQUEST");
    if (graphSocket != null) {
      graphSocket.sendCommand(message_request);
      String result = graphSocket.waitedResponse(8000);

      //Modules send None if they cannot make a prediction or they see no novelty.
      if (result == null) {
        log.error("Graph module returned null. It likely crashed.");
      } else if (!result.equals("None")) {
        //novelty detected
        //module_results.add(createVisionGraphAssertions(result)); // This line creates assertions from sense-all like response. This is currently not supported since we only care about binary responses.
        try {
          parseActorInfo(result);
        } catch (Exception e) {
          log.error("Error parsing actor report at end of game: " + result);
        }
        /*
        Predicate novelty = null;

        if (result.equals("New_String_Detected")) {
          novelty = Factory.createPredicate("novelty", "graph", "object");
        } else if (result.equals("Novelty_Detected")) {
          novelty = Factory.createPredicate("novelty", "graph", "discrepancy");
        } else {
          log.warn("Model result not recognized, treating as None");
        }
        //Report novelties
        if (novelty != null) {
          log.info("NNMODEL - Novelty Detected.");
          Set<Predicate> novelties = new HashSet<>();
          novelties.add(novelty);
          reportNovelties(novelties);
        }
         */
      }
    }
    return module_results;
  }

  @TRADEService
  public List<Predicate> getVisionAssertions() {
    String message_request = "SEND_DETECTIONS";
    List<Predicate> module_results = new ArrayList<>();

    if (visSocket != null) {
      visSocket.sendCommand(message_request);
      String result = visSocket.waitedResponse(5000);

      //Modules send None if they cannot make a prediction or they see no novelty.
      if (result == null) {
        String content = "";
        try {
          content = new String(Files.readAllBytes(Paths.get("/tmp/vision.log")));
        } catch (IOException e) {
          log.error("Error opening vision.log", e);
        }
        log.error("Vision module returned null. It likely crashed. Log contents: " + content);

        if (++visionNullCounter > 5) {
          log.error("Assuming vision module crashed. Setting vision socket to null.");
          visSocket = null;
        }
      } else {
        // reset counter
        visionNullCounter = 0;

        if (!result.equals("None") && (!result.equals("True"))) {
          ParsedVisionResponse parsedResponse = new Gson().fromJson(result, ParsedVisionResponse.class);
          log.debug(result);

          if (episodeNoveltyScore < parsedResponse.novel_score) {
            episodeNoveltyScore = parsedResponse.novel_score;
          }

          if (parsedResponse.is_novel) {
            Predicate novelty = Factory.createPredicate("vision", "object");

            if (localization) {
              Set<Term> locations = noveltyLocalization(parsedResponse.nov_dir, parsedResponse.player);
              if (locations.isEmpty()) {
                if (parsedResponse.nov_dir == 1) {
                  if (!observeFacingObject(Factory.createPredicate("facing_obj(self,air,one)")).isEmpty()) {
                    Justification interactJustification = ExploreUtils.doGoal(Factory.createPredicate("moveForward(self)"));
                    if (interactJustification.getValue()) {
                      log.debug("Action worked.");
                    }
              /*} else {
                Symbol s = Factory.createSymbol("novelitem");
                try {
                  Set<Predicate> explorations = (Set<Predicate>)TRADE.callThe("generateExplorationsToTry",new Term("object", s, s));
                  Iterator iterator = explorations.iterator();
                  while (iterator.hasNext()) {
                    TRADE.callThe("executeExploration",(Predicate) iterator.next());
                  }
                } catch (TRADEException e) {
                  log.error("Error generating/executing exploration:",e);
                }

               */
                  }
                }
              } else {
                //Report novelties
                if (novelty != null) {
                  log.info("VISION - Novelty Detected.");
                  Set<Predicate> novelties = new HashSet<>();
                  novelties.add(novelty);
                  module_results.add(novelty);
                }
              }
            } else {
              //if localization is not present just report normally
              log.info("VISION - Novelty Detected.");
              reportNovelties(Stream.of(novelty).collect(Collectors.toSet()));
            }
          }
        }
      }
    }
    return module_results;
  }

  private void parseActorInfo(String result) {
    //log.info(result);
    Type t = new TypeToken<Map<String, ParsedActorInfo>>() {
    }.getType();
    Map<String, ParsedActorInfo> parsedResponse = new Gson().fromJson(result, t);
    for (String key : parsedResponse.keySet()) {
      ParsedActorInfo actorInfo = parsedResponse.get(key);
      if (actorInfo.novelty) {
        Predicate novelty = Factory.createPredicate("agent()");
        reportNovelties(Stream.of(novelty).collect(Collectors.toSet()));
        //check for new actions or unlikely actions
        AgentJustification j = actorInfo.justification;
        if (!j.agent_type.reported_type.equals(j.agent_type.belief_type)) {
          //actor behaves like another actor type
          String believedType = j.agent_type.belief_type;
          if (!believedType.equals("None")) {
            //actor is behaving like a different actor
            if (!believedType.equals("unknown")) {
              //set actor to type of other actor
              Predicate p = Factory.createPredicate("subtype", j.agent_type.reported_type + "_" + key, believedType);
              try {
                TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, p, MemoryLevel.UNIVERSAL);
              } catch (TRADEException e) {
                log.error("Unable to assert subtype predicate", e);
              }
            }
          }
        }
        if (!j.first_unlikely_action.get("predicted_action").equals(j.first_unlikely_action.get("true_action"))) {
          //actor performed an unlikely action, let's try to imitate it
          characterize("Goals");
          String actionToCopy = j.first_unlikely_action.get("true_action");
          String[] splitAction = actionToCopy.split(" ");
          String action = splitAction[0];
          String parameter = "";
          if (splitAction.length > 1) {
            parameter = SymbolResolver.toGridItem(splitAction[1]);
            action = SymbolResolver.toAction(action, parameter);
          } else {
            action = SymbolResolver.toAction(action);
          }
          if (action != null) {
            ActionDBEntry abd = Database.getActionDB().getAction(action);
            if (abd != null) {
              Predicate novelAction = Factory.createPredicate(fillAction(abd, parameter, false));
              log.debug("Asserting " + novelAction.toString());
              try {
                TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, Factory.createPredicate("toExplore", novelAction, novelAction), MemoryLevel.UNIVERSAL);
              } catch (TRADEException e) {
                log.error("Unable to assert exploration predicate", e);
              }
            }
          }
        }
        if (!j.new_actions.isEmpty()) {
          //actor performed a new action
          for (String actionToCopy : j.new_actions) {
            String[] splitAction = actionToCopy.split(" ");
            String action = splitAction[0];
            String parameter = "";
            if (splitAction.length > 1) {
              parameter = SymbolResolver.toGridItem(splitAction[1]);
              action = SymbolResolver.toAction(action, parameter);
            } else {
              action = SymbolResolver.toAction(action);
            }
            if (action != null) {
              ActionDBEntry abd = Database.getActionDB().getAction(action);
              if (abd != null) {
                Predicate novelAction = Factory.createPredicate(fillAction(abd, parameter, false));
                log.debug("Asserting " + novelAction.toString());
                try {
                  TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, Factory.createPredicate("toExplore", novelAction, novelAction), MemoryLevel.UNIVERSAL);
                } catch (TRADEException e) {
                  log.error("Unable to assert exploration predicate", e);
                }
              }
            }
          }
        }
      }
    }
  }

  @TRADEService
  public String fillAction(ActionDBEntry a, String arg, boolean bytwo) {
    Map<Variable, Symbol> bindings = new HashMap<>();
    bindings.put(new Variable("?actor", "agent"), new Symbol("self"));

    List<ActionBinding> actionbindings = a.getInputRoles();
    for (ActionBinding b : actionbindings) {
      if (!bindings.containsKey(new Variable(b.name, b.getSemanticType()))) {
        if (bindings.containsKey(new Variable(b.name))) {
          Symbol s = bindings.remove(new Variable(b.name));
          bindings.put(new Variable(b.name, b.getSemanticType()), s);
        } else if (b.name.equals("?to_dist")) {
          if (bytwo) {
            bindings.put(new Variable(b.name, b.getSemanticType()), new Symbol("two"));
          } else {
            bindings.put(new Variable(b.name, b.getSemanticType()), new Symbol("one"));
          }
        } else {
          try {
            if (!arg.isEmpty() && TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type", arg, b.getSemanticType()), MemoryLevel.EPISODIC)) {
              Term query = Factory.createPredicate("typeobject", "X", arg);
              List<Map<Variable, Symbol>> results = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query, MemoryLevel.EPISODIC);
              if (!results.isEmpty()) {
                //found grounded objects of the type we're looking for
                for (Map<Variable, Symbol> map : results) {
                  bindings.put(new Variable(b.name, b.getSemanticType()), map.get(new Variable("X")));
                  break;
                }
              } else {
                bindings.put(new Variable(b.name, b.getSemanticType()), new Symbol(arg));
              }
            } else {
              Term query = Factory.createPredicate("typeobject", "X", b.getSemanticType());
              List<Map<Variable, Symbol>> results = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query, MemoryLevel.EPISODIC);
              if (!results.isEmpty()) {
                //find an object that matches the semantic type of the argument variable
                for (Map<Variable, Symbol> map : results) {
                  bindings.put(new Variable(b.name, b.getSemanticType()), map.get(new Variable("X")));
                  break;
                }
              } else {
                bindings.put(new Variable(b.name, b.getSemanticType()), new Symbol(b.getSemanticType()));
              }
            }
          } catch (TRADEException e) {
            log.error("Error binding action arguments", e);
          }
        }
      }
    }
    List<Condition> preconditions = a.getConditions();
    for (Condition c : preconditions) {
      Map<Predicate, Observable> map = c.getPredicates();
      Set<Predicate> predicates = map.keySet();
      for (Predicate p : predicates) {
        Term query = p.copyWithNewBindings(bindings);
        try {
          List<Map<Variable, Symbol>> results = new ArrayList<>();
          //if observable, check the observer
              /*if (map.get(p).shouldObserve()) {
                results = (List<Map<Variable, Symbol>>) TRADE.callThe("observe", query);
                if (results.size() == 1 && !results.get(0).values().contains(new Symbol("air"))) {
                  bindings.putAll(results.get(0));
                }
              } else {*/
          results =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query, MemoryLevel.EPISODIC);
          if (results.size() == 1) {
            for (Map<Variable, Symbol> m : results) {
              boolean typealigned = true;
              //check to make sure the results line up with the semantic type of the variable
              for (Variable v : query.getVars()) {
                //the variable from our results does not have a type. check the type of that variable in the query instead
                if (!v.getType().isEmpty()) {
                  query = Factory.createPredicate("type", m.get(new Variable(v.getName())), new Symbol(v.getType()));
                  if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, query, MemoryLevel.EPISODIC)) {
                    typealigned = false;
                    break;
                  }
                }
              }
              if (typealigned) {
                bindings.putAll(m);
                break;
              }
            }
            //}
          }
        } catch (TRADEException e) {
          log.error("Error observing preconditions:", e);
        }
      }
    }

    Predicate submitAction = a.getSignature(true);
    Predicate newAction = submitAction.copyWithNewBindings(bindings);

    //taking care of functions with no arguments
    if (!newAction.toString().endsWith(")")) {
      return newAction.toString() + "()";
    }
    return newAction.toString();
  }

  private Set<Term> noveltyLocalization(int dir, Player position) {
    //label this as a novel variant
    int px = position.pos[0];
    int py = position.pos[2];
    int range = 4;
    int start;
    int end;
    Set<Term> beliefs = new HashSet<>();

    for (int i = 1; i < range; i++) {
      if (dir == 0) {
        start = i + 1;
        end = (int) java.lang.Math.round(0.5 * i);
      } else if (dir == 2) {
        start = (int) java.lang.Math.round(0.5 * -i);
        end = i - 1;
      } else {
        start = java.lang.Math.round(i / (-(range + 1)));
        end = java.lang.Math.round(i / (range + 1));
      }
      while (start >= end) {
        Variable obj = new Variable("X");
        Symbol x;
        Symbol y;
        switch (position.facing) {
          case "SOUTH":
            x = new Symbol(String.valueOf(px + start));
            y = new Symbol(String.valueOf(py + i));
            break;
          case "EAST":
            x = new Symbol(String.valueOf(px + i));
            y = new Symbol(String.valueOf(py + start));
            break;
          case "WEST":
            x = new Symbol(String.valueOf(px - i));
            y = new Symbol(String.valueOf(py + start));
            break;
          default:
            x = new Symbol(String.valueOf(px + start));
            y = new Symbol(String.valueOf(py - i));
            break;
        }
        Term atquery = new Term("at", obj, x, y);
        //List<Map<Variable, Symbol>> results = observeState(atquery);
        try {
          List<Map<Variable, Symbol>> results = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, atquery);
          if (results.size() == 1 && !results.get(0).get(obj).toString().startsWith("novelitem")) {
            //if an object exists at the location and has not yet been marked as a novel item
            Symbol s = new Symbol("novelitem");
            int n = 0;
            //find a unique ID for novelitem that hasn't been used
            while (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("subtype", s, results.get(0).get(obj)))) {
              if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("subtype", s, new Variable("X")))) {
                //if there's no novelitem with this ID, we'll use it here
                beliefs.add(new Term("subtype", s, results.get(0).get(obj)));
                beliefs.add(new Term("constant", s, s));
                beliefs.add(Factory.createPredicate("fluent_equals", new Symbol("world"), s, new Symbol("1")));
                beliefs.add(new Term("unexplored", s));
                break;
              }
              //if there's already a novelitem with this ID in the world, and it doesn't have the type we want, continue to look for a new ID
              n++;
              s = new Symbol("novelitem" + n);
            }
            beliefs.add(new Term("at", s, x, y));
          }
          start--;
        } catch (TRADEException e) {
          log.error("Error calling queryBelief");
        }
      }
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, beliefs, MemoryLevel.EPISODIC);
    } catch (TRADEException e) {
      log.error("Error calling asserBeliefs:", e);
    }
    return beliefs;
  }

}
