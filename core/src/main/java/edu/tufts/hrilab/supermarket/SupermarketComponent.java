/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.planner.PlanningAgent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.supermarket.actions.*;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.*;

import static edu.tufts.hrilab.supermarket.Util.*;

public class SupermarketComponent extends DiarcComponent implements SupermarketInterface, PlanningAgent {
  private boolean busy;

  protected static boolean loadedInitialBeliefs = false;

  protected GamePlay game;
  protected String agentName = "self";
  protected int playerIndex = 0;
  protected boolean useBelief = false;
  protected int socketPort = 9000;
  protected int simSocketPort = -1;
  protected SupermarketInterface simComponent;
  protected boolean useSim = false;
  protected double STEP = 0.1;

  public SupermarketComponent() {
  }

  public SupermarketComponent(SupermarketInterface other) {
    this.game = new GamePlay(other.getSimPort());
    this.agentName = other.getAgentName();
    this.playerIndex = other.getPlayerIndex();
    this.useBelief = false;
  }

  @Override
  protected void init() {
    super.init();
    this.game = new GamePlay(this.socketPort);
    Util.Sleep(500);
    nop();
    transmitInitialBeliefs();
    transmitPlayerInitialBeliefs();
    if (simSocketPort != -1) {
      simComponent = new SupermarketComponent(this);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("belief").desc("connect component to belief").build());
    options.add(Option.builder("agentName").hasArg().argName("name").desc("name of the agent").build());
    options.add(Option.builder("agentIndex").hasArg().argName("index").type(Integer.class).desc("index of the agent").build());
    options.add(Option.builder("sim").hasArg().argName("simPort").type(Integer.class).desc("port of the sim " + "agent").build());
    options.add(Option.builder("socketPort").hasArgs().argName("num").type(Integer.class).desc("port of the simulation").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    super.parseArgs(cmdLine);
    if (cmdLine.hasOption("belief")) {
      this.useBelief = true;
    }
    if (cmdLine.hasOption("agentName")) {
      this.agentName = cmdLine.getOptionValue("agentName");
    }
    if (cmdLine.hasOption("agentIndex")) {
      this.playerIndex = Integer.parseInt(cmdLine.getOptionValue("agentIndex"));
    }
    if (cmdLine.hasOption("socketPort")) {
      this.socketPort = Integer.parseInt(cmdLine.getOptionValue("socketPort"));
    }
    if (cmdLine.hasOption("sim")) {
      this.simSocketPort = Integer.parseInt(cmdLine.getOptionValue("sim"));
    }
  }

  //////////////// GAME ACTIONS //////////////////
  @Override
  public final Justification nop() {
    if (useSim) {
      return simComponent.nop();
    } else {
      GameAction action = new Nop();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification goNorth() {
    if (useSim) {
      return simComponent.goNorth();
    } else {
      GameAction action = new North();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification goSouth() {
    if (useSim) {
      return simComponent.goSouth();
    } else {
      GameAction action = new South();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification goEast() {
    if (useSim) {
      return simComponent.goEast();
    } else {
      GameAction action = new East();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification goWest() {
    if (useSim) {
      return simComponent.goWest();
    } else {
      GameAction action = new West();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification goDir(SupermarketObservation.Direction dir) {
    if (useSim) {
      return simComponent.goWest();
    } else {
      return switch (dir) {
        case NORTH -> goNorth();
        case SOUTH -> goSouth();
        case EAST -> goEast();
        case WEST -> goWest();
        default -> {
          log.error("Unknown direction: " + dir);
          yield new ConditionJustification(false);
        }
      };
    }
  }

  @Override
  public final Justification rotateCW() {
    return goDir(SupermarketObservation.Direction.cw(SupermarketObservation.Direction.fromIndex(getPlayer().direction)));
  }

  @Override
  public final Justification rotateCCW() {
    return goDir(SupermarketObservation.Direction.ccw(SupermarketObservation.Direction.fromIndex(getPlayer().direction)));
  }

  @Override
  public Justification pickup(Symbol goal) {
    SupermarketObservation.InteractiveObject obj = getObjectFromName(goal);
    while (!obj.canInteract(getPlayer())) {
      rotateCW();
    }
    return interactWithObject();
  }

  private boolean aligned(double x1, double span1, double x2, double span2) {
    return x1 < x2 + span2 && x2 < x1 + span1;
  }

  //todo: Should this be moved to a different class, such as SupermarketAgent?
  @Override
  public Justification reactive_nav(Symbol goal) {
    String target = "x";

    SupermarketObservation.InteractiveObject obj = getObjectFromName(goal);

    while (true) {
      SupermarketObservation.Player player = getPlayer();
      SupermarketObservation.Direction direction = SupermarketObservation.Direction.fromIndex(player.direction);

      if (obj.canInteract(player)) {
        break;
      }

      if (target.equals("x")) {
        if (player.position[0] + player.width < obj.position[0]) {
          direction = SupermarketObservation.Direction.EAST;
        } else if (player.position[0] > obj.position[0] + obj.width) {
          direction = SupermarketObservation.Direction.WEST;
        } else {
          target = "y";
          continue;
        }
      }
      if (target.equals("y")) {
        if (player.position[1] + player.height < obj.position[1]) {
          direction = SupermarketObservation.Direction.SOUTH;
        } else if (player.position[1] > obj.position[1] + obj.height) {
          direction = SupermarketObservation.Direction.NORTH;
        } else {
          target = "x";
          continue;
        }
      }
      int counter = 0;
      while (projectCollision(direction, getLastObservation(), player)) {
        direction = SupermarketObservation.Direction.cw(direction);
        counter++;
        if (counter > 4) {
          if (target.equals("x")) {
            target = "y";
          } else {
            target = "x";
          }
          break;
        }
      }
      if (counter < 4) {
        goDir(direction);
      }
    }
    return new ConditionJustification(true);
  }


  @Override
  public final Justification cancelInteraction() {
    if (useSim) {
      return simComponent.cancelInteraction();
    } else {
      GameAction action = new Cancel();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification toggleShoppingCart() {
    if (useSim) {
      return simComponent.toggleShoppingCart();
    } else {
      GameAction action = new Toggle();
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public final Justification interactWithObject() {
    if (useSim) {
      return simComponent.interactWithObject();
    } else {
      GameAction action = new Interact();
      SupermarketObservation.InteractiveObject target = getInteractionTarget(game.observation);
      game.perform(action, this.playerIndex);
      return new ConditionJustification(action.getSuccess());
    }
  }

  @Override
  public Justification startGame() {
    return new ConditionJustification(false);
  }

  @Override
  public Justification resetGame() {
    return new ConditionJustification(false);
  }

  @Override
  public SupermarketObservation getLastObservation() {
    return game.observation;
  }

  private SupermarketObservation.Player getPlayer() {
    return getLastObservation().players[playerIndex];
  }

  @TRADEService
  @Override
  public void setObservation(Object observation) {
    if (!(observation instanceof SupermarketObservation)) {
      return;
    }
    if (useSim) {
      simComponent.setObservation(observation);
    } else {
      game.setObservation((SupermarketObservation) observation);
    }
  }

  //////////////// BELIEF //////////////////


  private SupermarketObservation.InteractiveObject getObjectFromName(Symbol name) {
    SupermarketObservation obs;
    log.debug("Observing " + name);

    //Use the simulation observation if in sim mode
    if (useSim) {
      obs = simComponent.getLastObservation(); //can also return objectObserver in sim, same thing
    } else {
      obs = getLastObservation();
    }

    int objNumber = Integer.parseInt(name.getName().replaceAll("\\D", ""));
    String objType = name.getName().replaceAll("\\d", "");
    SupermarketObservation.InteractiveObject obj = switch (objType) {
      //northOf, southOf
      case "basketReturn" -> obs.basketReturns[objNumber];
      case "basket" -> obs.baskets[objNumber];
      case "cartReturn" -> obs.cartReturns[objNumber];
      case "cart" -> obs.carts[objNumber];
      case "counter" -> obs.counters[objNumber];
      case "register" -> obs.registers[objNumber];
      case "shelf" -> obs.shelves[objNumber];
      default -> null;
    };

    if (obj == null) {
      log.error("Unknown type: " + objType);
    }

    return obj;

  }

  @TRADEService
  @Action
  public List<Map<Variable, Symbol>> objectObserver(Term state) {
    SupermarketObservation obs;
    log.debug("Observing " + state);

    //Use the simulation observation if in sim mode
    if (useSim) {
      obs = simComponent.getLastObservation(); //can also return objectObserver in sim, same thing
    } else {
      obs = getLastObservation();
    }

    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    //If the observation was found
    if (obs.atShelf(getPlayer(), getObjectFromName(state.get(1)))) {
      bindings.add(new HashMap<>());
    }
    return bindings;
  }

  protected void transmitPlayerInitialBeliefs() {
    SupermarketObservation obs = game.observation;
    Set<Predicate> toSubmit = new HashSet<>();
    toSubmit.add(Factory.createPredicate("object(" + this.agentName + ", agent)"));
    toSubmit.add(Factory.createPredicate("object(" + this.agentName + "List, list)"));
    toSubmit.add(Factory.createPredicate("free(" + this.agentName + ":agent)"));
//    toSubmit.add(Factory.createPredicate("at(" + this.agentName + ":agent, null:location)"));
    SupermarketObservation.Player player = getPlayer();

    for (int i = 0; i < obs.shelves.length; i++) {
      SupermarketObservation.Shelf shelf = obs.shelves[i];
      int index = Arrays.asList(player.shopping_list).indexOf(shelf.food_name);
      if (index == -1) {
        toSubmit.add(Factory.createPredicate("fluent_equals(amount(" + this.agentName + "List:list," + foodName(shelf) + ":food), 0)"));
      } else {
        toSubmit.add(Factory.createPredicate("fluent_equals(amount(" + this.agentName + "List:list," + foodName(shelf) + ":food), " + player.list_quant[index] + ")"));
      }
    }

    for (int i = 0; i < obs.counters.length; i++) {
      SupermarketObservation.Counter counter = obs.counters[i];
      int index = Arrays.asList(player.shopping_list).indexOf(counter.food);
      if (index == -1) {
        toSubmit.add(Factory.createPredicate("fluent_equals(amount(" + this.agentName + "List:list," + foodName(counter) + ":food), 0)"));
      } else {
        toSubmit.add(Factory.createPredicate("fluent_equals(amount(" + this.agentName + "List:list," + foodName(counter) + ":food), " + player.list_quant[index] + ")"));
      }

    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, toSubmit);
    } catch (TRADEException e) {
      log.error("assertBeliefs not found", e);
    }
  }

  protected void transmitInitialBeliefs() {

    SupermarketObservation obs = game.observation;

    Set<Predicate> toSubmit = new HashSet<>();

    toSubmit.add(Factory.createPredicate("object(null, location)"));

    for (int i = 0; i < obs.players.length; i++) {
      toSubmit.add(Factory.createPredicate("object(agent" + i + ", agent)"));
      toSubmit.add(Factory.createPredicate("object(basket" + i + ", basket)"));
      toSubmit.add(Factory.createPredicate("at(basket" + i + ":basket, basketReturn0:basketReturn)"));
    }

    for (int i = 0; i < obs.shelves.length; i++) {
      SupermarketObservation.Shelf shelf = obs.shelves[i];
      toSubmit.add(Factory.createPredicate("object(" + foodName(shelf) + ", food)"));
      toSubmit.add(Factory.createPredicate("object(shelf" + i + ", shelf)"));
      toSubmit.add(Factory.createPredicate(String.format("fluent_equals(amount(shelf%d:shelf,%s:food),%d)", i, foodName(shelf), 10)));
      for (int j = 0; j < obs.players.length; j++) {
        toSubmit.add(Factory.createPredicate(String.format("fluent_equals(amount(basket%d:basket,%s:food),%d)", j, foodName(shelf), 0)));
      }
    }
    for (int i = 0; i < obs.registers.length; i++) {
      toSubmit.add(Factory.createPredicate("object(register" + i + ", register)"));
    }
    for (int i = 0; i < obs.counters.length; i++) {
      SupermarketObservation.Counter counter = obs.counters[i];
      toSubmit.add(Factory.createPredicate("object(counter" + i + ", counter)"));
      toSubmit.add(Factory.createPredicate("object(" + foodName(counter) + ", food)"));
      toSubmit.add(Factory.createPredicate(String.format("fluent_equals(amount(counter%d:counter,%s:food),%d)", i, foodName(counter), 10)));
      for (int j = 0; j < obs.players.length; j++) {
        toSubmit.add(Factory.createPredicate(String.format("fluent_equals(amount(basket%d:basket,%s:food),%d)", j, foodName(counter), 0)));
      }
    }
    for (int i = 0; i < obs.basketReturns.length; i++) {
      toSubmit.add(Factory.createPredicate("object(basketReturn" + i + ", basketReturn)"));
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, toSubmit);
    } catch (TRADEException e) {
      log.error("assertBeliefs not found", e);
    }
  }

  //////////////// SIMULATION //////////////////
  @TRADEService
  public void enableSim() {
    useSim = true;
  }

  @TRADEService
  public void disableSim() {
    useSim = false;
  }

  @TRADEService
  public void setSimObservation() {
    this.simComponent.setObservation(getLastObservation());
  }

  @Override
  public Integer getSimPort() {
    return simSocketPort;
  }

  //////////////// UTILS //////////////////

  private Predicate convertObsToBelief(String belief, boolean truth) {
    if (truth) {
      return (Factory.createPredicate(belief));
    } else {
      return (Factory.createPredicate(belief).toNegatedForm());
    }
  }

  protected String cartName(int cartIndex) {
    return "cart" + cartIndex;
  }

  //    The foodName() functions are needed because the belief component in DIARC doesn't allow food names to contain
//    spaces. If you're, say, a COMP139 student extending this class, and you want to compare a shelf's food to the
//    agent's shopping list, just use shelf.food_name.
  public String foodName(SupermarketObservation.Shelf shelf) {
    return shelf.food_name.replace(" ", "_");
  }

  public String foodName(SupermarketObservation.Counter counter) {
    return counter.food.replace(" ", "_");
  }

  @Override
  public Symbol lastCart() {
    return Factory.createSymbol(cartName(game.observation.carts.length - 1));
  }

  @Override
  public String getAgentName() {
    return this.agentName;
  }

  @Override
  public int getPlayerIndex() {
    return this.playerIndex;
  }

  protected String aisleName(SupermarketObservation.Shelf shelf) {
    return "aisle" + (int) ((shelf.position[1] - 0.5) / 4. + 1);
  }

  protected int shelfIndex(SupermarketObservation.Shelf shelf) {
    return (int) ((shelf.position[0] - 5.5) / 2.);
  }

  protected List<SupermarketObservation.InteractiveObject> allObjects(SupermarketObservation obs) {
    List<SupermarketObservation.InteractiveObject> objs = new ArrayList<>();
    Collections.addAll(objs, obs.registers);
    Collections.addAll(objs, obs.counters);
    Collections.addAll(objs, obs.shelves);
    Collections.addAll(objs, obs.cartReturns);
    Collections.addAll(objs, obs.carts);
    return objs;
  }

  protected SupermarketObservation.InteractiveObject getInteractionTarget(SupermarketObservation obs) {
    for (SupermarketObservation.InteractiveObject obj : obs.registers) {
      if (obj.canInteract(getPlayer())) return obj;
    }
    for (SupermarketObservation.InteractiveObject obj : obs.carts) {
      if (obj.canInteract(getPlayer())) return obj;
    }
    for (SupermarketObservation.InteractiveObject obj : obs.cartReturns) {
      if (obj.canInteract(getPlayer())) return obj;
    }
    for (SupermarketObservation.InteractiveObject obj : obs.counters) {
      if (obj.canInteract(getPlayer())) return obj;
    }
    for (SupermarketObservation.InteractiveObject obj : obs.shelves) {
      if (obj.canInteract(getPlayer())) return obj;
    }
    return null;
  }

  @Override
  public boolean isBusy() {
    log.info(agentName + " is " + busy + " busy");
    return busy;
  }

  @Override
  public void setBusy(boolean busy) {
    this.busy = busy;
  }
}
