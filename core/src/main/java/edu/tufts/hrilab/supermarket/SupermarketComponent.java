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
import edu.tufts.hrilab.action.annotations.Observes;
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
    protected StateMachine sm=null;
    protected boolean useSim = false;

    public SupermarketComponent() {
    }

    public SupermarketComponent(SupermarketInterface other) {
        this.game = new GamePlay(other.getSimPort());
        this.sm = new StateMachine(other.getStateMachine(), false); //probably not necessary
        this.agentName = other.getAgentName();
        this.playerIndex = other.getPlayerIndex();
        this.useBelief = false;
    }

    @Override
    protected void init() {
        super.init();
        this.game = new GamePlay(this.socketPort);
        // TODO: StateMachine is not serializable so this should not be done. Find another way to seed the local SM.
        log.error("StateMachine initialized is not implemented.");
        this.sm = new StateMachine(new String[0]);
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
            if (target instanceof SupermarketObservation.CartReturn) {
                transmitNewCartBeliefs();
            }
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

    @TRADEService
    @Action
    @Observes({"inAisleHub(?x)", "inRearAisleHub(?x)", "northOfCartReturn(?x)", "southOfCartReturn(?x)",
            "atCartReturn(?x)", "northOfExitRow(?x)", "southOfExitRow(?x)", "inStore(?x)", "inEntrance(?x)"})
    public List<Map<Variable, Symbol>> navigationObserver(Term state) {
        log.debug("Observing " + state);
        SupermarketObservation obs;
        if (useSim) {
            obs = simComponent.getLastObservation();
        } else {
            obs = getLastObservation();
        }
        ;
        boolean check = false;
        switch (state.getName()) {
            case "inAisleHub":
                check = obs.inAisleHub(this.playerIndex);
                break;
            case "inRearAisleHub":
                check = obs.inRearAisleHub(this.playerIndex);
                break;
            case "northOfCartReturn":
                check = obs.northOfCartReturn(this.playerIndex);
                break;
            case "southOfCartReturn":
                check = obs.southOfCartReturn(this.playerIndex);
                break;
            case "atCartReturn":
                check = obs.atCartReturn(this.playerIndex);
                break;
            case "northOfExitRow":
                check = obs.northOfExitRow(obs.players[this.playerIndex]);
                break;
            case "southOfExitRow":
                check = obs.southOfExitRow(obs.players[this.playerIndex]);
                break;
            case "inStore":
                check = obs.inStore(obs.players[this.playerIndex]);
                break;
            case "inEntrance":
                check = obs.inEntrance(this.playerIndex);
                break;
        }
        List<Map<Variable, Symbol>> bindings = new ArrayList<>();
        if (check) { //if found the observation
            bindings.add(new HashMap<>());
        }
        return bindings;
    }

    @TRADEService
    @Action
    @Observes({"inAisle(?x, ?aisle)", "northOfAisle(?x, ?aisle)", "southOfAisle(?x, ?aisle)"})
    public List<Map<Variable, Symbol>> aisleObserver(Term state) {
        log.debug("Observing " + state);
        SupermarketObservation obs;

        //Use the simulation observation if in sim mode
        if (useSim) {
            obs = simComponent.getLastObservation(); //can also return objectObserver in sim, same thing
        } else {
            obs = getLastObservation();
        }

        //Observation value
        boolean check = false;

        //Get the instance of the object you want to obseve
        int objNumber = Integer.parseInt(state.get(1).getName().replaceAll("\\D", ""));
        switch (state.getName()) {
            case "inAisle":
                check = obs.inAisle(playerIndex, objNumber);
                break;
            case "northOfAisle":
                check = obs.northOfAisle(playerIndex, objNumber);
                break;
            case "southOfAisle":
                check = obs.southOfAisle(playerIndex, objNumber);
                break;
        }

        List<Map<Variable, Symbol>> bindings = new ArrayList<>();
        //If the observation was found
        if (check) {
            bindings.add(new HashMap<>());
        }
        return bindings;
    }

    @TRADEService
    @Action
    @Observes({"northOf(?x, ?object)", "southOf(?x, ?object)", "eastOf(?x, ?object)",
            "westOf(?x, ?object)", "atShelf(?x, ?shelf)", "canInteractWith(?x, ?object)"})
    public List<Map<Variable, Symbol>> objectObserver(Term state) {
        SupermarketObservation obs;
        log.debug("Observing " + state);

        //Use the simulation observation if in sim mode
        if (useSim) {
            obs = simComponent.getLastObservation(); //can also return objectObserver in sim, same thing
        } else {
            obs = getLastObservation();
        }

        //Observation value
        boolean check = false;

        //Get the instance of the object you want to obseve
        int objNumber = Integer.parseInt(state.get(1).getName().replaceAll("\\D", ""));
        String objType = state.get(1).getName().replaceAll("\\d","");
        SupermarketObservation.InteractiveObject obj = null;
        switch (objType) {
            //northOf, southOf
            case "counter":
                obj = obs.counters[objNumber];
                break;
            //northOf, southOf, eastOf, southOf
            case "register":
                obj = obs.registers[objNumber];
                break;
            //eastOf, westOf, or atShelf
            case "shelf":
                obj = obs.shelves[objNumber];
                break;
            //only canInteractWith
            case "cart":
                obj = obs.carts[objNumber];
                break;
        }

        if (obj == null) {
            log.error("Unknown type: " + objType);
            return null;
        }

        switch (state.getName()) {
            case "canInteractWith":
                check = obj.canInteract(obs.players[playerIndex]);
                break;
            case "southOf":
                check = obs.southOf(obs.players[playerIndex], obj);
                break;
            case "northOf":
                check = obs.northOf(obs.players[playerIndex], obj);
                break;
            case "eastOf":
                check = obs.eastOf(obs.players[playerIndex], obj);
                break;
            case "westOf":
                check = obs.westOf(obs.players[playerIndex], obj);
                break;
            case "atShelf":
                check = obs.atShelf(obs.players[playerIndex], obj);
                break;
        }

        List<Map<Variable, Symbol>> bindings = new ArrayList<>();
        //If the observation was found
        if (check) {
            bindings.add(new HashMap<>());
        }
        return bindings;
    }

//
//        for (int i = 0; i < obs.players.length; i++) {
//            if (i == this.playerIndex) continue;
//            this.beliefMap.put("fluent_equals(proximity(" + this.agentName + ", agent" + i + "), " +
//                    (int) Math.ceil(obs.proximity(this.playerIndex, i)) + ")", true);
//        }

    protected void transmitPlayerInitialBeliefs() {
        SupermarketObservation obs = game.observation;
        this.sm.assertBelief(Factory.createPredicate("object(" + this.agentName + ", agent)"));
        this.sm.assertBelief(Factory.createPredicate("handsFree(" + this.agentName + ")"));

        for (int i = 0; i < obs.shelves.length; i++) {
            SupermarketObservation.Shelf shelf = obs.shelves[i];
            SupermarketObservation.Player player = obs.players[this.playerIndex];
            int index = Arrays.asList(player.shopping_list).indexOf(shelf.food_name);
            this.sm.assertBelief(Factory.createPredicate("fluent_equals(inventoryQuantity( " + this.agentName + ":agent," + foodName(shelf) + ":food), 0)"));
            if (index == -1) {
                this.sm.assertBelief(Factory.createPredicate("fluent_equals(shoppingListQuantity( " + this.agentName + ":agent," + foodName(shelf) + ":food), 0)"));
            } else {
                this.sm.assertBelief(Factory.createPredicate("fluent_equals(shoppingListQuantity( " + this.agentName + ":agent," + foodName(shelf) + ":food), " + player.list_quant[index] + ")"));
            }
        }

        for (int i = 0; i < obs.counters.length; i++) {
            SupermarketObservation.Counter counter = obs.counters[i];
            int index = Arrays.asList(obs.players[this.playerIndex].shopping_list).indexOf(counter.food);
            this.sm.assertBelief(Factory.createPredicate("fluent_equals(inventoryQuantity( " + this.agentName + ":agent," + foodName(counter) + ":food), 0)"));
            if (index == -1) {
                this.sm.assertBelief(Factory.createPredicate("fluent_equals(shoppingListQuantity( " + this.agentName + ":agent," + foodName(counter) + ":food), 0)"));
            } else {
                this.sm.assertBelief(Factory.createPredicate("fluent_equals(shoppingListQuantity( " + this.agentName + ":agent," + foodName(counter) + ":food), " + obs.players[this.playerIndex].list_quant[index] + ")"));
            }
        }
    }

    protected void transmitInitialBeliefs() {
        SupermarketObservation obs = game.observation;

        Set<Predicate> toSubmit = new HashSet<>();

        for (int i = 0; i < obs.players.length; i++) {
            toSubmit.add(Factory.createPredicate("object(agent" + i + ", agent)"));
        }

        for (int i = 0; i < obs.shelves.length; i++) {
            SupermarketObservation.Shelf shelf = obs.shelves[i];
            toSubmit.add(Factory.createPredicate("object(" + foodName(shelf) + ", food)"));
            toSubmit.add(Factory.createPredicate("object(shelf" + i + ", shelf)"));
            toSubmit.add(Factory.createPredicate("stockedAt(" + foodName(shelf) + ", shelf" + i + ":shelf)"));
            toSubmit.add(Factory.createPredicate("shelfInAisle(shelf" + i + ", " + aisleName(shelf) + ")"));
            if (shelfIndex(shelf) == 0) {
                toSubmit.add(Factory.createPredicate("firstShelf(" + aisleName(shelf) + ", shelf" + i +
                        ":shelf)"));
            } else if (shelfIndex(shelf) == 4) {
                toSubmit.add(Factory.createPredicate("lastShelf(" + aisleName(shelf) + ", shelf" + i + ")"));
            }
        }
        for (int i = 0; i < obs.registers.length; i++) {
            toSubmit.add(Factory.createPredicate("object(register" + i + ", register)"));
        }
        for (int i = 0; i < obs.counters.length; i++) {
            SupermarketObservation.Counter counter = obs.counters[i];
            toSubmit.add(Factory.createPredicate("object(counter" + i + ", counter)"));
            toSubmit.add(Factory.createPredicate("object(" + foodName(counter) + ", food)"));
            toSubmit.add(Factory.createPredicate("stockedAt(" + foodName(counter) + ", counter" + i + ")"));
        }
        for (int i = 0; i < obs.cartReturns.length; i++) {
            toSubmit.add(Factory.createPredicate("object(cartReturn" + i + ", cartReturn)"));
        }
        sm.assertBeliefs(toSubmit);
    }

    @Override
    public void transmitNewCartBeliefs() {
        if (useSim) {
            simComponent.transmitNewCartBeliefs();
        } else {
            SupermarketObservation obs = game.observation;
            Set<Predicate> toSubmit = new HashSet<>();

            SupermarketObservation.Cart newCart = obs.carts[obs.carts.length - 1];
            String cartString = cartName(obs.carts.length - 1);
            toSubmit.add(Factory.createPredicate("object(" + cartString + ", cart)"));
            toSubmit.add(Factory.createPredicate("fluent_equals(generalFoodQuantity(" + cartString + ":cart), 0)"));

            for (SupermarketObservation.Shelf shelf : obs.shelves) {
                toSubmit.add(Factory.createPredicate("fluent_equals(specificFoodQuantity(" + cartString + ":cart, " + foodName(shelf) + ":food), 0)"));
            }

            toSubmit.add(Factory.createPredicate("fluent_equals(capacity(" + cartString + ":cart), " + newCart.capacity + ")"));
            toSubmit.add(Factory.createPredicate("holdingCart(" + this.agentName + ":agent, " + cartString + ":cart)"));
            sm.assertBeliefs(toSubmit);
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

    @TRADEService
    public StateMachine getSimStateMachine() {
        return this.simComponent.getStateMachine();
    }

    @TRADEService
    public void setSimStateMachine(StateMachine sm) {
        this.simComponent.setStateMachine(sm);
    }

    @Override
    public Integer getSimPort() {
        return simSocketPort;
    }

    //////////////// UTILS //////////////////
    @Override
    public StateMachine getStateMachine() {
        return this.sm;
    }

    @Override
    public void setStateMachine(StateMachine sm) {
        this.sm = sm;
    }

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
            if (obj.canInteract(obs.players[this.playerIndex])) return obj;
        }
        for (SupermarketObservation.InteractiveObject obj : obs.carts) {
            if (obj.canInteract(obs.players[this.playerIndex])) return obj;
        }
        for (SupermarketObservation.InteractiveObject obj : obs.cartReturns) {
            if (obj.canInteract(obs.players[this.playerIndex])) return obj;
        }
        for (SupermarketObservation.InteractiveObject obj : obs.counters) {
            if (obj.canInteract(obs.players[this.playerIndex])) return obj;
        }
        for (SupermarketObservation.InteractiveObject obj : obs.shelves) {
            if (obj.canInteract(obs.players[this.playerIndex])) return obj;
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
