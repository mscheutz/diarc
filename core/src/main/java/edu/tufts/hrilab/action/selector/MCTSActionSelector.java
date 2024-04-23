/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.selector;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.mcts.AgentModel;
import edu.tufts.hrilab.action.mcts.MCTSNode;
import edu.tufts.hrilab.action.mcts.MCTSUtils;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import com.google.common.collect.Lists;
import edu.tufts.hrilab.pddl.Operators;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class MCTSActionSelector extends ActionSelector {

  private static final Logger log = LoggerFactory.getLogger(MCTSActionSelector.class);
  List<ActionDBEntry> userActions;
  Symbol actor;
  List<AgentModel> rivals = new ArrayList<>(); //todo: rival order might matter
  ActionSelector planner;
  PreconditionPlanningSelector planChecker;

  MCTSActionSelector() {
    planner = new GoalPlanningActionSelector();
    planChecker = new PreconditionPlanningSelector();
  }

  /**
   * Entry point to MCTS. Only runs MCTS if the goal is reachable.
   *
   * @param goal         the goal that the selected action should accomplish
   * @param constraints  a returned goal shold satisfy any constraints
   * @param stateMachine in case the action selector cares about the state
   * @return
   */
  @Override
  public ParameterizedAction selectActionForGoal(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    //todo: change to precheck
    log.debug("Selecting action for goal: " + goal);
    ParameterizedAction selectedAction = planChecker.selectActionForGoal(goal, constraints, stateMachine);
    if (selectedAction != null) {
      return selectedAction;
    }

    //Can the player reach the final state?
    if (planner.selectActionForGoal(goal, constraints, stateMachine) == null) {
      log.debug("[MCTS] Initial goal not reachable");
      return null;
    }

    MCTSNode root = initializeMcts(goal);
    userActions = filterActionsByGoal(Database.getActionDB().getAllActions(), goal, constraints, stateMachine);
    log.debug("[MCTS] " + userActions.size() + " user actions populated");


    ActionConstraints mctsConstraints = new ActionConstraints();
    for (ActionDBEntry entry : userActions) {
      mctsConstraints.addAllowedAction(entry.getName());
    }

    seedTree(goal, stateMachine, mctsConstraints, root);
    int budget = 2000; //todo: make param
    mcts(root, budget, mctsConstraints, goal); //should I add constraints here?
    return createMCTSAction(root, goal, constraints);
  }

  /**
   * Initializes the root node, viable user actions, and viable rival actions.
   *
   * @param goal
   * @return The root node of the MCTS tree
   */
  private MCTSNode initializeMcts(Goal goal) {
    //Initialize MCTS
    //todo: make this cleaner
    actor = goal.getActor();
    MCTSNode root = new MCTSNode(goal.getCurrentContext().createSimulatedEquivalent(false), null, actor);

    //todo: make root use defaultGoalRecovery

    root.setVisits(1);
//    userActions = filterActions(Database.getActionDB().getAllActions());
    populateRivals(userActions);
    return root;
  }

  /**
   * Core loop of MCTS. Explores the tree for a given number of iterations.
   *
   * @param root        Root MCTS Node
   * @param budget      Number of individual nodes to explore.
   * @param constraints Action, state, and actor constraints
   * @param goal        Terminal state
   */
  private void mcts(MCTSNode root, int budget, ActionConstraints constraints, Goal goal) {
    //MCTS Loop
    for (int i = 0; i < budget; i++) {
      //get best child
      log.trace("Getting best child");
      MCTSNode bestChild = root.getBestChild();
      MCTSNode rolloutNode;

      if (bestChild.getVisits() == 0) { //Rollout on first visit
        rolloutNode = bestChild;
      } else {
        if (!bestChild.isExpanded()) { //Node hasn't been expanded
          expandNode(bestChild); // expand the children of the best child
          if (bestChild.getChildren().isEmpty()) { //Dead end state. Todo: check if in goal state
            bestChild.backup(-100);
            continue;
          }
        }
        rolloutNode = getUnexploredNode(bestChild); // choose random unevaluated node
      }

      // rollout node
      int value;
      value = rollout(rolloutNode, goal, constraints);
      //back up the tree!
      rolloutNode.backup(value);
      //If we have reached our goal
      if (rolloutNode.getContext().getStateMachine().holds(goal.getPredicate()).getValue()) {
        log.debug("[MCTS] reached goal");
        break;
      }
    }
  }

  /**
   * After MCTS is done, create an action called "mctsplanned" using the best found path. Each
   * node in this path is an eventspec.
   *
   * @param root        Root MCTS node
   * @param goal
   * @param constraints
   * @return A parameterized action containing the final plan of MCTS
   */
  private ParameterizedAction createMCTSAction(MCTSNode root, Goal goal, ActionConstraints constraints) {
    //Create a new action for the path
    ActionDBEntry.Builder combinedADB = new ActionDBEntry.Builder("mctsplanned");
    MCTSNode bestChild = root.getBestChild();

    //In most cases, the best child will not have been executed yet.
    if (bestChild.getContext() == null) {
      bestChild = bestChild.getParent();
    }

    //For each node, create event spec
    StateMachine lastState = bestChild.getContext().getStateMachine();
    while (bestChild != null) {
      //add event specs to an actionDBEntry
      if (bestChild.getContext() instanceof ActionContext) {
        String especString = reverseFilter(bestChild.toActionDesc());
        EventSpec es = new EventSpec(EventSpec.EventType.ACTION, especString);
        combinedADB.prependEventSpecs(List.of(es));
      } else if (bestChild.getContext() instanceof GoalContext) {
        for (ActionDBEntry action : userActions) {
          Database.getActionDB().removeAction(action);
        }

        ParameterizedAction remainingPlan = planner.selectActionForGoal(goal, constraints, lastState);
        for (EventSpec eventSpec : remainingPlan.getEntry().getEventSpecs()) {
          combinedADB.addEventSpec(new EventSpec(EventSpec.EventType.ACTION, eventSpec.toString()));
        }
      }
      bestChild = bestChild.getParent();
    }
    return new ParameterizedAction(combinedADB.build(true), goal);
  }

  /**
   * Initializes a path through the tree based off of a symbolic plan. This essentially biases
   * the tree towards taking this path, which doesn't consider any rival actions or any
   * stochastic perturbations.
   *
   * @param goal         Terminal state of the plan
   * @param stateMachine Initial state of the world
   * @param constraints  Constraints on any actions, states, or actors
   * @param root         MCTS root node
   */
  private void seedTree(Goal goal, StateMachine stateMachine, ActionConstraints constraints, MCTSNode root) {

    ParameterizedAction mctsplan = planner.selectActionForGoal(goal, constraints, stateMachine);
    if (mctsplan == null) {
      log.debug("[MCTS] Could not plan with constraints");
    }
    for (EventSpec entry : mctsplan.getEntry().getEventSpecs()) {
      ActionDBEntry action = Database.getActionDB().getAction(entry.getCommand());
      if (action.getRequiredInputRoles().size() != entry.getInputArgs().size()) {
        log.error("Action mismatch with entry. This will definitely break something.");
        continue;
      }
      Map<String, Object> bindings = new HashMap<>();
      for (int i = 0; i < action.getRequiredInputRoles().size(); i++) {
        bindings.put(action.getRequiredInputRoles().get(i).name, entry.getInputArgs().get(i));
      }
      bindings.put("?actor", entry.getActor());

      MCTSNode seededNode = new MCTSNode(action, bindings, root, root.getAgent());
      seededNode.setScore(10);
      seededNode.setVisits(1);
      root.getChildren().add(seededNode);
      root = seededNode;
    }

  }

  private void populateRivals(List<ActionDBEntry> actions) {
    //todo: figure out if agents have same actions as user
    Collection<TRADEServiceInfo> rivals = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getActor"));
    for (TRADEServiceInfo rival : rivals) {
      try {
        Symbol actor = rival.call(Symbol.class);
//                List<String> availableActions = (List<String>)TRADE.callThe(rival,
//                "getAvailableActions");
        this.rivals.add(new AgentModel(actor, actions));
      } catch (TRADEException e) {
        throw new RuntimeException(e);
      }
    }
  }

  /**
   * Given a parent, find a node that is unexplored, based on its score.
   *
   * @param node Node to check for the best unexplored child
   * @return An unexplored node
   */
  private MCTSNode getUnexploredNode(MCTSNode node) {
    List<MCTSNode> unexploredNodes = new ArrayList<>();

    for (MCTSNode child : node.getChildren()) {
      if (!child.isExpanded()) {
        unexploredNodes.add(child);
      }
    }
    Collections.shuffle(unexploredNodes); //todo: this is a hack to ensure duplicates aren't received
    // are random
    return unexploredNodes.stream().max(Comparator.comparing(MCTSNode::getScore)).get();
//        return unexploredNodes.get(random.nextInt(unexploredNodes.size()));
  }

  //

  /**
   * Rollout/execute a node using its parents state
   *
   * @param rolloutNode Node to execute
   * @param goal
   * @param constraints
   * @return
   */
  private int rollout(MCTSNode rolloutNode, Goal goal, ActionConstraints constraints) {
    log.trace("Rolling out ");

    rolloutNode.generateContext();

    //Is the player action viable?
    if (!rolloutNode.getContext().isApproved().getValue()) {
      return -100;
    }

    //Is the player action successful?
    Goal rolloutGoal = new Goal(Factory.createSymbol("self"), new Predicate("rollout", new String[0]), Observable.FALSE);

    ActionInterpreter ai = ActionInterpreter.createInterpreterFromContext(rolloutGoal, rolloutNode.getContext());
    ActionStatus status = ai.call();
    if (status.isFailure()) {
      return -100;
    }

    //Simulate rivals' actions (not MCTS nodes, just contexts)
    Context nextContext = ai.getRoot();
    for (AgentModel rival : rivals) {
      nextContext = rival.executeLikelyAction(nextContext, constraints);
    }

    rolloutNode.setContext(nextContext);

    //Can the player reach the final state?
    constraints.addAllowedActor("self");
    ParameterizedAction rollout = planner.selectActionForGoal(goal, constraints, nextContext.getStateMachine());
    constraints.removeAllowedActor("self");
    if (rollout == null) {
      return -100;
    }
    return 100;
  }

  /**
   * Adds all valid child nodes
   *
   * @param root
   */
  private void expandNode(MCTSNode root) {
    log.trace("Expanding");
    root.addChildren(getValidChildren(root, actor));
    root.setExpanded(true);
  }

  /**
   * Return all actions that are possible in a given context (from parent)
   *
   * @param parent Parent node used for its context/state
   * @param agent  Agent to see what actions are available
   * @return
   */
  private List<MCTSNode> getValidChildren(MCTSNode parent, Symbol agent) {
    log.trace("Getting valid children");
    List<MCTSNode> validChildren = new ArrayList<>();

    List<ActionDBEntry> availableActions;
    availableActions = this.userActions; //todo: change available actions to be agent specific

    for (ActionDBEntry action : availableActions) {
      List<List<Pair<String, Symbol>>> possibleBindings = MCTSUtils.populateBindings(action.getInputRoles());
      List<List<Pair<String, Symbol>>> permutations = Lists.cartesianProduct(possibleBindings);
      //todo: need to add agents in the system somewhere...
      for (List<Pair<String, Symbol>> permutation : permutations) {
        Map<String, Object> bindings = new HashMap<>();
        bindings.put("?actor", agent);
        for (Pair<String, Symbol> binding : permutation) {
          bindings.put(binding.getKey(), binding.getValue());
        }
        validChildren.add(new MCTSNode(action, bindings, parent, agent));
      }
    }

    return validChildren;
  }

  /**
   * Undoes the filter for the very primitive MCTS specific action
   *
   * @param entry
   * @return
   */
  private String reverseFilter(String entry) {
    return entry.replace("mcts", "");
  }

  private boolean isRelevant(Set<String> relevantEffects, Predicate p) {
    if (Operators.DIARC_TO_PDDL.containsKey(p.getName())) {
      if (relevantEffects.contains(p.getArgs().get(0).getName())) {
        return true;
      } else {
        return false;
      }
    } else if (relevantEffects.contains(p.toUnnegatedForm().getName())) {
      return true;
    } else {
      return false;
    }
  }


  /**
   * Given a list of action db entries, removes all that are not relevant for MCTS
   *
   * @param entries Action DB entries to filter
   * @return Updated list withotu irrelevent actions
   */
  private List<ActionDBEntry> filterActionsByGoal(Set<ActionDBEntry> entries, Goal goal, ActionConstraints constraints, StateMachine sm) {
    //Only look at goal predicates
    Set<String> relevantEffects = new HashSet<>();
    for (Symbol p : goal.getPredicate().getArgs()) {
      if (Operators.DIARC_TO_PDDL.containsKey(p.getName())) {
        relevantEffects.add(((Term) p).getArgs().get(0).getName());
      } else {
        relevantEffects.add(p.toUnnegatedForm().getName());
      }
    }

    relevantEffects.add("stockedAt");
    relevantEffects.add("fluent_assign");

    //Remove all operators without relevant effects
    entries.removeIf(action -> action.getEffects().stream().noneMatch(e -> isRelevant(relevantEffects, e.getPredicate())));

    //Then remove all non-relevant preconds and effects
    List<ActionDBEntry> newEntries = new ArrayList<>();
    for (ActionDBEntry action : entries) {
      ActionDBEntry.Builder actionBuilder = new ActionDBEntry.Builder(action);
      actionBuilder.setType("mcts" + action.getName());
      for (Effect e : action.getEffects()) {
        if (!isRelevant(relevantEffects, e.getPredicate())) {
          actionBuilder.removeEffect(e);
        }
      }

      for (Condition c : action.getConditions()) {
        Set<Predicate> predicates = c.getPredicates().keySet();
        if (predicates.stream().noneMatch(p -> isRelevant(relevantEffects, p))) {
          actionBuilder.removeCondition(c);
        } else {
          actionBuilder.removeCondition(c);
          predicates.forEach(pred -> actionBuilder.addCondition(new Condition(pred, ConditionType.PRE, Observable.FALSE)));
        }
      }
      actionBuilder.clearEventSpecList();
      newEntries.add(actionBuilder.build(true));
    }
    return newEntries;

  }
}
