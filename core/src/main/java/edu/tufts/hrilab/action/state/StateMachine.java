/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.state;

import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.justification.AndJustification;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.justification.OrJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.fol.util.Utilities;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

import edu.tufts.hrilab.belief.*;

/**
 * @author willie
 */
public class StateMachine {

  private static Logger log = LoggerFactory.getLogger(StateMachine.class);
  private BeliefComponent beliefComponent;
  private Lock lockBeliefComponent = new ReentrantLock();
  private State current;
  private StateMachine simulationStateMachine = null;

  /**
   * Create a new state machine. This also creates a BeliefComponent and registers it with TRADE.
   */
  public StateMachine(String[] beliefArgs) {
    current = State.createRoot();
    beliefComponent = createBeliefComponent(beliefArgs, true);
  }

  /**
   * Create a new state machine by branching off of an existing state machine. The new
   * StateMachine creates
   * a new BeliefComponent initialled with the state of other's BeliefComponent. The new
   * BeliefComponent
   * is NOT registered with TRADE.
   *
   * @param original StateMachine to branch off.
   */
  public StateMachine(StateMachine original) {
    this(original, true);
  }

  /**
   * Create a new state machine by branching off of an existing state machine. If the belief
   * component is not used but the original does, copy all the original's facts to the new state.
   *
   * @param originalSM State machine to be copied from
   * @param useBelief  If belief should be used
   */
  public StateMachine(StateMachine originalSM, boolean useBelief) {
    current = State.createRoot(originalSM.current);
    if (useBelief) {
      try {
        // this cloned belief component is not registered with TRADE
        originalSM.lockBeliefComponent.lock();
        beliefComponent = originalSM.beliefComponent.createClone();
      } finally {
        originalSM.lockBeliefComponent.unlock();
      }
    } else {
      // if the original statemachine contains a belief component, then copy facts from the
      // belief component to current state
      if (originalSM.beliefComponent != null) {
        // Not sure which memory level to use, for now just working memory
        originalSM.getAllFactsFromBelief(MemoryLevel.WORKING).forEach(fact -> current.addFact(new Predicate(fact)));
      }
      beliefComponent = null;
    }
  }

  /**
   * USE FOR TESTS ONLY.
   * Create a new state machine with an initial set of facts for unit tests.
   *
   * @param initFacts initial set of facts
   */
  static public StateMachine createTestStateMachine(Set<Predicate> initFacts) {
    return new StateMachine(initFacts);
  }

  /**
   * Pulls all the facts from this instance's belief into its current state
   */
  public void mergeBelief() {
    Set<Predicate> toAdd = new HashSet<>();
    for (Term t : getAllFacts()) {
      toAdd.add(new Predicate(t));
    }
    this.current.addFacts(toAdd);
  }

  /**
   * Create a new state machine with an initial set of facts.
   *
   * @param initFacts initial set of facts
   */
  private StateMachine(Set<Predicate> initFacts) {
    current = State.createRoot(initFacts);
    beliefComponent = createBeliefComponent(new String[]{}, true);
    Set<Term> initFactsAsTerms = new HashSet<>(initFacts);
    beliefComponent.assertBeliefs(initFactsAsTerms);
  }


  /**
   * Creates a new state machine from an initial state
   *
   * @param state
   */
  public StateMachine(State state) {
    current = State.createRoot(state);
  }

  public StateMachine simulatedStateMachine() {
    StateMachine stateMachine;
    if (simulationStateMachine == null) {
      stateMachine = new StateMachine(this);
      setSimulationStateMachine(stateMachine);
    } else {
      stateMachine = getSimulationStateMachine();
    }
    return stateMachine;
  }

  /**
   * Updates the simulation state to the given state
   *
   * @param s State to update the simulated SM to
   */
  public void setSimulationState(State s) {
    simulationStateMachine.setState(s);
  }

  /**
   * Sets the simulated SM to the instance of the SM passed in
   *
   * @param simSM
   */
  public void setSimulationStateMachine(StateMachine simSM) {
    simulationStateMachine = simSM;
  }

  /**
   * Gets the simulated SM, a standalone instance
   *
   * @return Simulated SM
   */
  public StateMachine getSimulationStateMachine() {
    return simulationStateMachine;
  }

  /**
   * Sets the simulated SM's current state to this instance's currrent state
   *
   * @return
   */
  public StateMachine updateSimStateMachine() {
    setSimulationState(this.current);
    return getSimulationStateMachine();
  }

  /**
   * Update the state based on action effects.
   *
   * @param context Context for which the update is done
   * @return Next/updated state
   */
  public synchronized State update(Context context) {

    // update belief component with assertions and retractions
    List<Predicate> stateUpdates = context.getStateUpdates();
    if (beliefComponent != null) {
      stateUpdates.stream().forEach(update -> {
        if (update.isNegated()) {
          retractBelief(new Predicate(update.toUnnegatedForm()));
        } else {
          assertBelief(update, MemoryLevel.EPISODIC,
                  Factory.createPredicate("context(" + context.getId() + "," + context.getParentContext().getId() + ")"));
        }
      });
    } else {
      log.debug("[update] belief component is null. Can't update belief with. This SM: " + this);
    }

    // update state machine with new state using assertions and retractions
    current = current.update(context);
    return current;
  }


  /**
   * Get all rules from belief
   *
   * @return List of pairs of rule heads and rule bodies
   */
  public List<Pair<Term, List<Term>>> getRules() {
    if(beliefComponent == null ) {
      return new ArrayList<>();
    }
    return beliefComponent.getRules();
  }

  /**
   * Get all facts from belief.
   *
   * @return
   */
  public List<Term> getAllFacts() {
    if (beliefComponent != null) {
      return beliefComponent.getFacts();
    } else {
      return new ArrayList<>(current.getFacts());
    }
  }

  /**
   * Get all facts from belief from specified memory level.
   *
   * @return
   */
  public List<Term> getAllFactsFromBelief(MemoryLevel memoryLevel) {
    return beliefComponent.getFacts(memoryLevel);
  }

  /**
   * Get current state from StateMachine. This does not consult belief.
   *
   * @return
   */
  public State getCurrentState() {
    return current;
  }

  /**
   * Checks that the predicate holds
   *
   * @param predicate predicate
   * @return True if the predicate holds.
   */
  public synchronized Justification holds(Predicate predicate) {
    // check if predicate is a conjunction of predicates (i.e., "and")
    if (predicate.getName().equals("and")) {
      AndJustification justification = new AndJustification();
      predicate.getArgs().forEach(arg -> justification.addJustification(holds((Predicate) arg)));
      return justification;
    }

    // check for negation of predicate
    boolean isNegated = false;
    Predicate predicateWithoutNegation;
    if (predicate.isNegated()) {
      predicateWithoutNegation = (Predicate) predicate.get(0);
      isNegated = true;
    } else {
      predicateWithoutNegation = predicate;
    }

    // solutions with action appropriate variable names (e.g., !x)
    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    boolean holds = false;
    if (beliefComponent != null) {
      List<Map<Variable, Symbol>> solutions = beliefComponent.queryBelief(predicateWithoutNegation);

      log.debug("solutions for condition query '" + predicateWithoutNegation + "' = " + solutions);
      if (solutions == null) {
        log.error("queryBelief returned null for query " + predicateWithoutNegation + ". Error in Belief?");
      } else if (!solutions.isEmpty()) {
        holds = true;
        // Print message if more than one set of solutions is found!
        if (solutions.size() > 1) {
          log.warn("Multiple sets of solutions found for query " + predicateWithoutNegation);
        }
        bindings.addAll(solutions);
      }
    } else {
      // TODO: What should we do when we don't have access to a belief component? (Luca)
      if (SMOperatorCheck.isOperator(predicateWithoutNegation)) {
        holds = SMOperatorCheck.query(predicateWithoutNegation, current);
      } else {
        //strip types off of predicate args. facts that were copied out from belief won't be typed.
        Predicate predicateWithoutNegationOrTypes = Factory.createPredicate(predicateWithoutNegation.getName(),
                predicateWithoutNegation.getArgs()
                        .stream().map(a -> Factory.createSymbol(a.getName()))
                        .collect(Collectors.toList()));
        holds = current.hasFact(predicateWithoutNegation) || current.hasFact(predicateWithoutNegationOrTypes);
      }
    }

    if (isNegated) {
      holds = !holds;
    }

    // Return result with justification
    return new ConditionJustification(holds, predicate, bindings);
  }

  /**
   * Checks that the disjunction of the listed predicates holds
   *
   * @param predicates List of predicates
   * @return True if at least one predicate holds (disjunction).
   */
  public synchronized OrJustification disjunctionHolds(List<Predicate> predicates) {
    OrJustification justification = new OrJustification();

    for (Predicate p : predicates) {
      justification.addJustification(holds(p));

      // if true, no need to check any more disjuncts
      if (justification.getValue()) {
        break;
      }
    }

    // Return result with justification
    return justification;
  }

  /**
   * Checks that the disjunction of the listed predicates holds for a specific assumption.
   *
   * @param predicates List of predicates, the disjunction of which has to hold
   * @param assumption List of predicates that are assumed to hold
   * @return True if at least one predicate holds (disjunction).
   */
  public synchronized OrJustification disjunctionHolds(List<Predicate> predicates,
                                                       List<Predicate> assumption) {
    if (beliefComponent != null && !predicates.isEmpty()) {
      // Add assumption to belief
      List<Predicate> boundAssumptions = assumption.stream()
              .filter(a -> a.getVars().isEmpty())
              .collect(Collectors.toList());
      List<Predicate> unboundAssumptions = assumption.stream()
              .filter(a -> !a.getVars().isEmpty())
              .collect(Collectors.toList());
      if (!unboundAssumptions.isEmpty()) {
        log.warn("[disjunctionHolds] attempting to assert unbound predicates into Belief: " + unboundAssumptions);
      }
      beliefComponent.assertBeliefs(new HashSet<>(boundAssumptions), MemoryLevel.EPISODIC);

      // Verify that disjunction of predicates holds
      OrJustification holds = disjunctionHolds(predicates);

      // Retract assumption
      beliefComponent.retractBeliefs(new HashSet<>(boundAssumptions));

      return holds;
    } else {
      //TODO: What if there is no belief component?
      return new OrJustification();
    }
  }

  public void assertBeliefs(Collection<Predicate> facts) {
    facts.forEach(this::assertBelief);
  }

  //TODO:brad: remove usages of this in favor of memory level + source version
  @Deprecated
  public void assertBelief(Predicate fact) {
    assertBelief(fact, MemoryLevel.EPISODIC);
  }

  //TODO:brad: remove usages of this in favor of memory level + source version
  @Deprecated
  public void assertBelief(Predicate fact, MemoryLevel memoryLevel) {

    assertBelief(fact, memoryLevel, Factory.createPredicate("no(sourceSpecified)"));
  }

  synchronized public void assertBelief(Predicate fact, MemoryLevel memoryLevel, Predicate source) {
    if (beliefComponent != null) {
      // convert fact to prolog form and assert
      beliefComponent.assertBelief(edu.tufts.hrilab.fol.util.Utilities.actionToProlog(fact), memoryLevel, source);
    }
    current.addFact(fact);
  }

  public void retractBelief(Predicate fact) {
    retractBelief(fact, MemoryLevel.WORKING);
  }

  public void retractBelief(Predicate fact, MemoryLevel memoryLevel) {
    if (beliefComponent != null) {
      // convert fact to prolog form and retract
      beliefComponent.retractBelief(Utilities.actionToProlog(fact), memoryLevel);
    }
    retractFactNonBelief(fact);
  }

  public void assertRule(Term head, List<Term> body) {
    beliefComponent.assertRule(head, body);
  }

  public void retractRule(Term head, List<Term> body) {
    beliefComponent.retractRule(head, body);
  }

  public void retractRule(Term head) {
    beliefComponent.retractRule(head);
  }

  /**
   * Gets a rule from belief whose head matches, without arguments
   *
   * @param name Head, without arguments
   * @return
   */
  public Pair<Term, List<Term>> getRule(String name) {
    for (Pair<Term, List<Term>> rule : beliefComponent.getRules()) {
      if (rule.getLeft().getName().equals(name)) {
        return rule;
      }
    }
    return null;
  }

  /**
   * Prune old states from the state machine.
   *
   * @param historyLength ms of history to keep
   */
  public void prune(long historyLength) {
    log.debug("Attempting to prune old history.");

    // find first state older than specified history length
    long pruneTime = System.currentTimeMillis() - historyLength;
    State stateToPrune = current;
    while (stateToPrune != null && stateToPrune.getTimestamp() > pruneTime) {
      stateToPrune = (stateToPrune.getPrevious() != null) ? stateToPrune.getPrevious().getParent() : null;
    }

    // prune history
    if (stateToPrune != null && stateToPrune.getPrevious() != null) {
      log.debug("Pruning state history before: " + stateToPrune.getPrevious().getContext());
      stateToPrune.pruneHistory();
    } else {
      log.debug("No state to prune.");
    }
  }

  /**
   * Helper method to instantiate a new BeliefComponent.
   *
   * CAREFUL: GMs now always make their own Belief Component; if the system starts another one, then
   * two will be available systemwide...
   *
   * @return reference to the requested BeliefComponent
   */
  static private BeliefComponent createBeliefComponent(String[] beliefArgs, boolean registerWithTrade) {
    return DiarcComponent.createInstance(BeliefComponent.class, beliefArgs, registerWithTrade);
  }

  public void shutdown() {
    beliefComponent.shutdown();
  }

  public void updateState(Map<Predicate, Boolean> facts) {
    for (Map.Entry<Predicate, Boolean> fact : facts.entrySet()) {
      Predicate pred = fact.getKey();
      boolean val = fact.getValue();
      boolean isNegated = false;
      if (pred.getName().equals("not")) {
        isNegated = true;
      }
      // TODO: tmf: should we add x if not(x) is false?"
      // if f holds
      // retracted any negated ones
      // assert f
      // if  !f
      // retracted f

      if (val) {
        if (isNegated) {
          retractBelief((Predicate) pred.getArgs().get(0));
        } else {
          assertBelief(pred);
        }
      } else {
        retractBelief(pred);
        if (isNegated) {
          assertBelief((Predicate) pred.getArgs().get(0));
        } // TODO: should assert the condition in the case when it is not negated?
        //    assert 'not(grasping(...))' when grasping(...) is false
      }

      //if ((isNegated && val) || (!isNegated && !val)) {
      //  retractBelief((Predicate) pred.getArgs().get(0));
      //} else if(!isNegated && val){
      //  assertBelief(pred);
      //}
    }
  }

  public Set<Predicate> getStateChange(Context startStep, Context endStep) {
    return current.getStateChange(startStep, endStep);
  }

  /**
   * get how the state changes from the initial state of an action to the completion of an action
   *
   * @param context action to get state change
   * @return set of predicates representing the resulting state the initial state
   */
  public Set<Predicate> getStateChange(Context context) {
    return current.getStateChange(context);
  }

  public State findStartingState(Context context) {
    return current.findStartingState(context);
  }

  public State findStartingState(Context context, boolean isTerminated) {
    return current.findStartingState(context, isTerminated);
  }

  public State findResultingState(Context context) {
    return current.findResultingState(context);
  }

  //todo: should this be done down in State? this fix definitely matters for PA and simulation,
  // but not clear if it's an issue elsewhere.

  private void retractFactNonBelief(Predicate fact) {
    Predicate untypedFact = Factory.createPredicate(fact.toUntypedString());
    current.removeFact(fact);
    current.removeFact(untypedFact);
  }

  //todo: I need this for simulation but it REALLY shouldn't be here. maybe it's time to actually
  // use the simulation
  // machine in this component?
  private void setState(State s) {
    current = s;
  }
}
