/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.state;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.regex.Pattern;

import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author willie
 */
public class State {
  private static Logger log = LoggerFactory.getLogger(State.class);

  private Set<Predicate> facts;
  private Transition previous = null;
  private Transition next = null;
  private final long timestamp;

  public State(State previous) {
    timestamp = System.currentTimeMillis();

    if (previous != null) {
      if (previous.facts != null) {
        facts = new HashSet<>(previous.facts);
      } else {
        facts = new HashSet<>();
      }
    }
  }

  protected State(Set<Predicate> init) {
    timestamp = System.currentTimeMillis();

    if (init == null) {
      log.warn("Initial list of predicates is null.  Creating an empty list.");
      facts = new HashSet<>();
    } else {
      facts = new HashSet<>(init);
    }
  }

  public static State createRoot() {
    return new State(new HashSet<>());
  }

  public static State createRoot(Set<Predicate> fs) {
    return new State(fs);
  }

  public static State createRoot(State s) {
    State root = new State(new HashSet<>(s.facts));
    root.previous = s.previous;
    return root;
  }

  public long getTimestamp() {
    return timestamp;
  }

  /**
   * Create the next State based on the Context and the stateUpdates that have been
   * extracted from the Context. The stateUpdates are an ordered list of Predicates where the boolean
   * indicates assertion (true) or retraction (false).
   *
   * @param context
   * @return
   */
  public synchronized State update(Context context) {
    State nextState = new State(this);
    Transition t = new Transition(context);
    this.next = t;
    nextState.previous = t;
    t.parent = this;
    t.child = nextState;

    // add or remove effects to list of facts
    context.getStateUpdates().stream().forEach(update -> {
      if (update.isNegated()) {
        nextState.removeFact((Predicate) update.get(0));
      } else {
        nextState.addFact(update);
      }
    });

    // TODO: EAK: this currently asserts the same justification at every level of the context tree.
    //  need to figure out a way to only assert once. maybe by not passing the same justification
    //  up the context tree during exit?
    if (context.getStatus().isFailure() && !context.getJustification().getFailureReason().isEmpty()) {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, new HashSet<>(context.getJustification().getFailureReason()));
      } catch (TRADEException e) {
        log.error("Error asserting justification to Belief.", e);
      }
    }

    return nextState;
  }

  public boolean addFacts(Collection<Predicate> facts) {
    for (Predicate fact : facts) {
      if (!this.facts.contains(fact)) {
        addFact(fact);
      }
    }
    return true;
  }

  public boolean addFact(Predicate fact) {
    return addFact(fact, true);
  }

  public boolean addFact(Predicate fact, boolean checkOperator) {
    if (checkOperator && SMOperatorCheck.isOperator(fact)) {
      return SMOperatorCheck.assertFluent(fact, this);
    } else {
      Predicate negFact = Factory.createNegatedPredicate(fact);
      facts.remove(negFact);
      return facts.add(fact);
    }
  }

  public boolean removeFacts(Collection<Predicate> facts) {
    for (Predicate fact : facts) {
      if (!removeFact(fact)) {
        return false;
      }
    }
    return true;
  }

  public boolean removeFact(Predicate fact) {
    return facts.remove(fact);
  }

  boolean hasFact(Predicate boundCond) {
    for (Predicate p : facts) {
      if (p.getName().equalsIgnoreCase(boundCond.getName())) {
        if (p.getArgs().equals(boundCond.getArgs())) {
          return true;
        }
      }
    }
    return false;
  }

  public Set<Predicate> getFacts() {
    return facts;
  }

  public Transition getPrevious() {
    return previous;
  }

  public State findResultingState(Context context) {
    Transition tmpPrevious = previous;
    State resultingState = this;
    while (!tmpPrevious.getContext().equals(context)) {
      resultingState = tmpPrevious.getParent();
      tmpPrevious = resultingState.getPrevious();
    }
    return resultingState;
  }

  /**
   * get the state prior to executing an action
   *
   * @param context action which want to get state prior to execution
   * @return state prior to execution of action
   */
  // should function check the resulting state for 'did()' or goal state to determine if it is the first or second
  //   instance in the state machine for that context
  public State findStartingState(Context context) {
    return findStartingState(context, true);
  }

  public State findStartingState(Context context, boolean isTerminated) {
    if (previous == null) {
      log.error("context: " + context.getSignatureInPredicateForm() + " doesn't exist in state machine");
      return null;
    }
    int totalInstances = isTerminated ? 2 : 1;
    if (totalInstances == 1) { // for some reason the action could be terminated, but not show as terminated initially
      try {
        log.debug("action instances in sm: " + totalInstances);
        Thread.sleep(1000);
        totalInstances = isTerminated ? 2 : 1;
        log.debug("action instances in sm: " + totalInstances);
      } catch (InterruptedException ex) {
      }
    }

    int instancesFound = 0;
    Transition priorAction = previous;
    State initialState = null;
    while (instancesFound < totalInstances) {
      try {
        if (priorAction.getContext().equals(context)) {
          instancesFound++;
          initialState = priorAction.getParent();
        }
        priorAction = priorAction.getParent().getPrevious();
      } catch (NullPointerException e) {
        log.warn("[findStartingState] not able to find initial state within state machine");
        return null;
      }
    }
    return initialState;
  }

  /**
   * get how the state changes from the initial state of an action to the completion of an action
   *
   * @param context action to get state change
   * @return set of predicates representing the resulting state the initial state
   */
  public Set<Predicate> getStateChange(Context context) {
    return getStateChange(context, context);
  }

  public Set<Predicate> getStateChange(Context startStep, Context endStep) {
    if (previous == null) {
      log.error("context: " + startStep.getSignatureInPredicateForm() + " doesn't exist in state machine");
      return null;
    }

    State finalState = null;
    State initialState = null;
    int totalInstances = endStep.isTerminated() ? 2 : 1;
    if (totalInstances == 1) { // for some reason the action could be terminated, but not show as terminated initially
      try {
        log.debug("action instances in sm: " + totalInstances);
        Thread.sleep(1000);
        totalInstances = endStep.isTerminated() ? 2 : 1;
        log.debug("action instances in sm: " + totalInstances);
      } catch (InterruptedException ex) {
      }
    }
    if (!startStep.equals(endStep)) {
      totalInstances -= 1; // remove one instance from end step, because first instance won't be used (prior to start step execution)
      if (totalInstances == 0) {
        finalState = this;
      }
      int stepTerminated = startStep.isTerminated() ? 2 : 1;
      totalInstances += stepTerminated;
    }
    int instancesFound = 0; // need to find end and start. should it consider if an action hasn't terminated?
    Transition priorAction;
    priorAction = previous;
    // TODO: test non-terminated steps: not sure if non-terminated steps are handled properly
    while (instancesFound < totalInstances) {
      try {
        if (instancesFound == 0) {
          if (priorAction.getContext().equals(endStep)) {
            instancesFound = 1;
            if (finalState == null) {
              finalState = priorAction.getChild();
            }
          }
        } else {
          if (priorAction.getContext().equals(endStep) || priorAction.getContext().equals(startStep)) {
            instancesFound++;
            initialState = priorAction.getParent();
          }
        }
        priorAction = priorAction.getParent().getPrevious();
      } catch (NullPointerException e) {
        log.warn("[getStateChange] not able to find action: " + startStep.getSignatureInPredicateForm() + " within state machine");
        return null;
      }
    }
    if (initialState == null || finalState == null) {
      log.warn("[getStateChange] initialState or finalState not found");
      return null;
    }

    HashSet<Predicate> stateChange = new HashSet<>();
    // FIXME: should not loop through entire state space...
    //        plus this doesn't consider retracted facts
    for (Predicate fact : getFacts()) { // loop through current state
      if (!initialState.hasFact(fact)) { // current fact state not in initial state
        stateChange.add(fact); // add fact to state change
      }
    }
    return stateChange;
  }

  /**
   * Sets the previous transition to null, effectively removing the state transition
   * history leading up to this state.
   */
  public void pruneHistory() {
    if (previous != null) {
      previous.child = null;
      previous = null;
    }
  }

  public List<Map<Variable, Symbol>> queryFluentValue(Term query) {
    List<Map<Variable, Symbol>> results = new ArrayList<>();
    //todo: this assumes you have a var as your second arg
    Variable valueVar = (Variable) query.get(1);
    String patternString = Factory.createPredicate(query.getName(), query.get(0).toString(), "\\d+\\.?\\d*").toString();
    Pattern pattern = Pattern.compile(patternString.replace(")", "\\)").replace("(", "\\("));
    for (Predicate p : facts) {
      if (pattern.matcher(p.toString()).matches()) {
        Map<Variable, Symbol> match = new HashMap<>();
        match.put(valueVar, p.get(1));
        results.add(match);
      }
    }
    return results;
  }

  public static class Transition {
    private Context context;
    private State parent;
    private State child;

    public Transition(Context context) {
      this.context = context;
    }

    /**
     * Get state that existed before this transition (i.e., the previous state).
     *
     * @return
     */
    public State getParent() {
      return parent;
    }

    /**
     * Get state that results from this transition (i.e., the next state).
     *
     * @return
     */
    public State getChild() {
      return child;
    }

    public Context getContext() {
      return context;
    }

  }
}
