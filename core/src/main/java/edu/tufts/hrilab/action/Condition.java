/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.ObservationContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.justification.OrJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.Serializable;
import java.util.*;

/**
 * A Condition is a (set of) predicate(s) that has(ve) to hold before or during the execution of an action.
 * It consists of one or more predicates (description of the condition) and a type (pre-condition, overall-condition).
 * <p>
 * Uses LinkedHashMap to keep key insertion order, important when you have disjunctions to keep
 * ordering and evaluation consistent (stop evaluating once the first condition holds).
 */
public class Condition implements Serializable {
  private static Logger log = LoggerFactory.getLogger(Condition.class);
  private static final long serialVersionUID = -3956261989056437765L;
  final private ConditionType type;
  final private LinkedHashMap<Predicate, Observable> predicates; // Condition <-> Observable

  public Condition(Predicate p, ConditionType t, Observable o) {
    type = t;
    predicates = new LinkedHashMap<>();
    predicates.put(p, o);
  }

  public Condition(Predicate p, ConditionType t) {
    this(p, t, Observable.DEFAULT);
  }

  public Condition(Disjunction or) {
    type = or.type;
    predicates = or.predicates;
  }

  public Condition(ConditionType t, LinkedHashMap<Predicate, Observable> p) {
    type = t;
    predicates = p;
  }

  /**
   * Using the roles from the ActionDBEntry, attempt to extract semantic type
   * information from the roles and apply them to the predicate(s) in this Condition.
   * @param actionDBEntry
   */
  public void applySemanticTypes(ActionDBEntry actionDBEntry) {
    LinkedHashMap<Predicate, Observable> predicatesCopy = new LinkedHashMap<>(predicates);
    predicates.clear();

    for (Predicate predicate : predicatesCopy.keySet()) {
        predicates.put(new Predicate(actionDBEntry.applySemanticTypes(predicate)), predicatesCopy.get(predicate));
    }
  }

  public ConditionType getType() {
    return type;
  }

  public boolean isDisjunction() {
    return predicates.entrySet().size() > 1;
  }

  //TODO:brad:this naming isn't the most clear...
  public Map<Predicate, Observable> getPredicates() {
    return Collections.unmodifiableMap(predicates);
  }

  /**
   * Bind each predicate in condition to a context.
   *
   * @param context Context in which to look up variable bindings
   * @return New Condition with unbound arguments bound, if possible.
   */
  public Condition bindToContext(ArgumentBasedContext context) {
    LinkedHashMap<Predicate, Observable> boundPredicates = new LinkedHashMap<>();
    predicates.forEach((pred, obs) -> boundPredicates.put(context.bindPredicate(pred), obs));
    return new Condition(this.type, boundPredicates);
  }

  /**
   * Verify if condition holds.
   *
   * @param context context providing variable bindings
   * @param sm      state machine in which condition is checked
   * @return justification with bindings. true if condition holds.
   */
  public Justification holds(ActionContext context, StateMachine sm) {
    //todo: Figure out how to handle if types need to hold. Maybe not for constants?
    if (predicates.isEmpty()) {
      log.error("[holds] empty predicates to check. This shouldn't happen. ");
      return new ConditionJustification(false);
    } else if (isDisjunction()) {
      OrJustification justification = new OrJustification();
      for (Map.Entry<Predicate, Observable> condition : predicates.entrySet()) {
        justification.addJustification(observeCondition(condition, context, sm));
        if (justification.getValue()) {
          break;
        }
      }
      return justification;
    } else {
      // if not a disjunction, there is guaranteed to be only one predicate entry
      Map.Entry<Predicate, Observable> condition = predicates.entrySet().iterator().next();
      return observeCondition(condition, context, sm);
    }
  }

  private Justification observeCondition(Map.Entry<Predicate, Observable> condition, ActionContext context, StateMachine sm) {
    Predicate predicate = condition.getKey();
    Observable observe = condition.getValue();

    // needed for simulations where action isn't executed
    // otherwise it will fail when trying to get information from observation
    // for example: performance assessment should look into state machine for condition
    if (!context.getExecType().shouldExecute()) {
      observe = Observable.FALSE;
    }

    if(observe == Observable.DEFAULT || observe == Observable.TRUE) {
      ObservationContext observationContext = context.getConditionObserver(this, predicate);
      if (observationContext != null) {
        Symbol actor = (Symbol) context.getArgumentValue("?actor");
        Goal goal = new Goal(actor, predicate, observe);
        ActionInterpreter observer = ActionInterpreter.createInterpreterFromContext(goal, observationContext);
        log.debug("Starting observer goal:" + observer.getGoal());
        observer.call();

        // return results if was able to observe
        if (observationContext.isSuccess()) {
          return observationContext.getBoundObservationResults();
        }
      } else {
        log.trace("No observer for '" + predicate + "'");
      }
    }

    if (observe == Observable.DEFAULT || observe == Observable.FALSE) {
      return sm.holds(predicate);
    } else {
      log.warn("No observer found for: " + predicate);
      return new ConditionJustification(false, Factory.createPredicate("observe", predicate.toUnnegatedForm()));
    }
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    for (Predicate p : predicates.keySet()) {
      if (predicates.get(p) == Observable.TRUE) {
        sb.append("[O]");
      } else if (predicates.get(p) == Observable.FALSE) {
        sb.append("[Ø]");
      }
      sb.append(p.toString());
      sb.append("  v  ");
    }
    sb.delete(sb.length() - 3, sb.length()); // Remove last disjunction symbol.
    return sb.toString();
  }

  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Condition)) {
      return false;
    }
    Condition oCondition = (Condition) o;

    // check condition type
    if (this.type != oCondition.type) {
      return false;
    }

    // check condition predicates (order matters)
    Iterator<Map.Entry<Predicate, Observable>> thisItr = this.predicates.entrySet().iterator();
    Iterator<Map.Entry<Predicate, Observable>> otherItr = oCondition.predicates.entrySet().iterator();
    while (thisItr.hasNext() && otherItr.hasNext()) {
      Map.Entry<Predicate, Observable> thisEntry = thisItr.next();
      Map.Entry<Predicate, Observable> otherEntry = otherItr.next();

      //AbstractList does null checks here but for maps we can assume you never get null entries
      if (!thisEntry.equals(otherEntry)) {
        return false;
      }
    }
    if (thisItr.hasNext() || otherItr.hasNext()) {
      return false;
    }

    return true;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + Objects.hashCode(type);

    // iterate through because Objects.hashCode(predicates) doesn't check order
    Iterator<Map.Entry<Predicate, Observable>> thisItr = this.predicates.entrySet().iterator();
    while (thisItr.hasNext()) {
      Map.Entry<Predicate, Observable> thisEntry = thisItr.next();
      result = prime * result + Objects.hashCode(thisEntry);
    }
    return result;
  }

  /**
   * Disjunction constructor.
   */
  public static class Disjunction {
    final private ConditionType type;
    final private LinkedHashMap<Predicate, Observable> predicates;

    public Disjunction(ConditionType t) {
      type = t;
      predicates = new LinkedHashMap<>();
    }

    public Disjunction or(Predicate p, Observable o) {
      predicates.put(p, o);
      return this;
    }

    public Disjunction or(Predicate p) {
      predicates.put(p, Observable.DEFAULT);
      return this;
    }
  }
}
