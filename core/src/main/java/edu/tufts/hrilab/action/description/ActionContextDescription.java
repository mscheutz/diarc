/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.description;

import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Predicate;

import java.util.ArrayList;
import java.util.List;

import edu.tufts.hrilab.fol.Factory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ActionContextDescription extends ContextDescription {

  private static final Logger log = LoggerFactory.getLogger(ActionContextDescription.class);

  private ActionContext context;

  public ActionContextDescription(ActionContext context) {
    super();
    this.context = context;
  }

  public List<Predicate> getPredicateDescription() {
    List<Predicate> predicates = new ArrayList<>();
    predicates.add(getActionInPredicateForm());
    predicates.add(getStepsInPredicateForm());
    predicates.add(getPreConditionsInPredicateForm());
    predicates.add(getOverallConditionsInPredicateForm());
    predicates.add(getPostConditionsInPredicateForm());
    predicates.add(getJustificationInPredicateForm());
    return predicates;
  }

  /**
   * Get ActionDBEntry that this class is describing.
   *
   * @return
   */
  public ActionDBEntry getAction() {
    return context.getDBE();
  }


  /**
   * Create predicate from the context signature.
   * @return
   */
  public Predicate getActionInPredicateForm() {
    return new Predicate("step", context.getSignatureInPredicateForm());
  }

  /**
   * Get preconditions in predicate form.
   *
   * @return
   */
  public Predicate getPreConditionsInPredicateForm() {
    return new Predicate("preconditions", getConditionsInPredicateForm(context.getPreConditions()));
  }

  /**
   * Get overall conditions in predicate form.
   *
   * @return
   */
  public Predicate getOverallConditionsInPredicateForm() {
    return new Predicate("overallconditions", getConditionsInPredicateForm(context.getOverAllConditions()));
  }

  /**
   * Get post-conditions in predicate form.
   *
   * @return
   */
  public Predicate getPostConditionsInPredicateForm() {
    List<Predicate> post = new ArrayList<>();
    context.getEffects().stream().filter(e -> e.getType().equals(EffectType.SUCCESS) || e.getType().equals(EffectType.ALWAYS))
            .forEach(e -> post.add(e.getPredicate()));
    return new Predicate("postconditions", post);
  }

  /**
   * Get action execution Justification(s) in predicate form.
   *
   * @return
   */
  public Predicate getJustificationInPredicateForm() {
    return new Predicate("justification", context.getJustification().getPredicates());
  }

  /**
   * Helper method to convert Conditions to Predicates.
   *
   * @param conditions
   * @return
   */
  private List<Predicate> getConditionsInPredicateForm(List<Condition> conditions) {
    List<Predicate> predicateConditions = new ArrayList<>();
    for (Condition condition : conditions) {
      if (condition.isDisjunction()) {
        predicateConditions.add(new Predicate("or", new ArrayList<>(condition.getPredicates().keySet())));
      } else {
        predicateConditions.addAll(condition.getPredicates().keySet());
      }
    }
    log.debug("boundPreconditions are: " + predicateConditions);
    if (predicateConditions.isEmpty()) {
      predicateConditions.add(Factory.createPredicate("none()"));
    }
    return predicateConditions;
  }

  /**
   * Get the top-level steps (i.e., the EventSpecs of the selected actionDBEntry)
   * of this action selection in predicate form. This is currently used by NLG to generate
   * action script narrations (via a getActDesc action script).
   * <p>
   * It's possible that this method shouldn't live here, but should
   * perhaps be moved to a new class for script narration. This is an early-stage
   * implementation of this idea, so it's unclear how this should be architected.
   *
   * @return
   */
  public Predicate getStepsInPredicateForm() {
    List<Predicate> predicateSteps = new ArrayList<>();

    // iterate through actionDBEntry's eventSpecs
    for (EventSpec eventSpec : context.getDBE().getEventSpecs()) {
      List<String> eventSpecArgs = new ArrayList<>();

      if (eventSpec.getType() != EventSpec.EventType.ACTION) {
        //log.warn("[getStepsInPredicateForm] only returning ACTSPEC steps.");
        continue;
      }

      // get actor binding
      String actor = eventSpec.getActor();
      eventSpecArgs.add(context.getArgumentValue(actor).toString());

      // get arg bindings
      for (String arg : eventSpec.getInputArgs()) {
        if (Utilities.isScriptVariable(arg)) {
          Object value = context.getArgumentValue(arg);
          if (value != null) {
            eventSpecArgs.add(value.toString());
          } else {
            eventSpecArgs.add("null");
          }
        } else {
          eventSpecArgs.add(arg);
        }
      }

      Predicate eventPred = new Predicate(eventSpec.getCommand(), eventSpecArgs.toArray(new String[0]));

      // add step to steps list
      predicateSteps.add(eventPred);
    }

    return new Predicate("steps", predicateSteps);
  }

  public Predicate getSuccessWithReason() {
//      return context.getJustification();
        switch (context.getStatus()) {
      case SUCCESS:
        return (Factory.createPredicate("response(true,none)"));
      case FAIL:
      case FAIL_NOTFOUND:
      case FAIL_SYNTAX:
      case FAIL_CHILD:
      case FAIL_ANCESTOR:
      case FAIL_RECOVERY:
      case FAIL_ARGUMENTS:
      case FAIL_FORBIDDEN:
      case FAIL_CONSTRAINTS:
      case FAIL_OBLIGATIONS:
      case FAIL_RETURNVALUE:
      case FAIL_POSTCONDITIONS:
      case FAIL_PRECONDITIONS:
      case FAIL_ARGUMENTUNBOUND:
      case FAIL_OVERALLCONDITIONS:
        //todo: and all failure reasons
        Predicate justification = context.getJustification().getFailureReason().get(0);
        return Factory.createPredicate("response(false,"+justification+")");
      case SUSPEND:
        log.warn("Suspension response not yet supported");
      case CANCEL:
        log.warn("Cancel response not yet supported");
      default:
        return null;
    }
  }

}
