/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ObservationContext extends ArgumentBasedContext {

  private final static Logger log = LoggerFactory.getLogger(ObservationContext.class);
  /**
   * Raw state to observe (i.e., how it appears in a script).
   * This can contain free-variables that get bound in a script during execution.
   */
  private final Predicate stateToObserve;
  /**
   * This is the bound form of the state to observe. Note that it's still
   * possible to contain unbound free-variables that need to be bound
   * by making the observation.
   */
  private Predicate boundStateToObserve;
  /**
   * If the observed state should be observed, inferred, or default.
   */
  private final Observable observable;
  /**
   * Actor executing the observer.
   */
  private final Symbol actor;
  /**
   * Results from the actual observation.
   * Empty list --> observation is false (i.e., state not observed)
   * Non-empty list --> observation is true (i.e., state was observed)
   * Null --> execution failure (assuming observation has been executed)
   */
  private List<Map<Variable, Symbol>> observationBindings = null;
  /**
   * Results of the observation.
   */
  private Justification observationResults = null;

  protected ObservationContext(Context caller, StateMachine sm, Predicate stateToObserve, Symbol actor, Observable observable) {
    super(caller, sm, "observe");
    this.stateToObserve = stateToObserve;
    this.actor = actor;
    this.observable = observable;
    setupArguments(stateToObserve.getOrderedVars(), null);
  }

  protected ObservationContext(Context caller, StateMachine sm, String cmd, List<String> arguments, Symbol actor, Observable observable) {
    super(caller, sm, "observe");
    this.actor = actor;
    this.stateToObserve = Factory.createPredicate(cmd, arguments.stream().toArray(String[]::new));
    this.observable = observable;

    setupArguments(stateToObserve.getOrderedVars(), null);
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    inputArgs.forEach(arg -> addArgument(arg.toString(), Symbol.class, arg, true, false)); // TODO: these shouldn't be local args

    if (!hasLocalArgument("?actor")) {
      addArgument("?actor", Symbol.class, actor, false, false);
    } else {
      setArgument("?actor", actor);
    }
  }

  @Override
  public void doStep() {
    // if no action has been selected, select one
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        if (childContexts.isEmpty()) {
          selectAction();
        }
        break;
      case OBSERVE:
        if (Database.getInstance().isDiarcAgent((Symbol) getArgumentValue("?actor"))) {
          setExecType(ExecutionType.ACT);
          if (childContexts.isEmpty()) {
            selectAction();
          }
        }
      default:
        break;
    }
  }

  /**
   * Check if this ObservationContext has found a valid Observer (i.e., ActionContext) in the system.
   * This first performs doStep which calls selectionAction, and then does the check.
   * @return
   */
  public boolean hasObserver() {
    doStep();
    if (childContexts.isEmpty() || childContexts.get(0) instanceof FailureContext) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Perform action selection to find an observer action, and add as a child ActionContext. If no observer action
   * can be found, a FailureContext is added as a child.
   */
  private void selectAction() {
    boundStateToObserve = new Predicate(stateToObserve);

    // collect all the unbound variables from the stateToObserve predicate
    if (Utilities.isScriptVariable(stateToObserve.getName()) && stateToObserve.size() == 0) {
      // case 1: entire state is specified by a single variable
      Object value = this.getArgumentValue(stateToObserve.getName());
      Class type = this.getArgumentType(stateToObserve.getName());

      // check that variable is non-null and of type Term or Predicate
      if (value == null || !Term.class.isAssignableFrom(type)) {
        Justification just = new ConditionJustification(false, Factory.createPredicate("valid", stateToObserve));
        Context child = new FailureContext(this, stateMachine, ActionStatus.FAIL_ARGUMENTS, just);
        this.childContexts.add(child);
        return;
      }
      boundStateToObserve = new Predicate((Term) value);
    } else {
      // case 2: bind any unbound arguments to get a bound predicate
      Set<Variable> argumentsToBind = stateToObserve.getVars();

      // build a bound (to the extent possible) stateToObserve. Unbound args are allowable.
      Map<Variable, Symbol> bindings = new HashMap<>();
      for (Variable v : argumentsToBind) {
        ActionBinding binding = getArgument(v.getName());
        if (binding == null) {
          log.warn("[selectAction] argument not declared in action: " + v);
        } else if (binding.isBound()) {
          Object val = binding.getBindingDeep();
          if (val == null) {
            // no binding for this variable
            continue;
          } else if (val instanceof Symbol) {
            bindings.put(v, (Symbol) val);
          } else {
            bindings.put(v, Factory.createSymbol(val.toString()));
          }
        }
      }

      if (!bindings.isEmpty()) {
        boundStateToObserve = stateToObserve.copyWithNewBindings(bindings);
      }
    }

    // get possible observations
//    List<ActionDBEntry> obsDBEntries = Database.getInstance().getObservers(actor, boundStateToObserve);
    Goal goal = new Goal(actor, boundStateToObserve, Observable.TRUE);
    ParameterizedAction obsAction = ActionSelector.getInstance()
                                   .selectActionForGoal(goal, this.constraints, this.stateMachine);

    Context child;
    if (obsAction != null) {
      ActionDBEntry obsDBEntry = obsAction.getEntry();

      // if obs is negated, strip off "not" for action context
      Predicate stateWithoutNegation = boundStateToObserve;
      if (stateWithoutNegation.getName().equalsIgnoreCase("not")) {
        Symbol innerPred = stateWithoutNegation.getArgs().get(0);
        if (!innerPred.isPredicate()) {
          log.error("[bindActionObservationPredicate] inner argument is not a predicate: " + stateWithoutNegation);
        }
        stateWithoutNegation = (Predicate) innerPred;
      }

      child = new ActionContext(this, this.stateMachine, obsDBEntry, Stream.of(stateWithoutNegation).collect(Collectors.toList()), new ArrayList<>(), actor);

      // bind any local free variables in the selected action's advertised observation predicatezn
      bindActionObservationPredicate(child, obsDBEntry.getObservations(), stateWithoutNegation);
    } else {
      log.debug("No observer in DB for: " + boundStateToObserve);
      Predicate p = new Predicate("observe", actor, boundStateToObserve);
      Justification just = new ConditionJustification(false, new Predicate("found", actor, p));
      child = new FailureContext(this, this.stateMachine, ActionStatus.FAIL_NOTFOUND, just);
    }

    childContexts.add(child);
  }

  private void bindActionObservationPredicate(Context action, List<Predicate> observers, Predicate obs) {
    List<Predicate> matchedPredicates = observers.stream().filter(p -> obs.instanceOf(p)).collect(Collectors.toList());
    if (!matchedPredicates.isEmpty()) {
      Predicate advertisedPred = matchedPredicates.get(0);

      Map<Variable, Symbol> advertisedPredBindings = advertisedPred.getBindings(obs);
      for (Variable var : advertisedPred.getVars()) {
        // if advertised var is a local variable of the action, try to bind it
        // FIXME: the following ends up throwing an error because it is trying to bind a variable from the advertised
        //        predicate to the observer. The observer only takes in a condition term, so it doesn't seem as though
        //        it will ever bind?
        ActionBinding localVar = action.getArgument(var.getName());
        if (localVar != null && localVar.isLocal) {
          localVar.bind(advertisedPredBindings.get(var));
        }
      }
    } else {
      log.error("[selectAction] couldn't find matching observation predicate.");
    }
  }

  @Override
  protected void performAdditionalStatusUpdates() {
    stateMachine.update(this);
    if (this.getStatus() == ActionStatus.SUCCESS || this.getStatus().isFailure()){
      setEndTime(System.currentTimeMillis());
    }
    // will this be updated when approved? which isn't the correct time.
    // should it not be more like the method in ActionContext?
    // TODO: should observation contexts update their success probability? -> child contains action to be executed
    //PerformanceMeasures performanceMeasures = Database.getPerformanceMeasures(this.getDBE());
    //Database.getInstance().updateSuccessProbability(cmd, getClonedArguments(), getJustification().getValue(), getDuration());
  }

  @Override
  public List<Predicate> getStateUpdates() {
    List<Predicate> stateUpdates = new ArrayList<>();

    // if the observation wasn't executed, then there is nothing to update
    // without this check, there will be an issue when trying to get observation bindings
    if (!getExecType().shouldExecute()) {
      return stateUpdates;
    }

    // if observation completed successfully (with non-null results) get all state updates based on observation results
    if (isSuccess()) {
      // check for negation of observation predicate
      Predicate stateWithoutNegation;
      Predicate stateWithNegation;
      if (boundStateToObserve.getName().equalsIgnoreCase("not")) {
        stateWithoutNegation = (Predicate) boundStateToObserve.get(0);
        stateWithNegation = boundStateToObserve;
      } else {
        stateWithoutNegation = boundStateToObserve;
        stateWithNegation = Factory.createPredicate("not", boundStateToObserve);
      }

      // NOTE: observationBindings are for non-negated boundStateToObserve (even if boundStateToObserve is negated)
      if (observationBindings == null || observationBindings.isEmpty()) {
        // (non-negated) boundStateToObserve was not observed
        // add negated state
        stateUpdates.add(stateWithNegation);
      } else {
        // (non-negated) boundStateToObserve was observed
        for (Map<Variable, Symbol> bindings : observationBindings) {
          stateUpdates.add(stateWithoutNegation.copyWithNewBindings(bindings));
        }
      }
    }

    return stateUpdates;
  }

  /**
   * If an observation returns null, it fails.
   *
   * @return
   */
  @Override
  public Justification verifyReturnValue() {
    // needed for simulations where action isn't executed
    // otherwise it will fail when trying to get information from observation
    if (!getExecType().shouldExecute()) {
      return new ConditionJustification(true, boundStateToObserve);
    }

    // set logical value based on results of observation (i.e., if boundStateToObserve was observed)
    Justification observationJustification = getBoundObservationResults();
    setLogicalValue(observationJustification.getValue());
    if (observationBindings == null) { // does observation execute successfully or not
      return new ConditionJustification(false, boundStateToObserve);
    } else {
      return new ConditionJustification(true, boundStateToObserve);
    }
  }

  /**
   * Process the observation results and cache results
   */
  private void processObservationBindings() {
    List<Map<Variable, Symbol>> observations = new ArrayList<>();

    // get the return argument from the action executed for this observation
    List<ActionBinding> returns = childContexts.get(0).getArguments().stream().filter(arg -> arg.isReturn).collect(Collectors.toList());
    Object value = returns.get(0).getBindingDeep();

    if (value == null) {
      return;
    }

    if (returns.size() == 1 && List.class.isAssignableFrom(value.getClass())) {
      List<?> list = (List) value;

      if (list.size() > 0) {
        for (Object o : list) {
          if (o instanceof Map) {
            Map<Variable, Symbol> bindings = new HashMap<>();
            Map map = (Map) o;
            if (map.size() > 0) {
              for (Object o2 : map.entrySet()) {
                if (o2 instanceof Map.Entry) {
                  Map.Entry entry = (Map.Entry) o2;
                  Variable var;
                  Symbol val;

                  if (entry.getKey() instanceof String) {
                    var = new Variable((String) entry.getKey());
                  } else if (entry.getKey() instanceof Variable) {
                    var = (Variable) entry.getKey();
                  } else {
                    log.error("Key is not String or edu.tufts.hrilab.fol.Variable!");
                    observationBindings = new ArrayList<>();
                    return; // Could not observe
                  }

                  if (entry.getValue() instanceof String) {
                    val = Factory.createSymbol((String) entry.getValue());
                  } else if (entry.getValue() instanceof Symbol) {
                    val = (Symbol) entry.getValue();
                  } else {
                    log.error("Value is not String or edu.tufts.hrilab.fol.Symbol!");
                    observationBindings = new ArrayList<>();
                    return; // Could not observe
                  }

                  bindings.put(var, val);
                } else {
                  log.error("Map.entrySet() did not return Map.Entry. WTF?");
                  observationBindings = new ArrayList<>();
                  return; // Could not observe
                }
              }
            }
            observations.add(bindings); // Add observation
          } else {
            log.error("Observation action does not return 'List<Map<?>>'");
            observationBindings = new ArrayList<>();
            return; // Could not observe
          }
        }
      }
    } else {
      log.error("Observation action does not return 'List<?>'");
    }
    observationBindings = observations;
    return;
  }

  /**
   * Attempt to retrieve the results of the observation and package them into a Justification.
   *
   * @return Justification of state with associated bindings, when applicable.
   */
  public Justification getBoundObservationResults() {
    if (observationResults != null) {
      return observationResults;
    }

    // if observation is a success, try to get solution bindings
    if (childContexts.get(0).isSuccess()) {

      // check for negation of observation predicate
      boolean isNegation = false;
      if (boundStateToObserve.getName().equalsIgnoreCase("not")) {
        isNegation = true;
      }

      boolean holds = false;
      processObservationBindings();
      if (observationBindings == null) {
        log.error("[getBoundObservationResults] observation solution is null for: " + stateToObserve);
        holds = false;
      } else if (!observationBindings.isEmpty()) {
        if (observationBindings.size() > 1) {
          log.warn("More than one result observed for: " + boundStateToObserve + " Selecting first binding option from: " + observationBindings);
        }
        // deep bind observed arguments
        observationBindings.get(0).forEach((k, v) -> this.getArgument(k.getName()).bindDeep(v));
        if (!isNegation) {
          holds = true;
        }
      } else {
        // if negated and there are no bindings, then the observation is true
        if (isNegation) {
          holds = true;
        }
      }
      observationResults = new ConditionJustification(holds, boundStateToObserve, observationBindings);
    } else if (childContexts.get(0).isFailure()) {
      log.debug("[observeCondition] observation completed with status: " + getStatus());
      observationResults = new ConditionJustification(false, new Predicate("observationFailed", boundStateToObserve));
    } else {
      // set to true, because we don't want observation to fail in this case
      observationResults = new ConditionJustification(true, new Predicate("observationCanceled", boundStateToObserve));
    }

    return observationResults;
  }

  @Override
  protected void resetConcreteContext() {
    this.childContexts.clear();
    this.observationBindings = null;
    this.observationResults = null;
  }

  @Override
  public ObservationContext copy(Context newParent) {
    ObservationContext newObsContext = new ObservationContext(newParent, newParent.stateMachine, this.stateToObserve, (Symbol)getArgumentValue("?actor"), this.observable);
    copyInternal(newObsContext);
    return newObsContext;
  }
}
