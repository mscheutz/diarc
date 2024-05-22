/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.lock.ResourceUnavailableException;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.performanceMeasures.PerformanceMeasures;
import edu.tufts.hrilab.action.description.ActionContextDescription;
import edu.tufts.hrilab.action.description.ContextDescription;
import edu.tufts.hrilab.action.translation.TranslationInfo;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;

import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.AndJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.lock.ActionResourceLock;

import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class ActionContext extends DatabaseEntryContext<ActionDBEntry> {

  private double posAff = 0.0;
  private double negAff = 0.0;

  private ScriptParser parser;

  /**
   * Map of observations for Effects. These need to be kept track of separately from children for normal execution.
   */
  private Map<Effect, ObservationContext> effectObservers = new HashMap<>();
  /**
   * Map of observations for Conditions. These need to be kept track of separately from children for normal execution.
   */
  private Map<Condition, Map<Predicate, ObservationContext>> conditionObservers = new HashMap<>();

  private List<ActionResourceLock> heldLocks;
  private String lockOwnerDescription;

  /**
   * Thread manager for concurrent overall conditions.
   */
  private final ExecutorService overallConditionExecutor = Executors.newSingleThreadExecutor();
  /**
   * Future for all overall conditions.
   */
  private Future overallConditionFuture = null;

  /**
   * @param caller
   * @param sm
   * @param action
   * @param inputArgs
   * @param returnArgs
   * @param actor
   */
  public ActionContext(Context caller, StateMachine sm, ActionDBEntry action, List<? extends Object> inputArgs, List<? extends Object> returnArgs, Symbol actor) {
    super(caller, sm, action);
    List<Object> argsWithActor = new ArrayList<>();
    argsWithActor.add(actor);
    argsWithActor.addAll(inputArgs);
    setupArguments(argsWithActor, returnArgs);
    initialize();
  }

  /**
   * @param caller
   * @param sm
   * @param action
   * @param bindings
   * @param executionType
   * @param actor
   */
  public ActionContext(Context caller, StateMachine sm, ActionDBEntry action, Map<String, Object> bindings, ExecutionType executionType, Symbol actor) {
    super(caller, sm, action, executionType);
    if (!bindings.containsKey("?actor")) {
      bindings.put("?actor", actor);
    }
    setupArguments(bindings);
    initialize();
  }

  private void initialize() {
    parser = new ScriptParser(getDBE().getEventSpecs());
    setTimeout();
  }

  /**
   * Acquire all locks registered for this action
   *
   * @param actionInt The AI that needs the lock
   * @param greedy    if not greedy, then release the locks if all can't be
   *                  acquired
   * @return true is all locks acquired, false otherwise
   */
  @Override
  public boolean acquireLocks(ActionInterpreter actionInt, boolean greedy) {
    try {
      heldLocks = getDBE().acquireResourceLocks(actionInt, greedy);
    } catch (ResourceUnavailableException e) {
      lockOwnerDescription = e.getMessage();
      return false;
    }

    return true;
  }

  /**
   * Release all locks registered for this action
   *
   * @param actionInt The AI that needs the lock
   */
  @Override
  public void releaseLocks(ActionInterpreter actionInt) {
    if (heldLocks == null) {
      // FIXME heldLocks is always null because we never call acquireLocks on any action
      log.debug("heldLocks is null. This should not be the case. Initializing and returning for now.");
      heldLocks = new ArrayList<>();
      return;
    }
    for (ActionResourceLock lock : heldLocks) {
      lock.release(actionInt);
    }
    heldLocks.clear();
  }

  public double getCost() {
    return getDBE().getCost();
  }

  public double getBenefit() {
    return getDBE().getBenefit();
  }

  public double getMinUrgency() {
    return getDBE().getMinUrg();
  }

  public double getMaxUrgency() {
    return getDBE().getMaxUrg();
  }

  /**
   * Get state updates for this context based on the current status (i.e., ActionStatus). This excludes any effects
   * that were verified via an observation, as observations are actions that automatically update Belief state.
   *
   * @return
   */
  @Override
  public List<Predicate> getStateUpdates() {
    List<Predicate> stateUpdates = new ArrayList<>();
    // filtering for non-observed effects because observers are actions, so the observed state is already updated in Belief
    if (getExecType() == ExecutionType.SIMULATE_PERFORMANCE) {
      // assert default post conditions if action successful (needs to check approved because just starts true
      if (!getStatus().equals(ActionStatus.APPROVED)) {
        getEffects(getStatus()).stream().filter(Effect::isAutoGenerated).forEach(e -> stateUpdates.add(e.getPredicate()));
      }
    } else {
      // get all non-observable effects for current status including those autogenerated
      getEffects(getStatus()).stream()
              .filter(effect -> !effectObservers.containsKey(effect))
              .forEach(effect -> stateUpdates.add(effect.getPredicate()));

      // TODO: this is not ideal. a better way to handle this could be to re-enable the use of the FAIL_CHILD
      //  context, instead of propagating the same failure status up the entire context tree
      // if action fails because of some failed post-condition check (must have been observed, not inferred post-condition),
      // we don't want all post-conditions to automatically be false, so include inferred post-conditions in state update
      // ignore autogenerated effects - now are now success and failure autogenerated effects
      if (getStatus() == ActionStatus.FAIL_POSTCONDITIONS && causedFailure()) {
        List<Predicate> tmpStates = new ArrayList<>();
        getEffects(ActionStatus.SUCCESS).stream()
                .filter(effect -> effect.getObservable() != Observable.TRUE && !effectObservers.containsKey(effect) && !effect.isAutoGenerated())
                .forEach(effect -> tmpStates.add(effect.getPredicate()));
        stateUpdates.addAll(tmpStates);
      }
    }
    return stateUpdates;
  }

  /**
   * Get the list of effects (filtered by ActionStatus) from the ActionDBEntry, bind variables, and return it all.
   *
   * @param status
   * @return
   */
  public List<Effect> getEffects(ActionStatus status) {
    List<Effect> effects = new ArrayList<>();
    getDBE().getEffects(status).forEach(e -> effects.add(e.bindToContext(this)));
    return effects;
  }

  /**
   * Get the list of effects from the ActionDBEntry, bind variables, and return it all.
   *
   * @return
   */
  public List<Effect> getEffects() {
    List<Effect> effects = new ArrayList<>();
    getDBE().getEffects().forEach(e -> effects.add(e.bindToContext(this)));
    return effects;
  }

  /**
   * Get the list of preconditions from the ActionDBEntry, bind variables, and return it all.
   *
   * @return List of conditions that have been bound using this context
   */
  public List<Condition> getPreConditions() {
    List<Condition> conditions = new ArrayList<>();
    getDBE().getPreConditions().forEach(c -> conditions.add(c.bindToContext(this)));
    return conditions;
  }

  /**
   * Get the list of overall conditions from the ActionDBEntry, bind variables and return it all.
   *
   * @return List of conditions that have been bound using this context
   */
  public List<Condition> getOverAllConditions() {
    List<Condition> conditions = new ArrayList<>();
    getDBE().getOverallConditions().forEach(c -> conditions.add(c.bindToContext(this)));
    return conditions;
  }

  /**
   * Get the list of obligation conditions from the ActionDBEntry, bind variables and return it all.
   *
   * @return List of conditions that have been bound using this context
   */
  public List<Condition> getObligationConditions() {
    List<Condition> conditions = new ArrayList<>();
    getDBE().getObligationConditions().forEach(c -> conditions.add(c.bindToContext(this)));
    return conditions;
  }

  // move into StepExecution
  @Override
  // TODO: check locks
  public Justification isApproved() {
    // adjust exec type based on who is acting (i.e., diarc agent or not)
    // setting exec type here allows for dynamic actors in action scripts and will ensure pre-conditions
    // are not observed for non-diarc agents
    setExecType();

    AndJustification totalCheck = new AndJustification();
    // Check constraints (forbidden action or results in a forbidden state)
    Justification cstrCheck = constraints.verifyConstraints(getDBE(), getEffects(), stateMachine);
    cstrCheck.setStep(this.getSignatureInPredicateForm());
    if (!cstrCheck.getValue()) {
      log.warn("Action " + cmd + " violates constraints: " + cstrCheck.getFailureReason());
      setStatus(ActionStatus.FAIL_CONSTRAINTS, cstrCheck);
      return cstrCheck;
    }
    totalCheck.addJustification(cstrCheck);

    // Check pre-conditions
    AndJustification pcCheck = new AndJustification();
    for (Condition condition : this.getPreConditions()) {
      Justification j = condition.holds(this, stateMachine);  // Verify condition
      pcCheck.addJustification(j);      // Conjunction
    }
    pcCheck.setStep(this.getSignatureInPredicateForm());
    if (!pcCheck.getValue()) {
      if (!cmd.startsWith("mcts")) {
        log.warn("Preconditions for action " + cmd + " are not met. Failure reason: " + pcCheck.getFailureReason());
      }
      setStatus(ActionStatus.FAIL_PRECONDITIONS, pcCheck);
      return pcCheck;
    }

    // check obligation conditions
    for (Condition condition : this.getObligationConditions()) {
      Justification j = condition.holds(this, stateMachine);  // Verify condition
      pcCheck.addJustification(j);      // Conjunction
    }
    pcCheck.setStep(this.getSignatureInPredicateForm());
    if (!pcCheck.getValue()) {
      log.warn("Obligations for action " + cmd + " are not met: " + pcCheck.getFailureReason());
      setStatus(ActionStatus.FAIL_OBLIGATIONS, pcCheck);
      return pcCheck;
    }

    totalCheck.addJustification(pcCheck);

    // Bind free variables using belief lookup (via condition checking!)
    for (String argName : arguments.keySet()) {
      ActionBinding arg = this.getEvaluatedArgument(argName);
      if ((arg.isLocal || arg.isReturn()) && arg.getBindingDeep() == null) {
        // get the potential bindings from the justification
        //TODO:brad: is there already a symbol in here somewhere
        Set<Symbol> potentialBindings = pcCheck.getBindings(new Variable(argName));
        if (potentialBindings != null) { // check to see if potential bindings is empty
          if (potentialBindings.isEmpty()) {
            // no available options -- this can happen in the case where a pre-condition is negated
            // e.g., not(repairing(?actor,!tube)). In this case no binding for !tube would be found if the
            // pre-condition passes (e.g., ?actor is not repairing any tubes)
            log.debug("Argument " + argName + " has no options");
          } else {
            if (potentialBindings.size() > 1) {
              log.warn("More than one variable binding found. Choosing one at random for arg: " + argName);
            }
            Symbol binding = potentialBindings.iterator().next();
            setArgument(argName, binding);
          }
        }
      }
    }

    //todo: Check norms
    try {
        Justification normJustification = TRADE.getAvailableService(new TRADEServiceConstraints().name("getViolations")).call(Justification.class,getArgumentValue("?actor"), stateMachine);
        if (!normJustification.getValue()) {
          setStatus(ActionStatus.FAIL_NORMS, normJustification);
        }
    } catch (TRADEException e) {
      //TODO:brad: this is debug for now, but that relies the exception is thrown in get available, and not call. Ideally they would throw different types of exceptions.
      log.debug("[isApproved][getViolations]",e);
    }

    // Return justifications
    return totalCheck;
  }

  /**
   * Continuously checks that the overall conditions are met. Calls the Context exit() method on failure.
   */
  @Override
  protected void startOverAllMonitor() {
    List<Condition> overallConditions = getOverAllConditions();
    if (!overallConditions.isEmpty()) {
      ActionContext context = this;
      overallConditionFuture = overallConditionExecutor.submit(() -> {
        try {
          while (getStatus() == ActionStatus.PROGRESS) {
            // Check conditions
            AndJustification check = new AndJustification();
            for (Condition condition : overallConditions) {
              Justification j = condition.holds(context, stateMachine);   // Verify condition
              check.addJustification(j);              // Conjunction
            }

            check.setStep(this.getSignatureInPredicateForm());
            //check.setContextRep(this.getContextDescription());
            if (!check.getValue()) {
              log.warn("OverAllConditions failed: " + check.getFailureReason());
              setStatus(ActionStatus.FAIL_OVERALLCONDITIONS, check); // Stop execution of this context with the appropriate status
            }
            Thread.sleep(250);
          }
        } catch (InterruptedException e) {
          log.error("Error in startOverAllMonitor. ", e);
        }
      });
    }
  }

  @Override
  protected void performAdditionalStatusUpdates() {
    stateMachine.update(this);
  }

  @Override
  protected void setTimeout() {
    // get timeouts right for autonomy
    long remainingTime;
    if (this.getDBE().hasTimeout()) {
      this.startTime = System.currentTimeMillis();

      // Limit the timeout, if necessary
      if (caller instanceof ActionContext) {
        ActionContext callerAction = (ActionContext) caller;
        if (callerAction.getDBE() != null && callerAction.getDBE().hasTimeout()) {
          remainingTime = callerAction.maxTime - (this.startTime - callerAction.startTime);
          if (this.maxTime > remainingTime) {
            this.maxTime = remainingTime;
          }
        } else {
          this.maxTime = this.getDBE().getTimeout();
        }
      }

    } else {
      this.maxTime = caller.maxTime;
      this.startTime = caller.startTime;
    }
  }

  @Override
  public boolean isAction() {
    return true;
  }

  @Override
  public ContextDescription getContextDescription(Predicate location) {
    // get location (e.g., current, next, last, etc) and action (optional action step) info from location predicate
    Symbol loc = location.get(0);
    Symbol act = null;
    if (location.size() > 1) {
      act = location.get(1);
    }

    ContextDescription contextDescription = null;
    if (act != null) {
      // TODO: generalize instanceOf method -- make it a Symbol base class method
      boolean matchesSignature = false;

      // check if this context matches the action part of the location predicate
      if (act.isTerm()) {
        if (((Term) act).instanceOf(this.getSignatureInPredicateForm())) {
          matchesSignature = true;
        }
      } else {
        if (act.getName().equals(this.getSignatureInPredicateForm().getName())) {
          matchesSignature = true;
        }
      }

      // if this context matches the action, then get its context description based on the loc (e.g., current,previous,next,etc)
      if (matchesSignature) {
        contextDescription = this.getContextDescription(new Predicate(location.getName(), loc));
      } else {
        // else keep searching down the context tree
        Context currentChild = childContexts.getCurrent();
        if (currentChild != null) {
          contextDescription = currentChild.getContextDescription(location);
        }
      }
    } else {
      // no action defined in location pred, assume it's referencing top-level goal/action (i.e., this context)
      Context targetChild = null;
      switch (loc.getName()) {
        case "current":
          targetChild = childContexts.getCurrent();
          break;
        case "previous":
        case "last":
          targetChild = childContexts.getPrevious();
          break;
        case "next":
          targetChild = childContexts.getNext();
          break;
        default:
          log.warn("[getContextDescription] invalid location specified: " + location);
      }

      if (targetChild != null) {
        contextDescription = targetChild.getContextDescription();
      }
    }

    if (contextDescription == null) {
      log.warn("[getContextDescription(location)] no context description available." +
              "This: " + this +
              "location: " + location +
              "numChildren: " + childContexts.size());
    }

    return contextDescription;
  }

  @Override
  public ContextDescription getContextDescription() {
    ActionContextDescription actionDescription = new ActionContextDescription(this);
    return actionDescription;
  }


  /**
   * Executes the action corresponding to this context. If the action is a
   * script, then this step initializes each of the steps in the script. If the
   * action is a primitive, then it executes that action.
   */
  @Override
  public void doStep() {
    if (isScript()) {
      // nothing to be done if this is a script
      // children will be built in calls to setupNextStep
    } else if (getExecType().shouldExecute()) {    // Check that action is not simulated
      log.debug(getCommand() + ": executing action");
      Collection<ActionBinding> args = collectArguments();
      Justification executionJustification = getDBE().executeAction(args);
      if (executionJustification.getValue()) {
        redistributeArguments();
      } else {
        // Sets status to fail
        setStatus(ActionStatus.FAIL, executionJustification);
      }
    } else {
      // do not execute simulations,
      // tmf: how to properly set return value during simulation
      redistributeArguments();
    }
  }

  /**
   * Check if action is script.
   *
   * @return true if action is child of script
   */
  public boolean isScript() {
    return getDBE().isScript();
  }

  @Override
  protected void setupNextStep() {
    if (parser != null && !parser.isEmpty()) {
      setupNextStep(parser);
      parser = parser.getRest();
    }
  }

  /**
   * Applies the effects of this context according to the execution state. Observes them if necessary.
   *
   * @return returns justification for verified effects. always true if no effects are observed.
   */
  @Override
  protected Justification verifyEffects() {
    // Observe effects
    log.debug("[verifyEffects] for: " + getSignatureInPredicateForm() + ". effects: " + this.getEffects());
    AndJustification observations = new AndJustification();
    PerformanceMeasures performanceMeasures;
    Justification effectJustification;

    long start = System.currentTimeMillis();
    // check effects in parallel
    // NOTE: it seems running in parallel is actually slower in most cases, likely due to
    // synchronization issues in other component (belief, etc), but more profiling needs to be done to be sure
    // what the bottleneck is.
//    List<Justification> justifications = Collections.synchronizedList(new ArrayList<>());
//    getEffects().parallelStream().forEach(effect -> justifications.add(effect.verify(this)));
//    justifications.forEach(j -> observations.addJustification(j));

    switch (getExecType()) {
      // FIXME: the following assumes the effects aren't side effects
      case ACT:  // Observe effects
        List<Effect> boundEffects = getEffects(ActionStatus.SUCCESS);
        HashMap<Predicate, Boolean> unboundEffectsMap = new HashMap<>();
        List<Predicate> unboundEffects = getDBE().getEffects().stream().map(Effect::getPredicate).collect(Collectors.toList());

        for (int i = 0; i < boundEffects.size(); ++i) {
          effectJustification = boundEffects.get(i).verify(this);
          observations.addJustification(effectJustification);  // Verify effect
          unboundEffectsMap.put(unboundEffects.get(i), effectJustification.getValue());
        }
        setEndTime(System.currentTimeMillis());
        // update effect models
        updatePerformanceModels(observations, unboundEffectsMap);
        break;
      case OBSERVE:
        boundEffects = getEffects(ActionStatus.SUCCESS);
        unboundEffectsMap = new HashMap<>();
        unboundEffects = getDBE().getEffects().stream().map(Effect::getPredicate).collect(Collectors.toList());
        // TMF: set the duration to observe the effects of the action
        // this duration could be set by sampling action performance models of this action
        // could factor in how long it takes to make each observation
        //TODO:brad: shouldn't you just call the observer and wait for it to return?
        int observationDuration = 300000; // 5 min (arbitrary time)
        long endTime = System.currentTimeMillis() + observationDuration;
        List<Effect> confirmedBoundEffects = new ArrayList<>();
        // observe each effect. if it holds then add it to observations
        // otherwise, when "observed long enough" add the remaining effects to the observations as not holding
        while (endTime > System.currentTimeMillis() && !boundEffects.isEmpty()) {
          for (int i = 0; i < boundEffects.size(); ++i) {
            effectJustification = boundEffects.get(i).verify(this);
            if (effectJustification.getValue()) {
              confirmedBoundEffects.add(boundEffects.get(i));
              observations.addJustification(effectJustification);  // Verify effect
              unboundEffectsMap.put(unboundEffects.get(i), effectJustification.getValue());
            }
          }
          boundEffects.removeAll(confirmedBoundEffects);
        }
        // if there are remaining boundEffects then they were not observed and must be false
        if (!boundEffects.isEmpty()) {
          for (int i = 0; i < boundEffects.size(); i++) {
            observations.addJustification(new ConditionJustification(false, boundEffects.get(i).getPredicate()));
            unboundEffectsMap.put(unboundEffects.get(i), false);
          }
        }

        setEndTime(System.currentTimeMillis());
        // update effect models
        updatePerformanceModels(observations, unboundEffectsMap);
        break;

      case SIMULATE_PERFORMANCE:  // sample each effect and set end time
        // get effect probabilities from database
        performanceMeasures = Database.getPerformanceMeasuresDB().getPerformanceMeasures(getDBE());
        List<ActionBinding> bindings = getArguments();
        boolean isSuccessful;
        Map<Predicate, Boolean> sampledEffects = new HashMap<>();
        if (childContexts.isEmpty()) { // primitive action
          if (getEffects(ActionStatus.SUCCESS).stream().anyMatch(eff -> !eff.isAutoGenerated())) {
            // contains success effects
            sampledEffects = performanceMeasures.sampleEffects(bindings);
            isSuccessful = successEffectsHold(sampledEffects);
          } else { // no non-autogenerated success effects
            isSuccessful = performanceMeasures.sampleSuccess(bindings);
          }
        } else { // action script
          // if all the children are successful, then this action is probably successful
          isSuccessful = getActualLogicalValue() && !getStatus().isFailure();
          Map<Predicate, Boolean> tmpSampledEffects = performanceMeasures.sampleEffects(bindings);
          for (Map.Entry<Predicate, Boolean> sampledEffect : tmpSampledEffects.entrySet()) {
            sampledEffects.put(sampledEffect.getKey(), isSuccessful);
          }
          // TODO: how to handle action script effects so that not duplicating sampling probability
          //       prob holding when executing pickup is 13/15, but this is because prob of graspObject is 14/15.
          //       so the effects for pickup (assuming all action steps succeed) would be 13/14
          //       but overall prob for holding is 13/15
          //       Need to maintain joint distribution for each action step (only when failure occurs at action step)
          //       as soon as failure occurs, need to update the prob each execution or possibly just keep track of failures
          //       at the location and then use the overall number of executions as the other info?
          //if (isSuccessful) { // if children all successful, then sample normally
          //  sampledEffects = performanceMeasures.sampleEffects(bindings);
          //  isSuccessful = successEffectsHold(sampledEffects);
          //} else { // child failure -- sample effects with the knowledge that the post-conditions won't hold
          //  // tmf: is there a situation where a child fails, but the post conditions of the parent hold?
          //  sampledEffects = performanceMeasures.sampleEffects(bindings, new HashMap<>(), false);
          //}
        }

        for (Map.Entry<Predicate, Boolean> eff : sampledEffects.entrySet()) {
          //sampledEffects.put(eff.getKey(),isSuccessful);
          observations.addJustification(new ConditionJustification(eff.getValue(), eff.getKey()));
        }
        if (!isSuccessful) {
          if (sampledEffects.isEmpty()) {
            Justification j = new ConditionJustification(isSuccessful, getSignatureInPredicateForm());
            j.setStep(getSignatureInPredicateForm());
            observations.addJustification(j);
            if (!isFailure()) { // if child hasn't failed
              setStatus(ActionStatus.FAIL_RETURNVALUE, observations);
            }
          } else {
            // tmf: should post-condition info of this action be added to justification if child failed?
            if (!isFailure()) { // if child hasn't failed
              setStatus(ActionStatus.FAIL_POSTCONDITIONS, observations);
            }
          }
          setLogicalValue(isSuccessful);
        }
        // FIXME: the following updates the state prior to adding 2nd instance of this context in the state machine
        stateMachine.updateState(sampledEffects);
        List<Context> children = childContexts.getChildrenContexts();
        double sampledTime = 0;
        if (children.isEmpty()) {
          sampledTime = performanceMeasures.sampleTime(getArguments(), observations.getValue());
          sampledTime *= Math.pow(10, 3);
          log.debug(getSignatureInPredicateForm() + " sampledTime: " + sampledTime);
        } else {
          for (Context child : children) {
            sampledTime += child.getDuration();
          }
        }
        setEndTime(getStartTime() + (long) sampledTime);
      default:
        break;
    }
    log.debug("Verifying effects for (" + this.getSignatureInPredicateForm() + ") took (sec):" + (System.currentTimeMillis() - start) / 1000.0);

    observations.setStep(this.getSignatureInPredicateForm());
    return observations;
  }

  /**
   * if the action was actually executed, then update the performance models based on sub-actions and effects
   *
   * @param justification is the action success or failure based on effect observations
   * @param effectResults map between effect and if it holds
   */
  private void updatePerformanceModels(Justification justification, Map<Predicate, Boolean> effectResults) {
    if (getExecType() == ExecutionType.ACT) {
      if (endTime == 0) {
        setEndTime(System.currentTimeMillis());
      }
      PerformanceMeasures performanceMeasures = Database.getPerformanceMeasuresDB().getPerformanceMeasures(getDBE());
      if (performanceMeasures == null) {
        log.debug("No performance measures for action " + getDBE().getName());
        return;
      }

      log.debug("Updating performance models for: " + getSignatureInPredicateForm());
      // if no non-Autogenerated effects -> get logical value and status
      if (getDBE().getEffects(ActionStatus.SUCCESS).stream().noneMatch(e -> !e.isAutoGenerated())) {
        boolean succeeded = getActualLogicalValue() && !isFailure();
        performanceMeasures.updatePerformanceModels(succeeded, getDuration(), effectResults, getArguments());
      } else {
        // success of action is based only on if effect holds
        performanceMeasures.updatePerformanceModels(justification.getValue(), getDuration(), effectResults, getArguments());
      }
      log.debug("finished updating performance models for: " + getSignatureInPredicateForm());
    }
  }

  /**
   * Determine if the sampled effects entail the success effects
   *
   * @param sampledEffects the sampled effects during performance assessment
   * @return boolean if sampled effects entail success effects
   */
  private boolean successEffectsHold(Map<Predicate, Boolean> sampledEffects) {
    List<Effect> successEffects = getEffects(ActionStatus.SUCCESS);
    boolean isSuccessful = true;
    for (Effect successEffect : successEffects) {
      if (successEffect.isAutoGenerated()) {
        continue;
      }
      if (!isSuccessful) {
        break;
      }
      isSuccessful = false;
      for (Predicate sampledEffect : sampledEffects.keySet()) {
        if (successEffect.getPredicate().instanceOf(sampledEffect)) {
          if (sampledEffects.get(sampledEffect)) {
            isSuccessful = true;
          }
          break;
        }
      }
    }
    return isSuccessful;
  }

  @Override
  protected void updatePerformanceModels() {
    updatePerformanceModels(getJustification(), new HashMap<>());
  }

  @Override
  public Justification verifyReturnValue() {
    // if action is a primitive with a Justification return, check that it has returned true
    if (!isScript()) {
      List<ActionBinding> returnArgs = collectArguments().stream().filter(a -> a.isReturn()).collect(Collectors.toList());
      if (returnArgs.isEmpty()) {
        // fine for primitive actions to have no return value
        return new ConditionJustification(true);
      } else if (returnArgs.size() > 1) {
        // This shouldn't be possible
        log.error("[verifyReturnValue] primitive action has more than one return value. Action: " + cmd);
        Justification justification = new ConditionJustification(false);
        justification.setStep(this.getSignatureInPredicateForm());
        return justification;
      }

      // have exactly one return value
      ActionBinding returnArg = returnArgs.get(0);
      if (Justification.class.isAssignableFrom(returnArg.getJavaType()) && !((Justification) returnArg.getBindingDeep()).getValue()) {
        // Justification return type and return value is false
        Justification justification = (Justification) returnArg.getBindingDeep();
        justification.setStep(this.getSignatureInPredicateForm());
        return justification;
      } else {
        return new ConditionJustification(true);
      }
    }

    // a script -- currently no restrictions on script return values
    return new ConditionJustification(true);
  }

  @Override
  protected void resetConcreteContext() {
    // make sure all overall conditions have finished before resetting
    if (overallConditionFuture != null) {
      try {
        overallConditionFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("Exception waiting for overall conditions to finish.", e);
      }
    }

    // reset all local arguments
    log.debug("resetting local arguments for context: " + this.cmd);
    List<ActionBinding> localRoles = this.getDBE().getRoles().stream().filter(role -> role.isLocal).collect(Collectors.toList());
    localRoles.forEach(role -> setArgument(role.name, role.defaultValue));
  }

  /**
   * Look up cached ObservationContext or attempt to create a new one for an Condition. Returns null
   * if an appropriate Observer could not be found in the system.
   *
   * @param condition Condition (can contain multiple predicates in the case of a disjunction)
   * @param predicate the particular predicate of the Condition
   * @return
   */
  public ObservationContext getConditionObserver(Condition condition, Predicate predicate) {
    ObservationContext observationContext;

    Map<Predicate, ObservationContext> observerMap = conditionObservers.get(condition);
    if (observerMap == null) {
      observerMap = new HashMap<>();
      conditionObservers.put(condition, observerMap);
    }

    // look up or create new observation context
    observationContext = observerMap.get(predicate);
    if (observationContext == null) {
      // try to create new observation context
      Symbol actor = (Symbol) getArgumentValue("?actor");
      observationContext = new ObservationContext(this, this.stateMachine, predicate, actor, Observable.TRUE);
      if (observationContext.hasObserver()) {
        // observer found
        observerMap.put(predicate, observationContext);
      } else {
        // no observer found during action selection
        observationContext = null;
      }
    } else {
      observationContext.resetContext();
    }
    return observationContext;
  }

  /**
   * Look up cached ObservationContext or attempt to create a new one for an Effect. Returns null
   * if an appropriate Observer could not be found in the system.
   *
   * @param effect
   * @return
   */
  public ObservationContext getEffectObserver(Effect effect) {
    ObservationContext observationContext;

    observationContext = effectObservers.get(effect);
    if (observationContext == null) {
      Symbol actor = (Symbol) getArgumentValue("?actor");
      if (!Database.getInstance().isDiarcAgent(actor)) {
        Context parentContext = getParentContext();
        while (parentContext != null) {
          parentContext = getParentContext();
          actor = (Symbol) parentContext.getArgumentValue("?actor");
          if (Database.getInstance().isDiarcAgent(actor)) {
            break;
          }
        }
      }
      observationContext = new ObservationContext(this, this.stateMachine, effect.getPredicate(), actor, Observable.TRUE);
      if (observationContext.hasObserver()) {
        // observer found
        effectObservers.put(effect, observationContext);
      } else {
        // no observer found during action selection
        observationContext = null;
      }
    } else {
      observationContext.resetContext();
    }
    return observationContext;
  }

  // **************************************************************************
  // Affect State stuff

  /**
   * Update affect state
   *
   * @param inc      the increment value for the update
   * @param positive boolean indicating whether to add positive affect
   */
  protected void updateAffect(double inc, boolean positive) {
    if (positive) {
      posAff += (1 - posAff) * inc;
      //log.debug("Action " + Type + " posAff: " + posAff);
    } else {
      negAff += (1 - negAff) * inc;
      //log.debug("Action " + Type + " negAff: " + negAff);
    }
  }

  /**
   * Get affect state
   *
   * @param positive boolean indicating whether to add positive affect
   * @return the requested affect state value
   */
  protected double getAffect(boolean positive) {
    if (positive) {
      return posAff;
    }
    return negAff;
  }

  /**
   * Get affective evaluation of action
   *
   * @return the evaluation
   */
  protected double affectEval() {
    //double a = 0.5 + (posAff*posAff - negAff*negAff) * 0.5;
    double a = 1 + posAff * posAff - negAff * negAff;

    return a;
  }

  protected void setExecType() {
    if (!Database.getInstance().isDiarcAgent((Symbol) getArgumentValue("?actor"))) {
      // tmf: for performance assessment, should it still convert execution type to observe for non-diarc agents?
      //      or should it keep as perf simulation and try to sample performance measures for that agent
      setExecType(ExecutionType.OBSERVE);
    } else if (getExecType() == ExecutionType.OBSERVE) {
      setExecType(ExecutionType.ACT);
    }
  }

  private ActionContext(Context caller, StateMachine sm, ActionDBEntry action, LinkedHashMap<String, ActionBinding> arguments) {
    super(caller, sm, action);
    copyArguments(arguments);
    initialize();
  }

  @Override
  public ActionContext copy(Context newParent) {
    ActionContext newActContext = new ActionContext(newParent, newParent.stateMachine, getDBE(), this.arguments);

    copyInternal(newActContext);
    newActContext.parser = this.parser;
    newActContext.posAff = this.posAff;
    newActContext.negAff = this.negAff;
    return newActContext;
  }

  @Override
  protected void setStartTime() {
    switch (getExecType()) {
      case SIMULATE_PERFORMANCE:  // sample each effect
        if (!getParentContext().getCommand().contains("root")) {
          Context previousChild = getParentContext().childContexts.getPrevious();
          if (previousChild != null) {
            setStartTime(previousChild.getEndTime());
          } else {
            setStartTime(getParentContext().getStartTime());
          }
        } else {
          setStartTime(System.currentTimeMillis());
        }
        break;
      case ACT:  // Observe effects
      default:
        setStartTime(System.currentTimeMillis());
        break;
    }
  }

  public synchronized void handleInterrupt(ActionStatus eStatus) {

    // Collect correct onInterrupt or onResume service details if relevant, otherwise return immediately
    EventSpec interruptEvent = null;
    if (eStatus == ActionStatus.CANCEL && getDBE().hasOnCancelEvent()) {
      // OnCancel Interruption behavior
      interruptEvent = getDBE().getOnCancelEvent();
    } else if (eStatus == ActionStatus.SUSPEND && getDBE().hasOnSuspendEvent()) {
      // OnSuspend Interruption Behavior
      interruptEvent = getDBE().getOnSuspendEvent();
    } else if (eStatus == ActionStatus.RESUME && getDBE().hasOnResumeEvent()) {
      // OnResume Recovery Behavior
      interruptEvent = getDBE().getOnResumeEvent();
    } else {
      //No need for interruption
      return;
    }

    Predicate interruptGoalPredicate = Factory.createPredicate("interrupt", Factory.createFOL(interruptEvent.getActor()), interruptEvent.getPredicateForm());
    Goal interruptGoal = new Goal(interruptGoalPredicate);
    ActionInterpreter ai = ActionInterpreter.createInterpreterFromEventSpec(interruptGoal, this, interruptEvent);
    //TODO: revisit
    //The above line was previously adding interruption contexts as a child of the current context, which is called and executed below,
    // but is not executed as a part of this context's AI. So this context's ChildrenContext counter will never be incremented.
    // This causes getNextStep() for this context to return the already completed interruption context, which we don't want
    childContexts.getNextAndIncrement();
    //Call interrupt event
    ai.call();
  }

  /**
   * This is overridden to handle interpretable actions.
   *
   * @param eStatus
   * @param justification
   */
  @Override
  public synchronized void setStatus(ActionStatus eStatus, Justification justification) {
    super.setStatus(eStatus, justification);
    handleInterrupt(eStatus);
  }

  public TranslationInfo getTranslation() {
    TranslationInfo temp = getDBE().getTranslationInfo();
    if (temp != null) {
      return temp.generateTranslationTemplate();
    }
    return null;
  }

}
