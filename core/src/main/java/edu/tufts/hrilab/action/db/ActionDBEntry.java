/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.lock.ResourceUnavailableException;
import edu.tufts.hrilab.action.recovery.PolicyConstraints;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.lock.ActionResourceLock;
import edu.tufts.hrilab.fol.Constant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import edu.tufts.hrilab.action.util.Utilities;
import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.stream.Collectors;

import ai.thinkingrobots.trade.*;

/**
 * An <code>ActionDBEntry</code> is the general-purpose container for one unit
 * of knowledge; can be factual (e.g., "person") or procedural (e.g., "goHome").
 */
public class ActionDBEntry extends DBEntry {

  private static Logger log = LoggerFactory.getLogger(ActionDBEntry.class);

  /**
   * allows for serialization in other locations
   */
  private static final long serialVersionUID = -6045952208937267757L;

  /**
   * Each eventSpec is a script event with its args.
   */
  private final List<EventSpec> eventSpecs;

  /**
   * Conditions that need to hold for this action to be executed.
   */
  private final List<Condition> conditions;

  /**
   * Resulting effects of that action. These are explicitly written for the action.
   */
  private final List<Effect> effects;

  /**
   * Default post-conditions (i.e., "did" effects).
   */
  private final List<Effect> defaultPostConditions;

  /**
   * Observations executed by this action.
   * TODO: move to new ObserverAction subclass
   */
  private List<Predicate> observations;

  /**
   * Policy constraints in the case that this action is a Recovery Policy.
   * TODO: move to new RecoveryPolicy subclass
   */
  private List<PolicyConstraints> policyConstraints;

  /**
   * Agents that can perform this action.
   */
  private final List<Symbol> agents;

  /**
   * A list of resources required for this action.
   */
  private final List<ActionResourceLock> resourceLocks;

  private final double posAff;
  private final double negAff;
  private final double cost;
  private final double benefit;

  private final long timeout;
  private final boolean hasTimeout;

  private final double minUrg;
  private final double maxUrg;

  /**
   * TRADE service in the case of a primitive action, null otherwise.
   */
  private TRADEServiceInfo tsi = null;

  /**
   * The file this DBEntry originates from, if it is a script.
   */
  private final String dbfile;

  /**
   * Event to be executed when action is canceled/suspended/resumed.
   */
  private EventSpec onCancelEvent;
  private EventSpec onSuspendEvent;
  private EventSpec onResumeEvent;

  public static class Builder {
    private final String defaultActorType = "agent";
    private String type;
    private String description = "";
    private List<EventSpec> eventSpecs = new ArrayList<>();
    private List<Condition> conditions = new ArrayList<>();
    private List<Effect> effects = new ArrayList<>();
    private List<Effect> defaultPostConditions = new ArrayList<>();
    private List<Predicate> observations = new ArrayList<>();
    private List<PolicyConstraints> policyConstraints = new ArrayList<>();
    private List<Symbol> agents = new ArrayList<>();
    private List<ActionBinding> roles = new ArrayList<>();
    private EventSpec onCancelEvent = null;
    private EventSpec onSuspendEvent = null;
    private EventSpec onResumeEvent = null;
    private boolean isVarArgs = false;
    private List<ActionResourceLock> resourceLocks = new ArrayList<>();
    private double posAff = 0.0;
    private double negAff = 0.0;
    private double cost = 0.0;
    private double benefit = 0.0;
    private long timeout = Integer.MAX_VALUE;
    private boolean hasTimeout = false;
    private double minUrg = 0.0;
    private double maxUrg = 1.0;
    private TRADEServiceInfo tsi;
    private String dbfile = null;
    private ActionDBEntry builtEntry = null;

    /**
     * Seed the Builder with an existing ActionDBEntry. This will copy
     * all the existing action's fields into the Builder's fields. This is useful
     * if you want to modify an existing action.
     *
     * @param entryToCopy
     */
    public Builder(ActionDBEntry entryToCopy) {
      if (entryToCopy.tsi != null) {
        log.warn("Copying a primitive action. Primitives cannot be modified in any meaning way, so this is likely not want you want.");
      }
      this.type = entryToCopy.type;
      this.description = entryToCopy.description;
      this.eventSpecs = new ArrayList<>(entryToCopy.eventSpecs);
      this.conditions = new ArrayList<>(entryToCopy.conditions);
      this.effects = new ArrayList<>(entryToCopy.effects);
      this.observations = new ArrayList<>(entryToCopy.observations);
      this.policyConstraints = new ArrayList<>(entryToCopy.policyConstraints);
      this.agents = new ArrayList<>(entryToCopy.agents);
      this.roles = new ArrayList<>(entryToCopy.roles);
      this.isVarArgs = entryToCopy.isVarArgs;
      this.onCancelEvent = entryToCopy.onCancelEvent;
      this.onSuspendEvent = entryToCopy.onSuspendEvent;
      this.onResumeEvent = entryToCopy.onResumeEvent;
      this.resourceLocks = new ArrayList<>(entryToCopy.resourceLocks);
      this.posAff = entryToCopy.posAff;
      this.negAff = entryToCopy.negAff;
      this.cost = entryToCopy.cost;
      this.benefit = entryToCopy.benefit;
      this.timeout = entryToCopy.timeout;
      this.hasTimeout = entryToCopy.hasTimeout;
      this.minUrg = entryToCopy.minUrg;
      this.maxUrg = entryToCopy.maxUrg;
      this.tsi = entryToCopy.tsi;
      this.dbfile = entryToCopy.dbfile;
    }

    public Builder(String type) {
      this.type = type;
    }

    /**
     * Getter for default semantic type for ?actor role.
     *
     * @return
     */
    public String getDefaultActorType() {
      return defaultActorType;
    }

    /**
     * Set the action type (i.e., action method name).
     *
     * @param type
     * @return
     */
    public Builder setType(String type) {
      this.type = type;
      return this;
    }

    public Builder setDescription(String val) {
      description = val;
      return this;
    }

    public Builder popEventSpec() {
      eventSpecs.remove(eventSpecs.size() - 1);
      return this;
    }

    /**
     * Add a new script event specification to a script
     *
     * @param val the event to be added to the (ordered) eventSpecs list (an
     *            event specification includes the name of the action to be executed and
     *            its parameters)
     * @return this Builder instance
     */
    public Builder addEventSpec(EventSpec val) {
      eventSpecs.add(val);
      return this;
    }

    /**
     * Prepend a sequence of actions to the event specs.
     *
     * @param eventSpecs
     * @return
     */
    public Builder prependEventSpecs(List<EventSpec> eventSpecs) {
      this.eventSpecs.addAll(0, eventSpecs);
      return this;
    }

    /**
     * Append a sequence of actions to the event specs.
     *
     * @param eventSpecs
     * @return
     */
    public Builder appendEventSpecs(List<EventSpec> eventSpecs) {
      this.eventSpecs.addAll(eventSpecs);
      return this;
    }

    public Builder clearEventSpecList() {
      this.eventSpecs = new ArrayList<>();
      return this;
    }

    /**
     * Add a condition that must hold for this action to be possible.
     *
     * @param c the condition to be added
     * @return this Builder instance
     */
    public Builder addCondition(Condition c) {
      if (!conditions.contains(c)) {
        conditions.add(c);
      }
      return this;
    }

    /**
     * Remove a condition.
     *
     * @param c condition to be removed
     * @return
     */
    public Builder removeCondition(Condition c) {
      conditions.remove(c);
      return this;
    }

    /**
     * Add collection of conditions that must hold for this action to be possible
     *
     * @param conditionCollection collection of conditions to be added
     * @return this Builder instance
     */
    public Builder addConditions(Collection<Condition> conditionCollection) {
      conditionCollection.forEach(condition -> this.addCondition(condition));
      return this;
    }

    /**
     * Add an effect to be sent as a planner update whenever the action is
     * executed.
     *
     * @param effect the effect to be added
     * @return this Builder instance
     */
    public Builder addEffect(Effect effect) {
      if (!effects.contains(effect)) {
        effects.add(effect);
      }
      return this;
    }

    /**
     * Add a collection of effects to be sent as a planner update whenever the action is
     * executed.
     *
     * @param effectCollection collection of effects to be added
     * @return this Builder instance
     */
    public Builder addEffects(Collection<Effect> effectCollection) {
      effectCollection.forEach(effect -> this.addEffect(effect));
      return this;
    }

    /**
     * Add an effect to the beginning of the Effect list to be sent as a planner update whenever the action is
     * executed.
     *
     * @param effect the effect to be added
     * @return this Builder instance
     */
    public Builder prependEffect(Effect effect) {
      if (!effects.contains(effect)) {
        effects.add(0, effect);
      }
      return this;
    }

    /**
     * Replace an effect at the same location in the effects list.
     *
     * @param toRemove the effect to be removed
     * @param toAdd    the effect to be added
     * @return this Builder instance
     */
    public Builder replaceEffect(Effect toRemove, Effect toAdd) {
      int i = effects.indexOf(toRemove);
      if (i == -1) {
        log.warn("Effects list does not contain effect: " + toRemove);
      } else {
        effects.set(i, toAdd);
      }
      return this;
    }

    /**
     * Remove an effect.
     *
     * @param effect the effect to be removed
     * @return this Builder instance
     */
    public Builder removeEffect(Effect effect) {
      effects.remove(effect);
      return this;
    }

    /**
     * Contains an effect.
     *
     * @param effect the effect
     * @return this Builder instance
     */
    public boolean hasEffect(Effect effect) {
      return effects.contains(effect);
    }

    /**
     * Adds a state that this action can observe.
     *
     * @param obs observable fact
     * @return this Builder instance
     */
    public Builder addObservation(Predicate obs) {
      // TODO: EAK: should we apply semantic type info here too?
      observations.add(obs);

      // get the current roles
      Set<String> obsVars = roles.stream().map(ActionBinding::getName).collect(Collectors.toSet());

      // add all obs args that are not already in the roles, and not ?actor
      for (Variable arg : obs.getVars()) {
        if (!obsVars.contains(arg.getName()) && !arg.getName().equals("?actor")) {
          addRole(new ActionBinding.Builder(arg.getName(), Symbol.class).setIsLocal(true).build());
          obsVars.add(arg.getName());
        }
      }
      return this;
    }

    /**
     * Adds a recovery policy constraint.
     *
     * @param constraint policy constraint
     * @return this Builder instance
     */
    public Builder addRecoveryPolicyConstraint(PolicyConstraints constraint) {
      policyConstraints.add(constraint);
      return this;
    }

    /**
     * Add EventSpec specifying what to do when this action is cancelled.
     *
     * @param onCancelEvent
     * @return
     */
    public Builder addOnCancelEvent(EventSpec onCancelEvent) {
      this.onCancelEvent = onCancelEvent;
      return this;
    }

    /**
     * Add EventSpec specifying what to do when this action is suspended.
     *
     * @param onSuspendEvent
     * @return
     */
    public Builder addOnSuspendEvent(EventSpec onSuspendEvent) {
      this.onSuspendEvent = onSuspendEvent;
      return this;
    }

    /**
     * Add EventSpec specifying what to do when this action is resumed (after being suspended).
     * @param onResumeEvent
     * @return
     */
    public Builder addOnResumeEvent(EventSpec onResumeEvent) {
      this.onResumeEvent = onResumeEvent;
      return this;
    }

    /**
     * Checks if OnCancel Event has been added to the Builder.
     * @return
     */
    public boolean hasOnCancelEvent() {
      return onCancelEvent != null;
    }

    /**
     * Checks if OnSuspend Event has been added to the Builder.
     * @return
     */
    public boolean hasOnSuspendEvent() {
      return onSuspendEvent != null;
    }

    /**
     * Checks if OnResume Event has been added to the Builder.
     * @return
     */
    public boolean hasOnResumeEvent() {
      return onResumeEvent != null;
    }

    /**
     * Adds an agent on which this action will have an effect.
     *
     * @param agentName agent name
     * @return this Builder instance
     */
    public Builder addAgent(Symbol agentName) {
      if (!agents.contains(agentName)) {
        agents.add(agentName);
      }
      return this;
    }

    /**
     * Replace all existing roles with the passed in roles.
     *
     * @param roles
     * @return
     */
    public Builder setRoles(List<ActionBinding> roles) {
      roles.clear();
      addRoles(roles);
      return this;
    }

    /**
     * Add new role.
     *
     * @param newRole
     * @return
     */
    public Builder addRole(ActionBinding newRole) {
      if (newRole == null) {
        log.warn("Trying to add null role. Ignoring.");
        return this;
      }

      if (newRole.name.equalsIgnoreCase("?actor") && !roles.isEmpty()) {
        log.error("?actor role can only be first arg. Ignoring.");
        return this;
      }

      if (!containsRole(newRole.name)) {
        roles.add(newRole);
        if (newRole.isVarArg()) {
          this.isVarArgs = true;
        }
      } else {
        log.warn("Role cannot be declared again. Ignoring it: " + newRole);
      }

      return this;
    }

    /**
     * Convenience method for adding set of new roles. Internally calls the single
     * addRole method to do any necessary checks before appending to roles list.
     *
     * @param newRoles
     * @return
     */
    public Builder addRoles(List<ActionBinding> newRoles) {
      for (ActionBinding newRole : newRoles) {
        addRole(newRole);
      }
      return this;
    }

    /**
     * Shallow copy of roles.
     *
     * @return
     */
    public List<ActionBinding> getRoles() {
      return new ArrayList<>(roles);
    }

    /**
     * Helper method to check if role with specific name already exists.
     *
     * @param roleName
     * @return
     */
    private boolean containsRole(String roleName) {
      return roles.stream().anyMatch(role -> role.name.equals(roleName));
    }

    public Builder addResourceLock(String name) {
      if (ActionResourceLock.getLock(name) == null) {
        log.error("[addResourceLock] not a valid lock name: " + name + " Not adding lock to action: " + type);
      } else {
        resourceLocks.add(ActionResourceLock.getLock(name));
      }
      return this;
    }

    // TODO: Never used?
    public Builder setPosAffect(String val) {
      posAff = Double.parseDouble(val);
      return this;
    }

    // TODO: Never used?
    public Builder setNegAffect(String val) {
      negAff = Double.parseDouble(val);
      return this;
    }

    public Builder setCost(String val) {
      cost = Double.parseDouble(val);
      return this;
    }

    public Builder setBenefit(String val) {
      benefit = Double.parseDouble(val);
      return this;
    }

    public Builder setTimeout(String val) {
      hasTimeout = true;
      timeout = Long.parseLong(val);
      return this;
    }

    public Builder setTimeout(Long val) {
      hasTimeout = true;
      timeout = val;
      return this;
    }

    public Builder setMinUrg(String val) {
      minUrg = Double.parseDouble(val);
      return this;
    }

    public Builder setMaxUrg(String val) {
      maxUrg = Double.parseDouble(val);
      return this;
    }

    public Builder setIsVarArgs(boolean val) {
      isVarArgs = val;
      return this;
    }

    public Builder setActionImplementer(TRADEServiceInfo tsi) {
      this.tsi = tsi;
      return this;
    }

    public Builder setDBFile(String file) {
      this.dbfile = file;
      return this;
    }

    /**
     * Generates the default post-condition defined as:
     * succeeded(?actor,action(?arg1, ?arg2, ...))
     * failed(?actor,action(?arg1, ?arg2, ...))
     */
    private void generateDefaultPC() {
      List<Variable> args = new ArrayList<>(roles.stream()
              .filter(r -> !r.isLocal()) // Get non-local roles
              .filter(r -> !r.isReturn()) // Get non-return roles
              .map(r -> new Variable(r.getName())).collect(Collectors.toList()));
      //brad:separate optional args and required args
      List<Variable> optionalArgs = new ArrayList<>(roles.stream()
              .filter(r -> !r.isLocal()) // get non-local args
              .filter(ActionBinding::hasDefaultValue) // get args with default value
              .map(r -> new Variable(r.getName())).collect(Collectors.toList()));
      args.removeAll(optionalArgs);
      //brad: add a new default PC for each set of possible default args, in order
      //if the way that we can specify optional args changes, this will have to change.

      // remove actor (first role) to put outside of action signature
      //Variable actor = args.remove(0);

      // remove any existing generic post-conditions so they aren't duplicated
      defaultPostConditions = new ArrayList<>();

      int argCounter = 0;
      do {
        List<Variable> argList = new ArrayList<>();
        argList.addAll(args);
        //get subset of args
        if (!optionalArgs.isEmpty()) {
          argList.addAll(optionalArgs.subList(0, argCounter));
        }
        // actionName(?arg1, ?arg2, ...) -> no actor
        Predicate innerPredicate = Factory.createPredicate(type, argList);
        // successful execution: succeeded(?actor, actionName(?arg0, ?arg1, ...)
        // failed execution: failed(?actor, actionName(?arg0, ?arg1, ...)
        Predicate succeededPredicate = Factory.createPredicate("succeeded", innerPredicate);
        defaultPostConditions.add(new Effect(succeededPredicate, EffectType.SUCCESS, Observable.FALSE, true));
        Predicate failPredicate = Factory.createPredicate("failed", innerPredicate);
        defaultPostConditions.add(new Effect(failPredicate, EffectType.FAILURE, Observable.FALSE, true));
        argCounter++;
      } while (argCounter <= optionalArgs.size());

    }

    public ActionDBEntry build(boolean addToDatabase) {

      // ensure ?actor is the first role
      if (roles.isEmpty() || !roles.get(0).getName().equals("?actor")) {
        roles.add(0, new ActionBinding.Builder("?actor:" + defaultActorType, Symbol.class).build());
      }

      // Generate default/generic post condition
      this.generateDefaultPC();

      // create new ActionDBEntry
      ActionDBEntry newDBE = new ActionDBEntry(this);

      if (addToDatabase) {
        Database.getInstance().addActionDBEntry(newDBE);
      }

      return newDBE;
    }
  }

  protected ActionDBEntry(Builder builder) {
    // Build DBEntry with basic information
    super(builder.type, builder.description, builder.roles, builder.isVarArgs);

    // Build ActionDBEntry with action-specific information
    eventSpecs = builder.eventSpecs;
    conditions = builder.conditions;
    effects = builder.effects;
    defaultPostConditions = builder.defaultPostConditions;
    observations = builder.observations;
    onCancelEvent = builder.onCancelEvent;
    onSuspendEvent = builder.onSuspendEvent;
    onResumeEvent = builder.onResumeEvent;
    policyConstraints = builder.policyConstraints;
    agents = builder.agents;
    resourceLocks = builder.resourceLocks;
    posAff = builder.posAff;
    negAff = builder.negAff;
    cost = builder.cost;
    benefit = builder.benefit;
    timeout = builder.timeout;
    hasTimeout = builder.hasTimeout;
    minUrg = builder.minUrg;
    maxUrg = builder.maxUrg;
    tsi = builder.tsi;
    dbfile = builder.dbfile;
  }

  /**
   * Check if action is a primitive action (i.e., has a TRADEServiceInfo).
   *
   * @return
   */
  public boolean isPrimitive() {
    return tsi != null;
  }

  /**
   * Check if action is an action script (i.e., does not have a TRADEServiceInfo).
   *
   * @return
   */
  public boolean isScript() {
    return tsi == null;
  }

  /**
   * Get the TRADEServiceInfo of the method implementing this action in TRADE.
   * This will be null for non-primitive actions (i.e., action scripts).
   *
   * @return the TRADEServiceInfo
   */
  public TRADEServiceInfo getServiceInfo() {
    return tsi;
  }

  /**
   * Get all the conditions that must be true before the action can begin.
   *
   * @return An unmodifiable list of pre conditions
   */
  public List<Condition> getPreConditions() {
    List<Condition> preConditions = conditions.stream()
            .filter(c -> c.getType() == ConditionType.PRE).collect(Collectors.toList());
    return preConditions;
  }

  /**
   * Get all the conditions that must be true before and during the action.
   *
   * @return An unmodifiable list of overAllConditions
   */
  public List<Condition> getOverallConditions() {
    List<Condition> overallConditions = conditions.stream()
            .filter(c -> c.getType() == ConditionType.OVERALL).collect(Collectors.toList());
    return overallConditions;
  }

  /**
   * Get all the conditions that must be true before and during the action.
   *
   * @return An unmodifiable list of overAllConditions
   */
  public List<Condition> getObligationConditions() {
    List<Condition> obligationConditions = conditions.stream()
            .filter(c -> c.getType() == ConditionType.OBLIGATION).collect(Collectors.toList());
    return obligationConditions;
  }

  /**
   * Get all the Conditions for this action.
   *
   * @return An unmodifiable copy of the list of conditions. The returned list is
   * unmodifiable to require all manipulations of the list to be done through
   * the containing ActionDBEntry.
   */
  public List<Condition> getConditions() {
    return Collections.unmodifiableList(conditions);
  }

  /**
   * Get the postconditions for the current action. The postconditions are the
   * intended effects.
   *
   * @return a list of the postconditions
   */
  public List<Effect> getPostConditions() {
    List<Effect> post = new ArrayList<>();
    effects.stream().filter(e -> e.getType().equals(EffectType.SUCCESS) || e.getType().equals(EffectType.ALWAYS)).forEach(e -> post.add(e));
    defaultPostConditions.stream().filter(e -> e.getType().equals(EffectType.SUCCESS)).forEach(e -> post.add(e));
    return post;
  }

  /**
   * Check if this action is an observer.
   *
   * @return
   */
  public boolean isObserver() {
    return !observations.isEmpty();
  }

  /**
   * Get the observations carried out by the action.
   * TODO: move to new subclass
   *
   * @return a list of the observations in Predicate form
   */
  public List<Predicate> getObservations() {
    return Collections.unmodifiableList(observations);
  }

  /**
   * Check if this action is a recovery policy.
   *
   * @return
   */
  public boolean isRecoveryPolicy() {
    return !policyConstraints.isEmpty();
  }

  /**
   * Get recovery policy constraints (i.e., constraining when a policy can be used).
   * TODO: move to new subclass
   *
   * @return shallow copy of the policy constraints
   */
  public List<PolicyConstraints> getPolicyConstraints() {
    return Collections.unmodifiableList(policyConstraints);
  }

  /**
   * Get the cost
   *
   * @return the action's (nominal) cost
   */
  public double getCost() {
    return cost;
  }

  /**
   * Get the benefit
   *
   * @return the action's (nominal) benefit
   */
  public double getBenefit() {
    return benefit;
  }

  /**
   * Get affect state
   *
   * @param positive boolean indicating whether to add positive affect
   * @return the requested affect state value
   */
  public double getAffect(boolean positive) {
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
  public double getAffectEval() {
    //double a = 0.5 + (posAff*posAff - negAff*negAff) * 0.5;
    double a = 1 + posAff * posAff - negAff * negAff;

    return a;
  }

  /**
   * Get the maxUrg
   *
   * @return the action's maxUrg
   */
  public double getMaxUrg() {
    return maxUrg;
  }

  /**
   * Get the minUrg
   *
   * @return the action's minUrg
   */
  public double getMinUrg() {
    return minUrg;
  }

  /**
   * Get the timeout
   *
   * @return the action's (nominal) timeout
   */
  public long getTimeout() {
    return timeout;
  }

  public boolean hasTimeout() {
    return hasTimeout;
  }

  /**
   * Get the names of all resource locks. Useful mainly for writing to XML.
   *
   * @return list of all resource lock names
   */
  public List<String> getResourceLockNames() {
    List<String> lockNames = new ArrayList<>();
    for (ActionResourceLock each_lock : resourceLocks) {
      if (each_lock != null) {
        lockNames.add(each_lock.getLockName());
      }
    }
    return lockNames;
  }

  /**
   * Acquire all locks registered for this action
   *
   * @param actionInt The AI that needs the lock
   * @param greedy    If true then hold whatever locks can be acquired
   * @return the list of resources acquired
   */
  public List<ActionResourceLock> acquireResourceLocks(ActionInterpreter actionInt, boolean greedy)
          throws ResourceUnavailableException {
    ArrayList<ActionResourceLock> heldLocks = new ArrayList<>();

    for (ActionResourceLock lock : resourceLocks) {
      if (!lock.nonBlockingAcquire(actionInt)) {
        ActionInterpreter owner = lock.getOwner();
        String descr;
        if (owner != null) {
          descr = owner.getGoal().toString();
        } else {
          descr = "unknown";
        }
        if (!greedy) {
          // since we could not acquire all the locks, let's not greedily
          // hold onto all of them
          for (ActionResourceLock held : heldLocks) {
            held.release(actionInt);
          }
        }
        throw new ResourceUnavailableException(lock.getLockName(), descr);
      }
      heldLocks.add(lock);
    }
    return heldLocks;
  }

  /**
   * Get the next script event specification from a script that matches.
   *
   * @param enames a list of event names to search for (e.g., for else, elseif)
   * @param start  the opening name of a potentially nested statement (e.g., if,
   *               while)
   * @param end    the closing name of a potentially nested statement (e.g., endif,
   *               endwhile)
   * @param pos    the starting point in the list of events to begin searching
   * @return the spec of the next event, or null if there is none (an event
   * specification includes the name of the action to be executed and its
   * parameters). EventSpec is immutable so returning reference to one is okay.
   */
  public EventSpec getEventSpec(String enames, String start,
                                String end, int pos) {
    EventSpec nextEvent;
    ArrayList<String> eNames = new ArrayList<>();
    StringTokenizer eNameTok = new StringTokenizer(enames);

    while (eNameTok.hasMoreTokens()) {
      eNames.add(eNameTok.nextToken());
    }

    while (pos < eventSpecs.size()) {
      nextEvent = eventSpecs.get(pos++);
      // Skip start-end sequences (e.g., everything between IF and
      // ENDIF)
      if (nextEvent.getCommand().equals(start)) {
        return getEventSpec(end, start, end, pos);
      } else {
        if (eNames.contains(nextEvent.getCommand())) {
          return nextEvent;
        }
      }
    }
    return null;
  }

  /**
   * Get the next script event specification from a script that matches
   *
   * @param enames a list of event names to search for (e.g., for else, elseif)
   * @param pos    the starting point in the list of events to begin searching
   * @return the spec of the next event, or null if there is none (an event
   * specification includes the name of the action to be executed and its
   * parameters). EventSpec is immutable so returning reference to one is okay.
   */
  public EventSpec getEventSpec(String enames, int pos) {
    EventSpec nextEvent;
    ArrayList<String> eNames = new ArrayList<>();
    StringTokenizer eNameTok = new StringTokenizer(enames);

    while (eNameTok.hasMoreTokens()) {
      eNames.add(eNameTok.nextToken());
    }

    while (pos < eventSpecs.size()) {
      nextEvent = eventSpecs.get(pos++);
      if (eNames.contains(nextEvent.getCommand())) {
        return nextEvent;
      }
    }
    return null;
  }

  /**
   * Get the next script event specification from a script
   *
   * @param pos the starting point in the list of events to begin searching
   * @return the spec of the next event, or null if there is none (an event
   * specification includes the name of the action to be executed and its
   * parameters). EventSpec is immutable so returning reference to one is okay.
   */
  public EventSpec getEventSpec(int pos) {
    if (pos < eventSpecs.size()) {
      return eventSpecs.get(pos);
    } else {
      return null;
    }
  }

  /**
   * Get the list of event specifications for the script.
   *
   * @return An unmodifiable copy of the list of events. The returned list is
   * unmodifiable to require all manipulations of the list to be done through
   * the containing ActionDBEntry.
   */
  public List<EventSpec> getEventSpecs() {
    return Collections.unmodifiableList(eventSpecs);
  }

  /**
   * Get all the effects for this action.
   *
   * @return An unmodifiable copy of the list of effects. The returned list is
   * unmodifiable to require all manipulations of the list to be done through
   * the containing ActionDBEntry.
   */
  public List<Effect> getEffects() {
    List<Effect> allEffects = new ArrayList<>();
    allEffects.addAll(effects);
    allEffects.addAll(defaultPostConditions);
    return allEffects;
  }

  /**
   * Called when the action completes and creates a list of the effects based on
   * the status of the action.
   *
   * @param status The status of the action determines which additional effects
   *               are returned. If {@link edu.tufts.hrilab.action.ActionStatus#SUCCESS} then the
   *               successEffects are included. If {@link edu.tufts.hrilab.action.ActionStatus#FAIL} then
   *               the failureEffects are included. If {@link ActionStatus#CANCEL} then the
   *               nonperformanceEffects are included.
   * @return All of the effects of this action based on the provided status of
   * the action.
   */
  public List<Effect> getEffects(ActionStatus status) {
    List<Effect> update = new LinkedList<>();

    // Add always effects
    update.addAll(effects.stream()
            .filter(effect -> effect.getType() == EffectType.ALWAYS)
            .collect(Collectors.toList()));

    // Add ActionStatus-specific effects
    if (status.isFailure()) {
      // add all failure effects
      update.addAll(effects.stream()
              .filter(effect -> effect.getType() == EffectType.FAILURE)
              .collect(Collectors.toList()));

      // Add failure default post conditions
      update.addAll(defaultPostConditions.stream()
              .filter(effect -> effect.getType() == EffectType.FAILURE)
              .collect(Collectors.toList()));
    } else if (status == ActionStatus.SUCCESS) {
      // add all success effects
      update.addAll(effects.stream()
              .filter(effect -> effect.getType() == EffectType.SUCCESS)
              .collect(Collectors.toList()));

      // Add default post conditions
      update.addAll(defaultPostConditions.stream()
              .filter(effect -> effect.getType() == EffectType.SUCCESS)
              .collect(Collectors.toList()));

    } else if (status == ActionStatus.CANCEL) {
      // add all non-performance effects
      update.addAll(effects.stream()
              .filter(effect -> effect.getType() == EffectType.NONPERF)
              .collect(Collectors.toList()));

    }

    return update;
  }

  /**
   * Get the list of agents that can perform this action.
   *
   * @return agents that can perform this action
   */
  public List<Symbol> getAgents() {
    return Collections.unmodifiableList(agents);
  }

  /**
   * Returns the file this script was parsed from.
   *
   * @return file this script was parsed from.
   */
  public String getDBFile() {
    return dbfile;
  }

  /**
   * Checks if this ActionDBEntry conflicts with another ActionDBEntry.
   *
   * @param other
   * @return true if this ActionDBEntry conflicts with other.
   */
  public boolean conflictsWith(ActionDBEntry other) {
    if (super.conflictsWith(other)) {
      if (this.agents.size() > 0) {
        // Check if there is an intersection in the set of agents.
        for (Symbol agent : this.agents) {
          if (other.agents.contains(agent)) {
            return true;
          }
        }
      } else if (other.agents.size() == 0) {
        return true; // Both sets empty!
      }
    }
    return false; // actions do not conflict
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder("Action" + super.toString());

    for (Condition c : conditions) {
      sb.append("\tcondition: " + c + '\n');
    }

    for (Effect e : effects) {
      sb.append("\teffect: " + e + '\n');
    }

    for (Predicate o : observations) {
      sb.append("\tobserves: " + o + '\n');
    }

    for (EventSpec a : eventSpecs) {
      sb.append("\teventSpec: " + a + '\n');
    }

    if (agents == null) {
      sb.append("\tagents: all\n");
    } else {
      for (Symbol agent : agents) {
        sb.append("\tagent: " + agent + '\n');
      }
    }

    return sb.toString();
  }

  /**
   * Get action signature in predicate form, ignoring ?actor arg, optional arguments, and return arguments (if any).
   *
   * @return
   */
  public Predicate getSignature(boolean includeActor) {
    List<Symbol> args = new ArrayList<>();

    if (includeActor) {
      ActionBinding actorRole = this.getRole("?actor");
      args.add(new Variable(actorRole.getName(), actorRole.getSemanticType()));
    }

    for (ActionBinding role : this.getRequiredInputRoles()) {
      String name = role.name;
      if (Utilities.isScriptVariable(name)) {
        args.add(new Variable(name, role.getSemanticType()));
      } else {
        log.error("Non-variable as action role. How is that possible?");
        args.add(new Constant(name, role.getSemanticType()));
      }
    }
    return Factory.createPredicate(this.type, args);
  }

  /**
   * Get all the possible action signatures in predicate form with optional argument permutations, ignoring actor and return args
   *
   * @return
   */
  public List<Predicate> getSignatureOptions(boolean includeActor) {
    List<Predicate> signatures = new ArrayList<>();
    signatures.add(this.getSignature(includeActor));
    List<Symbol> args = signatures.get(0).getArgs();
    for (ActionBinding role : this.getOptionalInputRoles()) {
      String name = role.getName();
      if (Utilities.isScriptVariable(name)) {
        args.add(new Variable(name, role.getSemanticType()));
      } else {
        args.add(new Constant(name, role.getSemanticType()));
      }
      signatures.add(Factory.createPredicate(this.type, args));
    }
    return signatures;
  }

  /**
   * Execute the primitive action.
   *
   * @param args
   * @return
   */
  public Justification executeAction(Collection<ActionBinding> args) {
    log.debug("executeAction with " + tsi);
    Justification executionJustification;
    boolean removedActorArg = false;
    if (tsi != null) {
      // if first arg of primitive action is not "actor", remove ?actor arg so that
      // it's not passed to primitive
      if (tsi.serviceParameterNames.length == 0 || !tsi.serviceParameterNames[0].equals("actor")) {
        args = args.stream()
                .filter(arg -> !arg.getName().equals("?actor"))
                .collect(Collectors.toList());
        removedActorArg = true;
      }

      // collect non-return args and find return arg to be filled
      List<Object> argBindings = new ArrayList();
      ActionBinding returnArg = null;
      if (args != null) {
        for (ActionBinding arg : args) {
          if (arg.isReturn) {
            if (returnArg != null) {
              log.error("More than one return value in executeAction for primitive action: " + tsi);
            }
            returnArg = arg;
          } else {
            if (!arg.isBound()) {
              log.error("Trying to call executeAction with unbound arguments for primitive action: " + tsi);
            }
            Object value = arg.getBindingDeep();
            argBindings.add(value);
          }
        }
      }

      try {
        // make call to primitive action
        Object returnValue = tsi.call(Object.class, argBindings.toArray());
        // bind return value
        if (returnArg == null) {
          log.debug("No designated return argument in executeAction for primitive action: " + tsi);
        } else {
          returnArg.bindDeep(returnValue);
        }
        executionJustification = new ConditionJustification(true);
      } catch (TRADEException e) {
        log.error("Exception while calling " + tsi + " for action: " + type + " with args : " + args, e);

        //TODO:brad: I think that this gets triggered incorrectly in some cases, and leads to unhelpful error messages

        if (args != null) {
          Iterator<ActionBinding> rit = this.roles.iterator();
          Iterator<ActionBinding> ait = args.iterator();

          if (removedActorArg) {
            // skip ?actor role
            rit.next();
          }

          while (rit.hasNext() && ait.hasNext()) {
            ActionBinding role = rit.next();
            ActionBinding arg = ait.next();
            if (!role.getBindingTypeDeep().equals(arg.getBindingTypeDeep())) {
              log.error("[" + this.type + "] Type mismatch:");
              log.error("\tExpected (" + role.getBindingTypeDeep() + ") " + role.name + ", but received ("
                      + arg.getBindingTypeDeep() + ") " + arg.name);
            }
          }
        }

        // TODO: this should probably use the default post-condition instead of the method name
        executionJustification = new ConditionJustification(false, Factory.createPredicate("succeed", tsi.serviceName));
      }
    } else {
      log.error("No method invocation defined for " + this.getName());
      executionJustification = new ConditionJustification(false, Factory.createPredicate("set(methodName)"));
    }
    return executionJustification;
  }

  /**
   * If action has an onCancel EventSpec defined.
   *
   * @return
   */
  public boolean hasOnCancelEvent() {
    return (onCancelEvent != null);
  }

  /**
   * If action has an onSuspend EventSpec defined.
   *
   * @return
   */
  public boolean hasOnSuspendEvent() {
    return (onSuspendEvent != null);
  }

  /**
   * If action has an onResume EventSpec defined.
   *
   * @return
   */
  public boolean hasOnResumeEvent() {
    return (onResumeEvent != null);
  }

  public EventSpec getOnCancelEvent() {
    return onCancelEvent;
  }

  public EventSpec getOnSuspendEvent() {
    return onSuspendEvent;
  }

  public EventSpec getOnResumeEvent() {
    return onResumeEvent;
  }

  /**
   * Attempts to apply the semantic type information from the roles of this ActionDBEntry
   * to the input term.
   * <p>
   * NOTE: it's possible that the returned predicate is still not fully typed if the roles aren't all typed
   *
   * @param term
   * @return
   */
  public Term applySemanticTypes(Term term) {
    if (!edu.tufts.hrilab.fol.util.Utilities.isFullyTyped(term)) {
      List<Symbol> args = term.getArgs(); // shallow copy, will be modified in place
      for (int i = 0; i < args.size(); ++i) {
        Symbol arg = args.get(i);
        if (arg.isTerm()) {
          // recurse into nested predicates (e.g., not(fact(arg0,arg1)))
          args.set(i, applySemanticTypes((Term) arg));
        } else if (arg.isVariable() && !arg.hasType()) {
          // try to find matching role for untyped variable
          ActionBinding matchingRole = roles.stream()
                  .filter(role -> role.getName().equals(arg.getName()) && role.hasSemanticType())
                  .findFirst().orElse(null);
          if (matchingRole != null) {
            Symbol typedArg = new Variable(arg.getName(), matchingRole.getSemanticType());
            args.set(i, typedArg);
          }
        }
      }

      // add newly typed predicat
      // NOTE: it's possible that the predicate is still not fully typed if the roles aren't all typed
      return Factory.createPredicate(term.getName(), args);
    } else {
      // already fully typed, just return it
      return term;
    }
  }

  @Override
  public boolean equals(Object o) {
    if (o == null) {
      return false;
    }
    if (!o.getClass().equals(this.getClass())) {
      return false;
    }
    ActionDBEntry other = (ActionDBEntry) o;
    if (!Objects.equals(other.type, this.type)) {
      return false;
    }
    if (!Objects.equals(other.description, this.description)) {
      return false;
    }
    if (!Objects.equals(other.roles, this.roles)) {
      return false;
    }
    if (!Objects.equals(other.eventSpecs, this.eventSpecs)) {
      return false;
    }
    if (!Objects.equals(other.conditions, this.conditions)) {
      return false;
    }
    if (!Objects.equals(other.effects, this.effects)) {
      return false;
    }
    if (!Objects.equals(other.defaultPostConditions, this.defaultPostConditions)) {
      return false;
    }
    if (!Objects.equals(other.observations, this.observations)) {
      return false;
    }
    if (!Objects.equals(other.policyConstraints, this.policyConstraints)) {
      return false;
    }
    if (!Objects.equals(other.agents, this.agents)) {
      return false;
    }
    if (!Objects.equals(other.resourceLocks, this.resourceLocks)) {
      return false;
    }
    if (other.posAff != this.posAff) {
      return false;
    }
    if (other.negAff != this.negAff) {
      return false;
    }
    if (other.cost != this.cost) {
      return false;
    }
    if (other.benefit != this.benefit) {
      return false;
    }
    if (other.timeout != this.timeout) {
      return false;
    }
    if (other.hasTimeout != this.hasTimeout) {
      return false;
    }
    if (other.minUrg != this.minUrg) {
      return false;
    }
    if (other.maxUrg != this.maxUrg) {
      return false;
    }
    if (other.tsi != this.tsi) {
      return false;
    }
    if (other.dbfile != this.dbfile) {
      return false;
    }
    return true;
  }

  @Override
  public int hashCode() {
    int result = 1;
    result = 31 * result + Objects.hashCode(type);
    result = 31 * result + Objects.hashCode(description);
    result = 31 * result + Objects.hashCode(roles);
    result = 31 * result + Objects.hashCode(eventSpecs);
    result = 31 * result + Objects.hashCode(conditions);
    result = 31 * result + Objects.hashCode(effects);
    result = 31 * result + Objects.hashCode(defaultPostConditions);
    result = 31 * result + Objects.hashCode(observations);
    result = 31 * result + Objects.hashCode(policyConstraints);
    result = 31 * result + Objects.hashCode(agents);
    result = 31 * result + Objects.hashCode(resourceLocks);
    result = 31 * result + (int) Double.doubleToLongBits(posAff);
    result = 31 * result + (int) Double.doubleToLongBits(negAff);
    result = 31 * result + (int) Double.doubleToLongBits(cost);
    result = 31 * result + (int) Double.doubleToLongBits(benefit);
    result = 31 * result + (int) Double.doubleToLongBits(timeout);
    result = 31 * result + (hasTimeout ? 1 : 0);
    result = 31 * result + (int) Double.doubleToLongBits(minUrg);
    result = 31 * result + (int) Double.doubleToLongBits(maxUrg);
    result = 31 * result + Objects.hashCode(tsi);
    return 31 * result + Objects.hashCode(dbfile);
  }
}
