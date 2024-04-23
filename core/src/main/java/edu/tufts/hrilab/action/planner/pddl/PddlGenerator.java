/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.planner.pddl;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.pddl.Action;
import edu.tufts.hrilab.pddl.Pddl;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

/**
 * This class gathers information from several places/components in DIARC to generate a pddl domain and problem
 * file on the fly. This is used during the action selection process in order to leverage pddl planners.
 */
public class PddlGenerator {

  public Pddl pddl;
  Pddl.PddlBuilder builder;
  private final Set<String> typeNames; //Holds the names of all seen types
  private static final Logger log = LoggerFactory.getLogger(PddlGenerator.class);
  private final Set<Symbol> ignoredAgents;

  public PddlGenerator(Goal goal, ActionConstraints constraints, List<? extends Term> facts, List<Pair<Term, List<Term>>> rules) {
    typeNames = new HashSet<>();
    ignoredAgents = getAgentsNotInActor(goal.getActor());
    builder = new Pddl.PddlBuilder("diarc");

    // NOTE: the order of these methods matters
    generateRequirements(); // no dependencies
    scrapeBelief(facts, goal.getActor()); //generates PDDL types, functions, objects, constants, predicates -- no dependencies
    generateRules(rules); // generates rules -- no dependencies
    scrapeConsultants(goal.getActor()); // generates predicates, inits, constants, objects -- no dependencies
    scrapeActionDatabase(constraints); //generates PDDL actions, and predicates -- depends on constants and objects
    generateInit(facts, goal.getActor()); // generates inits -- depends on predicate and functions
    generateGoal(goal);
  }

  public PddlGenerator(Goal goal, List<? extends Term> facts) {
    this(goal, new ActionConstraints(), facts, new ArrayList<>());
  }

  public PddlGenerator(Goal goal, StateMachine stateMachine) {
    this(goal, new ActionConstraints(), stateMachine.getAllFacts(), stateMachine.getRules());
  }

  /**
   * Builds the PDDL object from the builder.
   *
   * @return
   */
  public Pddl build() {
    pddl = builder.build();
    return pddl;
  }

  /**
   * Adds the grounded goal predicates to the builder.
   *
   * @param goal
   */
  private void generateGoal(Goal goal) {
    Set<Predicate> goalPreds = Utils.expand(goal.getPredicate());
    if (goal.getMetric() != null ) {
      builder.addMetric(goal.getMetric());
    }
    for (Predicate goalPred : goalPreds) {
      //In the case of a disjunctive goal
      if (goalPred.getName().equals("or")) {
        //Filter out unknown predicates
        List<Predicate> filteredPreds = new ArrayList<>();
        for (Symbol arg : goalPred.getArgs()) {
          if (!arg.isPredicate()) {
            log.warn("Disjunctive goal contains non-predicate goal. Ignoring: " + arg);
          }
          Predicate goalState = (Predicate) arg;
          if (!Utils.isValidPredicate(goalState, builder)) {
            log.warn("Disjunctive goal contains invalid goal. Ignoring: " + arg);
            continue;
          }
          filteredPreds.add(goalState);
        }
        if (filteredPreds.isEmpty()) {
          log.warn("Disjunctive goal contains no valid goals. Ignoring: " + goalPred);
          continue;
        }
        goalPred = new Predicate("or", filteredPreds); //rebuild or with valid preds
        builder.addGoal(goalPred);
      } else if (Utils.isValidPredicate(goalPred, builder)) {
        builder.addGoal(goalPred);
      } else {
        log.warn("Invalid goal predicate: " + goalPred);
      }
    }
  }

  /**
   * Adds requirement declarations to the builder.
   */
  private void generateRequirements() {
    //todo: quantified conditions, conditional effects
    builder.addRequirement("strips").addRequirement("typing").addRequirement("fluents").addRequirement("negative-preconditions").addRequirement("equality").addRequirement("derived-predicates");
  }

  private void generateRules(List<Pair<Term, List<Term>>> rules) {
//        constraintViolation(agent, physical) :- not(owned(physical,agent)), holding(physical).

    for (Pair<Term, List<Term>> r : rules) {
      if (!r.getLeft().getName().contains("Violation")) {
        continue;
      }
      Action.Builder actionBuilder = new Action.Builder(r.getLeft().getName() + "Constraint");
      actionBuilder.setIsEvent(true);

      Map<String, String> varTypeBinding = new HashMap<>();
      for (Term cond : r.getRight()) {
        if (cond.getName().equals("type")) {
          varTypeBinding.put(cond.getArgs().get(0).toString(), cond.getArgs().get(1).toString());
          actionBuilder.addParameter("?" + cond.getArgs().get(0), cond.getArgs().get(1).toString());
        }
      }

      //Add preconds
      //should be of the form not(owned(?physical, ?agent))
      for (Term cond : r.getRight()) {
        if (cond.getName().equals("type")) {
          continue;
        }

        boolean negated = false;
        if (cond.isNegated()) {
          negated = true;
          cond = cond.toUnnegatedForm();
        }
        String name = cond.getName();
        List<Symbol> args = new ArrayList<>();
        cond.getArgs().forEach(arg -> {
          Symbol typedArg = Factory.createSymbol("?" + arg.toString(), varTypeBinding.get(arg.toString()));
          args.add(typedArg);
        });
        Predicate precond = Factory.createPredicate(name, args);
        builder.addPredicate(precond);
        if (negated) {
          precond = precond.toNegatedForm();
        }
        actionBuilder.addPrecondition(precond);
      }

      //Add effects
      List<Symbol> args = new ArrayList<>();
      r.getLeft().getArgs().forEach(arg -> {
        Symbol typedArg = Factory.createSymbol("?" + arg.toString(), varTypeBinding.get(arg.toString()));
        args.add(typedArg);
      });

      Predicate newEffect = Factory.createPredicate(r.getLeft().getName(), args);
      actionBuilder.addEffect(newEffect);
      actionBuilder.addEffect(Factory.createPredicate("violation"));
      builder.addPredicate(newEffect);
      builder.addEvent(actionBuilder.build());
    }
  }

  /**
   * Goes through belief to add relevant data to the PDDL builder.
   *
   * @param facts set of facts
   */
    private void scrapeBelief(List<? extends Term> facts, Symbol actor) {

    for (Term term : facts) {
      String name = term.getName();
      List<Symbol> args = term.getArgs();

      switch (name) {
        case "subtype": {
          if (args.size() != 2) {
            log.warn("Ignoring subtype. Invalid form: " + term);
            continue;
          }
          typeNames.add(args.get(0).getName());
          typeNames.add(args.get(1).getName());
          builder.addType(args.get(0).getName(), args.get(1).getName());
          break;
        }
        case "object": {
          if (args.size() != 2) {
            log.warn("Ignoring object. Invalid form: " + term);
          }

          String object = args.get(0).getName().replaceAll("\"", ""); // remove surrounding quotes which might have been added by belief/prolog
          String type = args.get(1).getName();
          builder.addObject(object, type);
          break;
        }
        case "constant": {
          if (args.size() != 2) {
            log.warn("Ignoring constant. Invalid form: " + term);
          }
          builder.addConstant(args.get(0).getName(), args.get(1).getName());
          break;
        }
        case "function": {
          // repackage function: function(functionName, functionArgs...)
          String functionName = args.remove(0).getName();
          builder.addFunction(functionName, args);
          break;
        }
        case "predicate": {
          String predicateName = args.remove(0).getName();
          builder.addPredicate(predicateName, args);
          break;
        }
        case "derived": {
          Predicate head = (Predicate) args.remove(0);
          List<Predicate> body = new ArrayList<>();
          for (Symbol arg : args) {
            if (arg.isPredicate()) {
              body.add((Predicate) arg);
            } //todo, if not, exit
          }
          builder.addDerived(head, body);
          break;
        }
      }
    }
  }

  /**
   * Check all consultants and add relevant knowledge to the PDDL builder.
   */
  private void scrapeConsultants(Symbol actor) {
    //brad: for every new Resolver get list of consultants(as TSI+ kbName) and property cache (call get properties handled on everything)

    //If an agent is busy, remove it from available agents
    Collection<TRADEServiceInfo> agentServices = TRADE.getAvailableServices(new TRADEServiceConstraints().name("isBusy"));

    for (TRADEServiceInfo agentService : agentServices) {
      try {
        if ( agentService.call(Boolean.class)) {
          for (String group : agentService.getGroups()) {
            log.debug(group + " is busy");
            ignoredAgents.add(Factory.createSymbol(group.split(":")[1]));
          }
        }
      } catch (TRADEException e) {
        log.error("Exception calling busy service", e);
      }
    }

    Set<TRADEServiceInfo> relevantInitialDomainServices = new HashSet<>();
    //logic for determining which consultants (from members of the team) should be included in PDDL problem generation
    try {
      Set<String> possibleGroups = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllDiarcAgentsForActor").argTypes(Symbol.class,Boolean.class)).call(Set.class, actor, true);

      //Iterates through all consultants (vision, location, etc) to generate observable objects
      Collection<TRADEServiceInfo> initialDomainServices = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getInitialDomain"));

      for (TRADEServiceInfo tsi : initialDomainServices) {
        if (tsi.getGroups().isEmpty() || tsi.getGroups().stream().anyMatch(possibleGroups::contains)) {
          relevantInitialDomainServices.add(tsi);
        }
      }
    } catch (TRADEException e) {
      log.error("[scrapeConsultants] error making TRADE call to getAllDiarcAgentsForActor", e);
    }

    List<Predicate> possibleAdditionsToInit = new ArrayList<>();
    for (TRADEServiceInfo consultantService : relevantInitialDomainServices) {
      try {
        List<Symbol> refIds = consultantService.call(List.class, new ArrayList<>());
        Collection<String> relevantGroups=consultantService.getGroups();

        String kbname = TRADE.getAvailableService(new TRADEServiceConstraints().name("getKBName").inGroups(relevantGroups.toArray(new String[0]))).call(String.class);

        for (Symbol refId : refIds) {
          List<Term> properties = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAssertedProperties").inGroups(relevantGroups.toArray(new String[0]))).call(List.class,refId);

          if (typeNames.contains(kbname)) {
            builder.addObject(refId.getName(), kbname);
            for (Term property : properties) {
              Predicate propertyPred = Factory.createPredicate(property.getName(),property.getArgs());
              if (propertyPred.size() > 1) {
                if (Utilities.isFullyTyped(propertyPred)) {
                  builder.addPredicate(propertyPred);
                } else {
                  log.debug("Ignoring property from planning domain: " + propertyPred);
                  continue;
                }
                // properties with 2 or more args (e.g., relational property)
                Map<Variable, Symbol> map = new HashMap<>();
                map.put((Variable) propertyPred.get(0), refId); // this assumes the first arg is the variable representing the refId
                Predicate p = propertyPred.copyWithNewBindings(map); // replace variable with refId
                builder.addInit(p);
              } else {
                builder.addInit(Factory.createPredicate("property_of", refId.getName(), propertyPred.getName()));

                // add property to pddl as a constant so it can be used explicitly (e.g., in a precondition property_of(?x, locked))
                // where "locked" is a property
                // this can clash with pddl objects if they're defined with a semantic type other than
                // "property", so we check if this property has already been defined as an object
                if (builder.getProblem().getObject(propertyPred.getName()) == null) {
                  builder.addConstant(propertyPred.getName(), "property");
                }
              }
            }
          }
        }
      } catch (TRADEException e) {
        log.error("Exception getting info from consultant: " + consultantService, e);
      }
    }
  }

  /**
   * Make PDDL actions from ASL action by scraping the action Database.
   */
  private void scrapeActionDatabase(ActionConstraints constraints) {
    for (ActionDBEntry action : constraints.filterByConstraints(Database.getActionDB().getAllActions())) {
      if (Utils.hasValidEffect(action, builder)) {
        Action.Builder actionBuilder = new Action.Builder(action.getName());

        // if the body of the ASL action is empty, assume it's defining a pddl event
        if (action.isScript() && action.getEventSpecs().isEmpty()) {
          // action is actually a pddl event
          actionBuilder.setIsEvent(true);
        }

        // Add parameters
        boolean validParams = true;
        for (ActionBinding binding : action.getInputRoles()) {
          if (!binding.hasSemanticType()) {
            log.trace("No semantic type for argument: " + binding + " in action: " + action.getName());
            validParams = false;
            continue;
          }
          actionBuilder.addParameter(binding.getName(), binding.getSemanticType());
        }

        if (!validParams) {
          log.debug("Not loading action into PDDL. At least one arg does not have a semantic type: " + action.getSignature(true));
          continue;
        }

        // Add preconditions
        for (Condition preCond : action.getPreConditions()) {

          // ignore invalid pre-conditions
          if (!Utils.isValidCondition(preCond, action, builder)) {
            log.debug("Ignoring invalid condition: " + preCond);
            continue;
          }

          // generate semantically typed predicates in case pre-condition contains existentially quantified
          // free-variable so that it can be added to action input parameters
          Set<Predicate> typedPredicates = new HashSet<>();
          preCond.getPredicates().keySet()
                  .forEach(predicate -> typedPredicates.add(new Predicate(action.applySemanticTypes(predicate))));
          typedPredicates.forEach(typedPredicate -> builder.addPredicate(typedPredicate));
          actionBuilder.addPrecondition(typedPredicates);
        }


        //Apply actor constraints
        //todo: This is a hack for now, but could the concept can be useful.
        //todo: Make it an or statement for multiple allowed actors
        for (String actor : constraints.getAllowedActors()) {
          actionBuilder.addPrecondition(Factory.createPredicate("equals(?actor," + actor + ")"));
        }

        // Add effects
        List<Effect> effects = action.getPostConditions();
        for (Effect effect : effects) {

          // ignore invalid effects
          if (!Utils.isValidEffect(effect, action, builder)) {
            log.debug("Ignoring invalid effect: " + effect);
            continue;
          }

          Predicate predicate = effect.getPredicate();
          Predicate typedPredicate = new Predicate(action.applySemanticTypes(predicate));
          builder.addPredicate(typedPredicate);
          actionBuilder.addEffect(typedPredicate);
        }

        // add action or event
        Action pddlAction = actionBuilder.build();
        if (pddlAction.isEvent()) {
          builder.addEvent(pddlAction);
        } else {
          builder.addAction(pddlAction);
        }
      }
    }
  }

  /**
   * Add :init fact to PDDL by getting all facts from Belief.
   *
   * @param facts
   * @param actor
   */
  private void generateInit(List<? extends Term> facts, Symbol actor) {

    for (Term fact : facts) {
      List<Symbol> factArgs = fact.getArgs();
      //todo: we shouldn't just be ignoring all facts related to agents that aren't valid actors for the plan. should have a separate precondition that is injected to make them invalid
      if (Collections.disjoint(ignoredAgents, factArgs)) {
        builder.addInit(Factory.createPredicate(fact.toUntypedString()));
      } else {
        log.debug("Init removed because agent disabled: " + fact);
      }
    }
  }

  /**
   * Utility method to get the set of agents known to the system which are not the actor (or in the team "actor").
   * @param actor
   * @return the set of agents which are not valid members of "actor"
   */
  private Set<Symbol> getAgentsNotInActor(Symbol actor) {
    Set<Symbol> invalidAgents = Database.getInstance().getDiarcAgents();
    Set<Symbol> validAgents = new HashSet<>();
    try {
      //todo: this should be accessible as symbols as well. should separate instances where we need strings into different functionality. is it just TRADE groups?
      Set<String> actorsStrings = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllDiarcAgentsForActor").argTypes(Symbol.class,Boolean.class)).call(Set.class, actor, false);
      actorsStrings.forEach(s -> validAgents.add(Factory.createSymbol(s)));
    } catch (TRADEException e) {
      log.error("[generateInit] error making TRADE call to getAllDiarcAgentsForActor", e);
    }
    invalidAgents.removeAll(validAgents);
    Set<Symbol> toReturn = new HashSet<>(invalidAgents);
    invalidAgents.forEach(a -> toReturn.add(Factory.createSymbol(a.getName())));
    return toReturn;
  }
}
