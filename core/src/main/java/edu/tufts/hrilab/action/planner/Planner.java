/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.planner;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.planner.pddl.PddlGenerator;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.pddl.Action;
import edu.tufts.hrilab.pddl.Domain;
import edu.tufts.hrilab.pddl.Pddl;
import edu.tufts.hrilab.pddl.Problem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.util.*;

public abstract class Planner {
  protected final Logger log;

  public Planner() {
    log = LoggerFactory.getLogger(this.getClass());
  }

  /**
   * Last instantiated Pddl. Cached so that the Domain and Problem can be retrieved.
   */
  protected Pddl pddl;
  private String lastPlan = "";

  public enum Type {
    PDDL4J,
    FFPLANNER
  }

  /**
   * Register to make services available (e.g., getCurrentPlan, getDomain)
   */
  public void registerWithTRADE() {
    try {
      TRADE.registerAllServices(this,new ArrayList<>());
    } catch (TRADEException e) {
      log.error("Error registering Planner with TRADE.", e);
    }
  }

  /**
   * Only method that needs to be implemented by a class extending the Planner class.
   *
   * @return
   */
  protected abstract String plan(File domain, File problem);

  /**
   * Main access point to use any planner that extends the base Planner class.
   * Serves as both a factory method and execution method, where only the
   * resulting plan is returned.
   *
   * @param goal
   * @param stateMachine
   * @return
   */
  public final ParameterizedAction plan(edu.tufts.hrilab.action.goal.Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    PddlGenerator pddlBuilder = new PddlGenerator(goal, constraints, stateMachine.getAllFacts(), stateMachine.getRules());
    pddl = pddlBuilder.build();

    File domain = pddl.generateDomainFile();
    File problem = pddl.generateProblemFile();

    if (log.isDebugEnabled()) {
      log.debug("domain file: " + domain.getAbsolutePath());
      log.debug("problem file: " + problem.getAbsolutePath());
    }

    String plan = plan(domain, problem);
    if (plan == null) {
      log.error("Can't plan.");
      return null;
    } else {
      log.info("PLAN = " + plan);
      lastPlan = plan;
    }

    // convert string plan to a DIARC action script
    return generateActionScript(plan, goal);
  }

  /**
   * Returns a DIARC-compliant parametrized action
   */
  private ParameterizedAction generateActionScript(String plan, Goal goal) {
    if (plan == null) {
      return null;
    }

    // build object/constant -> (object/constant):type map from pddl problem's objects and domain's constants
    // NOTE: toLowerCase is needed bc results from pddl cannot be mixed case (all upper or lower so we make everything lower)
    Map<String, Symbol> objectTypeMap = new HashMap<>();
    pddl.getProblem().getObjects().forEach(object -> objectTypeMap.put(object.getName().toLowerCase(Locale.ROOT), object));
    pddl.getDomain().getConstants().forEach(constant -> objectTypeMap.put(constant.getName().toLowerCase(Locale.ROOT), constant));

    // Convert plan into a list of string actions
    ActionDBEntry.Builder newAction = new ActionDBEntry.Builder("planned");

    if(plan.isEmpty()){
      log.warn("goal state is already true, plan doesn't do anything");
    } else {
      String[] sActions = plan.split("\n"); //Actions in string form
      for (String sAction : sActions) {
        // Convert pddl output into something understandable by Action Interpreter
        String cleaned_action = parsePDDLFormat(sAction);
        cleaned_action = convertToAslAction(cleaned_action);
        Predicate actionWithActor = Factory.createPredicate(cleaned_action);

        // apply semantic types
        actionWithActor = createTypedPredicate(actionWithActor, objectTypeMap);

        newAction.addEventSpec(new EventSpec(actionWithActor));
      }
    }
    newAction.setDescription(goal.getPredicate().toString());
    ActionDBEntry newADB = newAction.build(true); //todo: this sticks a new planned action on top of the old one every time. should we be replacing it?

    return new ParameterizedAction(newADB, goal);
  }

  /**
   * Convert pddl action signature to asl action signature. This is needed because ASL and PDDL actions might have different
   * input parameters to accommodate ASL features that are unsupported in PDDL (e.g., using existentially quantified
   * pre-condition free-variables in post-conditions).
   *
   * @param pddlActionStr
   * @return
   */
  private String convertToAslAction(String pddlActionStr) {
    Predicate pddlActionPred = Factory.createPredicate(pddlActionStr);
    for (Action action : pddl.getDomain().getActions()) {
      if (pddlActionPred.instanceOf(action.getPddlSignature())) {
        Map<Variable, Symbol> bindings = action.getPddlSignature().getBindings(pddlActionPred);
        Predicate aslActionPred = action.getAslSignature().copyWithNewBindings(bindings);
        return aslActionPred.toString();
      }
    }

    // if no match found
    log.warn("No pddl action found matching action step of plan: " + pddlActionStr);
    return pddlActionStr;
  }

  /**
   * Returns a string form (e.g., "pickup(self, a)" when the input is "0: ( pickup self a) [1]"
   */
  private String parsePDDLFormat(String a) {
    int sInt;
    int eInt;
    sInt = a.indexOf("(") + 1;
    eInt = a.indexOf(")");
    String[] actionArray = a.substring(sInt, eInt).trim().split(" "); //Action in string array form
    String sName = actionArray[0]; //name of the action

    StringJoiner sj = new StringJoiner(", ", sName + "(", ")");
    for (int i = 1; i < actionArray.length; i++) {
      sj.add(actionArray[i]);
    }

    return sj.toString();
  }

  private String deSanitize(String input) {
    int startIndex = input.indexOf("QUOTE");
    StringBuilder builder = new StringBuilder();
    if (startIndex != -1) {
      int endIndex = input.indexOf("QUOTE", startIndex + 5);
      if (endIndex != -1) {
        builder.append("\"");
        builder.append(input.substring(startIndex + 5, endIndex).replaceAll("_", " "));
        builder.append(("\""));
        builder.append(input.substring(endIndex + 5));
      }
    }
    if (builder.isEmpty()) {
      return input;
    } else {
      return builder.toString();
    }
  }

  private Predicate createTypedPredicate(Predicate untypedPredicate, Map<String, Symbol> objectTypeMap) {
    List<Symbol> typedArgs = new ArrayList<>();
    for (Symbol arg : untypedPredicate.getArgs()) {
      if (objectTypeMap.containsKey(arg.getName())) {
        Symbol typedButNotSanitized=objectTypeMap.get(arg.getName());
        typedArgs.add(Factory.createSymbol(deSanitize(typedButNotSanitized.getName()), typedButNotSanitized.getType()));
      } else {
        log.warn("Unknown object. Using untyped: " + arg);
        typedArgs.add(arg);
      }
    }

    return Factory.createPredicate(untypedPredicate.getName(), typedArgs);
  }

  /**
   * Return the last PDDL Domain. Can be null.
   *
   * @return
   */
  public Domain getPddlDomain() {
    if (pddl == null) {
      return null;
    }

    return pddl.getDomain();
  }

  /**
   * Return the last PDDL Problem. Can be null.
   *
   * @return
   */
  public Problem getPddlProblem() {
    if (pddl == null) {
      return null;
    }

    return pddl.getProblem();
  }

  /**
   * Return the last generated plan. This is currently only for RapidLearn integration.
   *
   * This should really be exposed in relation to the goal that the plan is for, since in general concurrent goals can be running.
   *
   * @return
   */
  @TRADEService
  public String getCurrentPlan() {
    return lastPlan;
  }

  /**
   * Return the last generated pddl Domain. This is currently only for RapidLearn integration.
   *
   * This should really be exposed in relation to the goal that the plan is for, since in general concurrent goals can be running.
   *
   * @return
   */
  @TRADEService
  public Domain getCurrentDomain() {
    return getPddlDomain();
  }

}
