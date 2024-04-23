/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.norms;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.belief.provers.DCEC;
import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.belief.provers.clingo.ClingoProver;
import edu.tufts.hrilab.belief.provers.prolog.Prolog;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;

import java.io.IOException;
import java.util.*;


/**
 * This class is VERY MUCH still a work in progress.
 */
//todo: Create an abstract class, with specific implementations for different ways to reason
// about norms
public class NormComponent extends DiarcComponent {

  Prover prover;
  Prover.Engine proverType = Prover.Engine.PROLOG;
  List<String> initfiles;

  public NormComponent() {
    try {
      prover.initializeFromFiles(initfiles);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  protected void init() {
    prover = createProver(proverType);
    initfiles = new ArrayList<>();
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("initfile")) {
      initfiles.addAll(Arrays.asList(cmdLine.getOptionValues("initfile")));
    }
    if (cmdLine.hasOption("prover")) {
      proverType = Prover.Engine.valueOf(cmdLine.getOptionValue("prover"));
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {

    List<Option> options = new ArrayList<>();

    options.add(Option.builder("initfile").hasArgs().argName("file").desc("Initialization files").build());
    options.add(Option.builder("prover").hasArg().argName("prover").desc("Prover type").build());

    return options;
  }

  private Prover createProver(Prover.Engine engine) {
    // Instantiate prover
    switch (engine) {
      case PROLOG:
        return new Prolog();
      case DCEC:
        return new DCEC();
      case CLINGO:
        return new ClingoProver();
      default:
        log.error("Cannot instantiate prover " + proverType + ". Defaulting to prolog.");
        return new Prolog();
    }
  }


  public boolean assertNorm(String norm) {
    prover.assertRule(norm);
    Term head = null;
    List<Term> body = null;
    //todo: Maybe auto assert the types?
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertRule").argTypes(Term.class,List.class)).call(void.class, head, body);
      return true;
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Given a failed state, get all constraints that are violated
   * @param actor
   * @param stateMachine
   * @return
   */
  @TRADEService
  @Action
  public Justification getViolations(Symbol actor, StateMachine stateMachine) {
    //todo: instead of state machine, get from prover
    for (Pair<Term, List<Term>> rule : stateMachine.getRules()) {

      //Find all constraint rules (hardcoded for now)
      if (!rule.getLeft().getName().contains("Violation")) {
        continue;
      }

      //Manually insert actor
      Map<Variable, Symbol> actorBinding = new HashMap<>();
      actorBinding.put(Factory.createVariable("X"), actor);
      Predicate query = (Predicate) rule.getLeft().copyWithNewBindings(actorBinding);

      //Test constraint
      Justification potentialViolation = stateMachine.holds(query);
      if (potentialViolation.getValue()) {
        potentialViolation.setValue(false); //have to flip it since we want the query to fail if
        // true
        return potentialViolation;
      }
    }
    return new ConditionJustification(true);
  }

  /**
   * WIP action for the system to ask a superior if it can relax any of its norms
   * @param actor
   * @param options
   */
  @Action
  @TRADEService
  public void negotiateConstraints(Symbol actor, Predicate options) {
    //todo: Send dialogue to LLM component about options. Some example outputs should be:
    //sayText("I can complete this task if I can touch things that aren't mine")
    //sayText("I can complete this task if I can touch blue things, or I can touch things that
    // aren't mine")
    //todo: Wait for response. These should be parsed to strings/symbols/ predicates somewhere in
    // the LLM component
    // *before* sending them back here
    //Okay, this time you can touch things that aren't yours -> ownershipViolation


    //translate response to one of the predicate options
    String response = "ownershipViolation";

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("retractRule")).call(void.class, response);
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }
  }

}
