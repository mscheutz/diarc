/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.asl;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Helper class to expose various TRADEServices for testing ASL tsc calls.
 */
public class TestHelper {
  private static Logger log = LoggerFactory.getLogger(TestHelper.class);

  /**
   * Because this helper class lives in the action test package, it automatically
   * gets included in the action tests, and fails if there's not at least one @Test.
   * TODO: is there a better way to handle this?
   */
  @Test
  public void doNothing() {

  }

  @TRADEService
  public void tsc1() {
    log.info("tsc1");
  }

  @TRADEService
  public void tsc2(Symbol arg) {
    log.info("tsc2 arg: {}", arg);
  }

  @TRADEService
  public void tsc3(Symbol actor, Symbol arg) {
    log.info("tsc3 actor: {} arg: {}", actor, arg);
  }

  @TRADEService
  public Justification tsc4() {
    log.info("tsc4");
    return new ConditionJustification(true, Factory.createPredicate("succeeded(tsc4)"));
  }

  @TRADEService
  public boolean tsc_bool_true() {
    log.info("tsc_bool_true");
    return true;
  }

  @TRADEService
  public boolean tsc_bool_false() {
    log.info("tsc_bool_true");
    return false;
  }

  @TRADEService
  public Justification tsc_justification_true() {
    log.info("tsc_justification_true");
    return new ConditionJustification(true);
  }

  @TRADEService
  public Justification tsc_justification_false() {
    log.info("tsc_justification_false");
    return new ConditionJustification(false);
  }

  @Action
  @TRADEService
  public boolean act_bool_true() {
    log.info("act_bool_true");
    return true;
  }

  @Action
  @TRADEService
  public boolean act_bool_false() {
    log.info("act_bool_false");
    return false;
  }

  @Action
  @TRADEService
  public Justification act_justification_true() {
    log.info("act_justification_true");
    return new ConditionJustification(true);
  }

  @Action
  @TRADEService
  public Justification act_justification_false() {
    log.info("act_justification_false");
    return new ConditionJustification(false);
  }
}
