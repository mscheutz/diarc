/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.util.Util;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;

public class InterruptTest {
  private static Logger log = LoggerFactory.getLogger(InterruptTest.class);
  private boolean startFlag = false;
  private static GoalManagerImpl gm;

  @BeforeClass
  static public void init() {
    gm = DiarcComponent.createInstance(GoalManagerImpl.class, "-beliefinitfile agents/agents.pl", true);
  }

  @AfterClass
  static public void cleanup() {
    gm.shutdownComponent();
    gm = null;
  }

  @Before
  public void registerThis() {
    try {
      log.info("registerThis called.");
      TRADE.registerAllServices(this,new ArrayList<>());
      Util.Sleep(1000); // give time for actions to get into action DB
    } catch (TRADEException e) {
      log.error("Error registering with TRADE.", e);
    }
  }

  @After
  public void deregisterThis() {
    try {
      log.info("deregisterThis called.");
      TRADE.deregister(this);
    } catch (TRADEException e) {
      log.error("Error deregistering with TRADE.", e);
    }
  }

  @Test
  public void interruptTest() {
    long id = gm.submitGoal(Factory.createPredicate("start(self:agent)"));
    GoalStatus status = gm.joinOnGoal(id, 1000);
    Assert.assertEquals(GoalStatus.ACTIVE, status);

    gm.cancelGoal(id);
    status = gm.joinOnGoal(id);
    Assert.assertEquals(GoalStatus.CANCELED, status);
    Assert.assertFalse(startFlag);
  }

  @Test
  public void interruptWithPredicateTest() {
    long id = gm.submitGoal(Factory.createPredicate("startWithPredicate(self:agent)"));
    GoalStatus status = gm.joinOnGoal(id, 1000);
    Assert.assertEquals(GoalStatus.ACTIVE, status);

    gm.cancelGoal(id);
    status = gm.joinOnGoal(id);
    Assert.assertEquals(GoalStatus.CANCELED, status);
    Assert.assertFalse(startFlag);
  }

  @Test
  public void interruptWithStringTest() {
    long id = gm.submitGoal(Factory.createPredicate("startWithString(self:agent)"));
    GoalStatus status = gm.joinOnGoal(id, 1000);
    Assert.assertEquals(GoalStatus.ACTIVE, status);

    gm.cancelGoal(id);
    status = gm.joinOnGoal(id);
    Assert.assertEquals(GoalStatus.CANCELED, status);
    Assert.assertFalse(startFlag);
  }

  @Test
  public void interruptWithVariableTest() {
    long id = gm.submitGoal(Factory.createPredicate("startWithVariable(self:agent, val(X))"));
    GoalStatus status = gm.joinOnGoal(id, 1000);
    Assert.assertEquals(GoalStatus.ACTIVE, status);

    gm.cancelGoal(id);
    status = gm.joinOnGoal(id);
    Assert.assertEquals(GoalStatus.CANCELED, status);
    Assert.assertFalse(startFlag);
  }

  @Action
  @OnInterrupt(onCancelServiceCall = "stop()")
  @TRADEService
  public void start() {
    log.info("start called.");
    startFlag = true;
    while (startFlag) {
      Util.Sleep(100);
    }
  }

  @TRADEService
  public void stop() {
    log.info("stop called.");
    startFlag = false;
  }

  @Action
  @OnInterrupt(onCancelServiceCall = "stopWithPredicate(val(X))")
  @TRADEService
  public void startWithPredicate() {
    log.info("startWithPredicate called.");
    startFlag = true;
    while (startFlag) {
      Util.Sleep(100);
    }
  }

  @TRADEService
  public void stopWithPredicate(Predicate predicate) {
    log.info("stopWithPredicate called: {}", predicate);
    startFlag = false;
  }

  @Action
  @OnInterrupt(onCancelServiceCall = "stopWithString(\"value\")")
  @TRADEService
  public void startWithString() {
    log.info("startWithString called.");
    startFlag = true;
    while (startFlag) {
      Util.Sleep(100);
    }
  }

  @TRADEService
  public void stopWithString(String string) {
    log.info("stopWithString called: {}", string);
    startFlag = false;
  }

  @Action
  @OnInterrupt(onCancelServiceCall = "stopWithVariable(?input)")
  @TRADEService
  public void startWithVariable(Predicate input) {
    log.info("startWithVariable called.");
    startFlag = true;
    while (startFlag) {
      Util.Sleep(100);
    }
  }

  @TRADEService
  public void stopWithVariable(Predicate predicate) {
    log.info("stopWithVariable called: {}", predicate);
    startFlag = false;
  }
}
