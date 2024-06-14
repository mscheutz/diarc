/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.moveit.MockMoveItComponent;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import javax.vecmath.Point3d;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.ArrayList;

public class MockFetchComponent extends MockMoveItComponent implements FetchInterface {

  public MockFetchComponent() {
    super();
  }

  @Override
  public boolean setTorsoPosition(double position) {
    log.info("[setTorsoPosition]");
    simExecTime();
    return true;
  }

  @Override
  public boolean pointHeadTo(MemoryObject target_object) {
    log.info("[pointHeadTo memObj]");
    simExecTime();
    return true;
  }

  @Override
  public boolean pointHeadTo(Symbol objectRef) {
    log.info("[pointHeadTo objRef]");
    simExecTime();
    return true;
  }

  @Override
  public boolean pointHeadTo(Point3d target_point) {
    log.info("[pointHeadTo point]");
    simExecTime();
    return true;
  }

  @Override
  public Justification look(String direction) {
    log.info("[look]");
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification lookAround() {
    log.info("[lookAround]");
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public List<Map<Variable, Symbol>> checkGrasping(Term graspingTerm) {
    log.info("[checkGrasping] " + graspingTerm);
    simExecTime();
    List<Map<Variable,Symbol>> toReturn = new ArrayList<>();
    if (grasping) {
      toReturn.add(new HashMap<>());
    }
    return toReturn;
  }

  public float getChargeLevel() {
    log.info("[getChargeLevel]");
    simExecTime();
    return 1;
  }

  public FetchBreakerStates getBreakerState(String breakerName) {
    log.info("[getBreakerState]");
    simExecTime();
    return FetchBreakerStates.STATE_ENABLED;
  }

  public FetchBreakerStates setBreakerState(String breakerName, boolean state) {
    log.info("[setBreakerState]");
    simExecTime();
    return FetchBreakerStates.STATE_ENABLED;
  }

  protected void simExecTime() {
    if (shouldSimExecTime) {
      try {
        Thread.sleep(simExecTimeout);
      } catch (InterruptedException e) {
      }
    }
  }
}
