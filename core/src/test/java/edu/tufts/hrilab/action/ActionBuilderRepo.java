/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Symbol;

/**
 *
 * @author willie
 */
public class ActionBuilderRepo {

  public static ActionDBEntry buildLookforAction() {
    ActionDBEntry.Builder lookforBuilder = new ActionDBEntry.Builder("lookfor");
    lookforBuilder.addRole(new ActionBinding.Builder("?loc", Symbol.class).build());
    lookforBuilder.addRole(new ActionBinding.Builder("?obj", Symbol.class).build());
//    lookforBuilder.addResourceLock("motionLock"); //put back in once locks are re-implemented
    lookforBuilder.setTimeout(1L);
    lookforBuilder.setCost("0.23");
    lookforBuilder.setBenefit("0.79");
    lookforBuilder.setMinUrg("0.0");
    lookforBuilder.setMaxUrg("1.0");
    lookforBuilder.addEffect(new Effect(Factory.createPredicate("located(?loc,?obj)"), EffectType.SUCCESS));
    lookforBuilder.addEffect(new Effect(Factory.createPredicate("located(?obj)"), EffectType.SUCCESS));
    ActionDBEntry adb = lookforBuilder.build(true);
    return adb;
  }

  public static ActionDBEntry buildGraspAction() {
    ActionDBEntry.Builder graspBuilder = new ActionDBEntry.Builder("grasp");
    graspBuilder.addRole(new ActionBinding.Builder("?obj", Symbol.class).build());
//    graspBuilder.addResourceLock("motionLock");  //put back in once locks are re-implemented
    graspBuilder.setTimeout(1L);
    graspBuilder.setCost("9.23");
    graspBuilder.setBenefit("0.79");
    graspBuilder.setMinUrg("0.0");
    graspBuilder.setMaxUrg("1.0");
    graspBuilder.addEffect(new Effect(Factory.createPredicate("grasped(?obj)"), EffectType.SUCCESS));
    graspBuilder.addEffect(new Effect(Factory.createPredicate("touching(?actor,?obj)"), EffectType.SUCCESS));
    ActionDBEntry adb = graspBuilder.build(true);
    return adb;
  }


  public static ActionDBEntry buildMoveToObjectAction() {
    ActionDBEntry.Builder moveToObjBuilder = new ActionDBEntry.Builder("moveToObject");
    moveToObjBuilder.addRole(new ActionBinding.Builder("?obj", Symbol.class).build());
//    moveToObjBuilder.addResourceLock("motionLock");  //put back in once locks are re-implemented
    moveToObjBuilder.setTimeout(1L);
    moveToObjBuilder.setCost("8.23");
    moveToObjBuilder.setBenefit("19.79");
    moveToObjBuilder.setMinUrg("0.0");
    moveToObjBuilder.setMaxUrg("1.0");
    Predicate pre = Factory.createPredicate("located(?obj)");
    moveToObjBuilder.addCondition(new Condition(pre, ConditionType.PRE));
    moveToObjBuilder.addEffect(new Effect(Factory.createPredicate("movedTo(?obj)"), EffectType.SUCCESS));
    ActionDBEntry adb = moveToObjBuilder.build(true);
    return adb;
  }

  public static ActionDBEntry buildObsTouching() {
    ActionDBEntry.Builder touchingObsBuilder = new ActionDBEntry.Builder("observerTouching");
    touchingObsBuilder.addRole(new ActionBinding.Builder("?predicate", Predicate.class).build());
    touchingObsBuilder.addRole(new ActionBinding.Builder("!x", edu.tufts.hrilab.fol.Symbol.class).setIsLocal(true).build());
    touchingObsBuilder.addRole(new ActionBinding.Builder("!y", edu.tufts.hrilab.fol.Symbol.class).setIsLocal(true).build());
    touchingObsBuilder.addRole(new ActionBinding.Builder("ret", java.util.List.class).setIsReturn(true).build());
//    touchingObsBuilder.addResourceLock("motionLock");  //put back in once locks are re-implemented
    touchingObsBuilder.setTimeout(1L);
    touchingObsBuilder.setCost("8.23");
    touchingObsBuilder.setBenefit("19.79");
    touchingObsBuilder.setMinUrg("0.0");
    touchingObsBuilder.setMaxUrg("1.0");
    Predicate observes = Factory.createPredicate("touching(!x,!y)");
    touchingObsBuilder.addObservation(observes);
    ActionDBEntry adb = touchingObsBuilder.build(true);
    return adb;
  }

}
