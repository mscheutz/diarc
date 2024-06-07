/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.vision.stm.MemoryObjectUtil;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.*;

/**
 * This class contains hacks specific to the FetchIt! challenge, including:
 * moving an object above a specific part of the caddy
 * picking up the caddy with a specific orientation and z offset
 */
public class FetchItComponent extends FetchComponent implements FetchItInterface {

  /**
   * Override to handle special cases for "caddy" and "medical caddy".
   *
   * @param groupName
   * @param refId
   * @param constraints a list of predicate constraints
   * @return
   */
  @Override
  public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
    log.debug("[moveTo(group,refId,constraints)] method entered with constraints: " + constraints);

    List<Grasp> graspOptions = null;
    try {
      //this is pretty messy
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class, List.class));
      graspOptions = Arrays.asList(tsi.call(Grasp[].class, refId, Grasp[].class, constraints));
    } catch (TRADEException e) {
      log.error("[moveTo] exception getting memory object from reference, returning null", e);
      return new ConditionJustification(false, Factory.createPredicate("has", refId, Factory.createSymbol("graspOptions")));
    }

    // Prioritize grasp points from a specific angle. Can be set from JSON, defaults to vertical.
    graspOptions.sort((grasp0, grasp1) -> {
      double angleA = MemoryObjectUtil.getAngleFromAngle(grasp0, moveItConfig.graspAngle);
      double angleB = MemoryObjectUtil.getAngleFromAngle(grasp1, moveItConfig.graspAngle);
      if (angleA == angleB)
        return 0;
      else
        return (angleA < angleB) ? -1 : 1;
    });

    // turn on (or update if it's already on) collision avoidance
    enableCollisionAvoidance();
    // iterate through grasp options until one works, or we're out of options
    boolean approachSuccess = false;
    int graspIndex = 0;
    while (!approachSuccess && (graspIndex < graspOptions.size())) {
      // pick a grasp option
      log.debug("Grasp index: " + graspIndex);

      // convert grasp MemoryObject option to Grasp
      Grasp grasp = graspOptions.get(graspIndex++);
      MemoryObject object = getMemoryObject(refId);
      if (object == null) {
        return new ConditionJustification(false);
      }
      if (object.getDescriptors().stream().anyMatch(des -> des.getName().equals("caddy") || des.getName().equals("medicalcaddy"))) {
        for (int i = 0; i < grasp.getNumPoints(); i++) {
          Vector3d point = grasp.getPoint(0);
          point.x = object.getLocation().x;
          point.y = object.getLocation().y;
          point.z = 0.97;
          grasp.setOrientation(i, -0.7018492426773312, 0.08819182984701168, 0.7013116255313993, 0.08827143136937808);
        }
      }

      // doing some basic grasp checks
      if (grasp.getNumPoints() == 0) {
        log.error("[moveTo(group,object)] failed to find a grasp.");
        return new ConditionJustification(false);
      }
      if (grasp.getType() == null) {
        log.error("[moveTo(group,object)] grasp type null!");
        return new ConditionJustification(false);
      }

      // try moving to the grasp (first stage of approach)
      // first stage of moveTo: use collision avoidance to get close to object without touching it
      if (!moveToApproach(groupName, grasp, moveItConfig.graspApproachOffset)) {
        log.warn("First stage of moveTo failed. GraspIndex: " + graspIndex);
        continue;
      }

      // second stage of moveTo: turn off collision avoidance and move into position, either touching
      // the object or in position to close gripper on the object
      // This allows us to avoid a situation where the gripper complains about being planned into an object,
      // and allows us to deal with potential momentum drift.
      log.debug("Final approach to object");

      // TODO: this is where we should be making the object an allowed collision object using the AllowedCollisionMatrix!
      // EAK: setObjectCollisions(Long.toString(object.getTokenId() * 2 + 1), true);
      // setObjectCollisions(Long.toString(object.getTokenId() - 1),false);

      // completely disable all collision avoidance to allow grippers to not complain about being planned into an object
      if (moveItConfig.allowDisableCollisionAvoidance) {
        disableCollisionAvoidance();
      }

      // try moving to the grasp (final approach)
      if (moveToApproach(groupName, grasp, moveItConfig.graspContactOffset)) {
        approachSuccess = true;
      } else {
        log.warn("Final stage of moveTo failed. GraspIndex: " + graspIndex);
      }

      // (re)enable collision avoidance
      if (moveItConfig.allowDisableCollisionAvoidance) {
        enableCollisionAvoidance();
      }
    }

    if (!approachSuccess) {
      log.debug("[moveTo(group,refId)] failed.");
      return new ConditionJustification(false);
    }

    log.debug("[moveTo(group,refId)] success!");
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveObjectAbove(Symbol objectRef_0, Symbol objectRef_1, String groupName) {
    Point3d point;
    double zOff = 1.2;
    Map<String, Point3d> offsets = new HashMap<>();
    offsets.put("screw", new Point3d(.04, -0.05, zOff));
    offsets.put("smallgear", new Point3d(.04, -0.05, zOff));
    offsets.put("largegear", new Point3d(-.075, -0.04, zOff));
    offsets.put("gearboxtop", new Point3d(.00, 0.04, zOff));
    offsets.put("gearboxbottom", new Point3d(.00, 0.04, zOff));
    offsets.put("antiseptic", new Point3d(.04, -0.05, zOff));
    offsets.put("painkiller", new Point3d(-.075, -0.04, zOff));
    offsets.put("bandagebox", new Point3d(.00, 0.04, zOff));
    offsets.put("caddy", new Point3d(0.0, 0.0, 1.3));
    offsets.put("medicalcaddy", new Point3d(-0.075, 0.0, 1.15));

    String obj0 = getDominantProperty(objectRef_0, offsets.keySet());
    String obj1 = getDominantProperty(objectRef_1, offsets.keySet());
    if (obj1.equals("caddy") || obj1.equals("medicalcaddy")) {
      Point3d offset = offsets.get(obj0);
      MemoryObject mo = getMemoryObject(objectRef_1);
      Point3d caddy = mo.getLocation();
      point = new Point3d(caddy.getX() + offset.getX(), caddy.getY() + offset.getY(), offset.getZ());
    } else if (obj0.equals("caddy") || obj0.equals("medicalcaddy")) {
      Point3d offset = offsets.get(obj0);
      MemoryObject mo = getMemoryObject(objectRef_1);
      Point3d obj2Point = mo.getLocation();
      point = new Point3d(obj2Point.getX() + offset.getX(), obj2Point.getY() + offset.getY(), offset.getZ());
      //point = offsets.get("caddy");
    } else {
      log.warn("don't know where to move object " + objectRef_0);
      Predicate action = Factory.createPredicate("moveObjectAbove", objectRef_0, objectRef_1);
      return new ConditionJustification(false, Factory.createPredicate("succeeded", action));
    }
    Quat4d orientation = new Quat4d(0.7, 0.03, -0.7, 0.03);
    lookAround();
    boolean moved = moveTo(groupName, point, orientation);
    return new ConditionJustification(moved);
  }

  private String getDominantProperty(Symbol objectRef, Set<String> knownObjects) {
    try {
      Collection<TRADEServiceInfo> tsis = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class));
      for (TRADEServiceInfo tsi : tsis) {
        List<Term> properties = tsi.call(List.class, objectRef);
        for (Term property : properties) {
          String propertyName = property.getName();
          if (knownObjects.contains(propertyName)) {
            return propertyName;
          }
        }

      }
    } catch (TRADEException e) {
      log.error("Error calling getProperties for reference: " + objectRef, e);
    }
    return objectRef.getName();
  }

  protected MemoryObject getMemoryObject(Symbol objectRef) {
    MemoryObject mo = null;
    log.debug("getting Memory Object for " + objectRef);
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      mo = tsi.call(MemoryObject.class, objectRef, MemoryObject.class);
    } catch (TRADEException e) {
      log.error("[getMemoryObject] exception getting memory object from reference, returning null", e);
    }

    return mo;
  }
}
