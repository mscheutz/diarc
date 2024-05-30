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

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.*;

/**
 * This class contains hacks specific to the FetchIt! challenge, including:
 * moving an object above a specific part of the caddy
 * picking up the caddy with a specific orientation and z offset
 */
public class FetchItComponent extends FetchComponent implements FetchItInterface {

  @Override
  public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
    log.debug("[moveTo(group,refId,constraints)] method entered with constraints: " + constraints);

    MemoryObject object = getMemoryObject(refId);
    if (object == null) {
      return new ConditionJustification(false, Factory.createPredicate("have(memoryObject)"));
    }

    List<Grasp> graspOptions = null;
    if (object.getDescriptors().stream().anyMatch(des -> des.getName().equals("caddy") || des.getName().equals("medicalcaddy"))) {
      Grasp grasp = new Grasp();
      grasp.setPoint(0, object.getLocation().x, object.getLocation().y, 0.97);
      grasp.setOrientation(-0.7018492426773312, 0.08819182984701168, 0.7013116255313993, 0.08827143136937808);
      graspOptions.add(grasp);
    } else {
      // call super class method
      graspOptions = getOrderedGraspOptions(refId, constraints);
    }

    return moveToGraspOption(groupName, graspOptions);
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
