/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.ros2.Ros2Factory;
import edu.tufts.hrilab.ros2.Ros2Node;
import edu.tufts.hrilab.vision.consultant.VisionReference;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.*;
import java.util.*;

//todo: instead, extend Ros2MoveBaseComponent?
public class ForkliftPython extends DiarcComponent {

  private Ros2Node subNode;
  private PoseConsultant consultant;
  private AITVisionConsultant visConsultant;
  public boolean ready = false;

  //todo: load from config?
  private final String mapDescriptor = "location";
  private final String visDescriptor = "physobj";

  @Override
  protected void init() {
    //Setup pose consultant
    consultant = new PoseConsultant(PoseReference.class, mapDescriptor, new ArrayList<>());
    visConsultant = new AITVisionConsultant(VisionReference.class, "physobj");

    try {
      TRADE.registerAllServices(consultant, this.getMyGroups());
      TRADE.registerAllServices(visConsultant, this.getMyGroups());
    } catch (TRADEException e) {
      log.error("Error registering pose consultant.", e);
    }

    subNode = Ros2Factory.getDefaultNode();
    subNode.spin(100);
  }


  public ForkliftPython() {
    super();
  }

  private Pair<Point3d, Quat4d> getLocationPose(Symbol location) {
    Pair<Point3d, Quat4d> pose = null;
    if (!location.isTerm() && location.getName().startsWith(consultant.getKBName())) {
      // location is a reference
      PoseReference poseReference = consultant.getReference(location);
      if (poseReference != null && poseReference.hasPose()) {
        pose = poseReference.getPose();
      }
    }
    return pose;
  }

  // Populates belief with the distances between zones, e.g. fluent_equals(dist(zone1, zone2), 5)
  private void calculateDistances() {
    List<Term> properties = new ArrayList<>();
    Set<Term> beliefs = new HashSet<>();
    properties.add(Factory.createPredicate("loadingzone", Factory.createVariable("VAR0", mapDescriptor)));
    properties.add(Factory.createPredicate("truck", Factory.createVariable("VAR0", mapDescriptor)));
    properties.add(Factory.createPredicate("location", Factory.createVariable("VAR0", mapDescriptor)));
    List<Symbol> refs = consultant.getReferencesWithAnyProperties(properties);
    for (Symbol ref1 : refs) {
      for (Symbol ref2 : refs) {
        if (ref1 != ref2) {
          long dist = consultant.calculateDistance(ref1, ref2);
          Predicate dist_pred = Factory.createPredicate("distance", ref1, ref2);
          beliefs.add(Factory.createPredicate("fluent_equals", dist_pred, Factory.createSymbol(String.valueOf(dist))));
        }
      }
    }

    while (TRADE.getAvailableServices(new TRADEServiceConstraints().name("assertBeliefs")).isEmpty()) {
      try {
        Thread.sleep(500);
      } catch (InterruptedException e) {
        throw new RuntimeException(e);
      }
    }

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, beliefs);
    } catch (TRADEException e) {
      log.error("[calculateDistances]",e);
    }

  }

  // Populates belief with the weights of objects, e.g. fluent_equals(weight(palette1), 5)
  private void readWeights() {

  }

  //FIXME: Mock class until I know what the actual version looks like
  private class Area {
    String name;
    int slots;
    int pallets;
    int x;
    int y;
  }

  // FIXME: This needs to change once we know what the area object looks like
  @TRADEService
  @Action
  public void initAreas() {
    List<Area> areas = getAreas();
    Set<Term> beliefs = new HashSet<>();
    for (Area area : areas) {

      // Create location reference for each area
      List<Term> properties = new ArrayList<>();
      if (area.name.contains("loadingzone")) {
        properties.add(Factory.createPredicate("loadingzone", Factory.createVariable("VAR0", mapDescriptor)));
      } else if (area.name.contains("truck")) {
        properties.add(Factory.createPredicate("truck", Factory.createVariable("VAR0", mapDescriptor)));
      }

      PoseReference location = consultant.createReference(Factory.createVariable("VAR0", mapDescriptor), properties);
      Point3d pt = new Point3d(area.x, area.y, 0);
      Quat4d qt = new Quat4d(0, 0, 0, 1);
      location.setPose(pt, qt);

      beliefs.add(Factory.createPredicate("fluent_equals",
              Factory.createPredicate("current_weight", location.refId),
              Factory.createSymbol(String.valueOf(0))
      ));
      beliefs.add(Factory.createPredicate("fluent_equals",
              Factory.createPredicate("slots", location.refId),
              Factory.createSymbol(String.valueOf(area.slots - area.pallets))
      ));

      for (int i = 0; i < area.pallets; i++) {
        VisionReference pallet = visConsultant.createReference(Factory.createVariable("VAR0", visDescriptor), new ArrayList<>());
        beliefs.add(Factory.createPredicate("at", pallet.refId, location.refId));
      }
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, beliefs);
    } catch (TRADEException e) {
     log.error("[initAreas]",e);
    }

    calculateDistances();
  }


  @Action
  @TRADEService
  public Predicate createWeightGoal(Symbol location) {
    List<Symbol> pallets = new ArrayList<>(visConsultant.getActivatedEntities().keySet());
    List<Integer> weights = new ArrayList<>();

    Variable x = Factory.createVariable("X");
    for (Symbol pallet : pallets) {
      try {
        List<Map<Variable, Symbol>> bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class)).call(List.class, Factory.createPredicate("fluent_equals",
                Factory.createPredicate("weight", pallet), x));
        Symbol val = bindings.get(0).get(x);
        weights.add(Float.valueOf(val.getName()).intValue());
      } catch (TRADEException e) {
        throw new RuntimeException(e);
      }
    }

    List<Integer> indices = maximizeWeight(weights.stream().mapToInt(i -> i).toArray(), 150, 3);
    for (Integer index : indices) {
      log.info("Pallet " + pallets.get(index) + " with weight " + weights.get(index));
    }
    List<Predicate> goals = new ArrayList<>();
    for (Integer index : indices) {
      goals.add(Factory.createPredicate("at", pallets.get(index), location));
    }
    return Factory.createPredicate("and", goals);
  }

  private List<Integer> maximizeWeight(int[] weights, int capacity, int slots) {
    int[][][] dp = new int[weights.length + 1][capacity + 1][slots + 1];

    for (int n = 1; n <= weights.length; n++) {
      for (int w = 1; w <= capacity; w++) {
        for (int s = 1; s <= slots; s++) {
          if (weights[n - 1] > w) {
            dp[n][w][s] = dp[n - 1][w][s];
          } else {
            dp[n][w][s] = Math.max(dp[n - 1][w][s], dp[n - 1][w - weights[n - 1]][s - 1] + weights[n - 1]);
          }
        }
      }
    }

    log.debug("Items selected:");
    List<Integer> indeces = new ArrayList<>();
    int w = capacity;
    int s = slots;
    for (int n = weights.length; n > 0; n--) {
      if (dp[n][w][s] != dp[n - 1][w][s]) {
        indeces.add(n - 1);
        w -= weights[n - 1];
        s--;
      }
    }
    return indeces;

  }

  //TODO: Change to ROS call
  private List<Area> getAreas() {
    List<Area> areas = new ArrayList<>();
    Area zonea = new Area();
    zonea.name = "loadingzone1";
    zonea.slots = 5;
    zonea.pallets = 3;
    zonea.x = 0;
    zonea.y = 0;
    areas.add(zonea);

    Area zoneb = new Area();
    zoneb.name = "loadingzone2";
    zoneb.slots = 5;
    zoneb.pallets = 3;
    zoneb.x = 5;
    zoneb.y = 5;
    areas.add(zoneb);

    Area truck = new Area();
    truck.name = "truck1";
    truck.slots = 3;
    truck.pallets = 0;
    truck.x = 5;
    truck.y = 0;
    areas.add(truck);

    return areas;
  }

}
