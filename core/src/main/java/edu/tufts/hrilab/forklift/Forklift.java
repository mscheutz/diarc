/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.forklift.messages.*;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;
import edu.tufts.hrilab.ros2.Ros2Factory;
import edu.tufts.hrilab.ros2.Ros2Node;
import edu.tufts.hrilab.util.RotationHelpers;
import edu.tufts.hrilab.util.SimpleGeometry;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.consultant.VisionReference;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import edu.tufts.hrilab.forklift.RosForkliftStateListener;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URI;
import java.net.URISyntaxException;

import javax.vecmath.*;
import java.util.*;

//todo: instead, extend Ros2MoveBaseComponent?
public class Forklift extends DiarcComponent implements MoveBaseInterface {

  private Ros2Node node;
  private PoseConsultant consultant;
  private AITVisionConsultant visConsultant;
  private String map_frame = "map";
  private String base_frame = "base";
  public boolean ready = false;

  //todo: load from config?
  private final String mapDescriptor = "location";
  private final String visDescriptor = "physobj";
  private final double pointDistThresh = .25;
  private final double orientationThresh = .40;

  private static final boolean wait_for_ros_topic = true;
  private static final boolean ros_cmd_as_echo = true;
  private RosForkliftStateListener forkliftStateListener;

  private Map<Symbol, Symbol> diarcToAitRefs = new HashMap<>();

  @Override
  protected void init() {
    //Setup pose consultant
    consultant = new PoseConsultant(PoseReference.class, mapDescriptor, new ArrayList<>());
    visConsultant = new AITVisionConsultant(VisionReference.class, visDescriptor);

    try {
      List<String> visConsultantGroups = this.getMyGroups();
      List<String> poseConsultantGroups = this.getMyGroups();
      visConsultantGroups.add(visDescriptor);
      poseConsultantGroups.add(mapDescriptor);
      TRADE.registerAllServices(consultant, poseConsultantGroups);
      TRADE.registerAllServices(visConsultant, visConsultantGroups);
    } catch (TRADEException e) {
      log.error("Error registering pose consultant.", e);
    }


    node = Ros2Factory.getDefaultNode();
    node.createTf();
    node.spin(100);

    try {
      URI uri = new URI("ws://localhost:9090");
      forkliftStateListener = new RosForkliftStateListener(uri, "/current_mission", "Success", "Failure");
      if (wait_for_ros_topic) {
        forkliftStateListener.connect();
      }
    } catch (URISyntaxException e) {
      log.error("Error connecting to /current_mission topic",e);
    }
  }

  public Forklift() {
    super();
  }


  @Override
  public double[] getPoseGlobalQuat() {
    Matrix4d transform = node.lookupTransform(map_frame, base_frame);
    Quat4d quat = new Quat4d();
    transform.get(quat);
    Vector3d trans = new Vector3d();
    transform.get(trans);
    return new double[]{trans.x, trans.y, quat.x, quat.y, quat.z, quat.w};
  }

  @Override
  public void setPoseGlobal(double x, double y, double theta) {

  }

  private Pair<Point3d, Quat4d> getLocationPose(Symbol location) {
    Pair<Point3d, Quat4d> pose = null;
    if (!location.isTerm()) {
      // location is a reference
      PoseReference poseReference = consultant.getReference(location);
      if (poseReference != null && poseReference.hasPose()) {
        pose = poseReference.getPose();
      } else {
        log.error("Pose " + location + " not in consultant");
      }
    } else {
      log.error("Wrong type");
    }
    return pose;
  }

  protected Pair<Point3d, Quat4d> getPose() {
    Matrix4d transform = node.lookupTransform(map_frame, base_frame);
    Quat4d quat = new Quat4d();
    transform.get(quat);
    Vector3d trans = new Vector3d();
    transform.get(trans);
    Point3d point = new Point3d(trans.x, trans.y, trans.z);
    return new MutablePair<>(point, quat);
  }

  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    Pair<Point3d, Quat4d> pose = getLocationPose(location);
    return goToLocation(pose.getLeft().x, pose.getLeft().y, pose.getRight().x, pose.getRight().y,
            pose.getRight().z, pose.getRight().w, wait);
  }

  @Override
  public Justification goToLocation(Symbol location) {
    return goToLocation(location, true);
  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    return goToLocation(desiredLocation);
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y,
                                    double quat_z, double quat_w, boolean wait) {
    // ControllerCommandRequestMessage msg = new ControllerCommandRequestMessage();
    // //{end_x: 2.0, end_y: 11.0, end_yaw: 1.5, driving_direction: 0, command: 0}"
    // msg.end_x = xdest;
    // msg.end_y = ydest;
    // msg.end_yaw = RotationHelpers.quatToXYZRotations(new Quat4d(quat_x, quat_y, quat_z, quat_w)).z;
    // msg.driving_direction = false; //todo: Check this based on if carrying something or not
    // msg.command = 0;
    // ControllerCommandResponseMessage resp = node.callService("controller_service", msg);
    // if (resp != null && resp.success) {
    //   return new ConditionJustification(true);
    // } else {
    //   log.error("Goto failed. Continuing"); //todo: for testing

    //   return new ConditionJustification(true);
    // }

    return new ConditionJustification(true);
  }

  private Justification goToLocation(Point3d point, Quat4d quat) {
    return goToLocation(point.x, point.y, quat.x, quat.y, quat.z, quat.w, true);
  }

  @Override
  public Justification approachLocation(Symbol location) {
    //get object from consultant
    Quat4d objectOrientation = new Quat4d();
    Point3d objectPose = new Point3d();

    //Calculate point given an offset
    Point3d newPoint = RotationHelpers.calcTargetOffset(objectPose, objectOrientation, new Vector3d(0, 0, 1), 1.0f);

    Quat4d newRotation = new Quat4d(objectOrientation);
    newRotation.inverse();

    return goToLocation(newPoint, newRotation);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    return approachLocation(desiredLocation);
  }

  @Override
  public Justification stop() {
    // ControllerCommandRequestMessage msg = new ControllerCommandRequestMessage();
    // msg.command = 3;
    // ControllerCommandResponseMessage resp = node.callService("controller_service", msg);
    // if (resp.success) {
    //   return new ConditionJustification(true);
    // } else {
    //   return new ConditionJustification(false);
    // }
    return new ConditionJustification(true);
  }

  //todo
  @Override
  public Justification isMoving() {
    return null;
  }

  @Override
  public List<Map<Variable, Symbol>> checkAt(Term locationTerm) {
    List<Symbol> args = locationTerm.getArgs();
    Pair<Point3d, Quat4d> locationPose = getLocationPose(args.get(1));
    ArrayList<Map<Variable, Symbol>> returnVal = new ArrayList<>();
    if (locationPose != null) {
      Point3d currentPoint = getPose().getLeft();
      Point3d goalPoint = locationPose.getLeft();
      log.debug("currentPoint " + currentPoint.getX() + " " + currentPoint.getY() + " " + currentPoint.getZ());
      log.debug("goalPoint " + goalPoint.getX() + " " + goalPoint.getY() + " " + goalPoint.getZ());
      double dist = Util.getDistanceFrom(currentPoint.getX(), currentPoint.getY(),
              goalPoint.getX(), goalPoint.getY());
      log.debug("offset " + pointDistThresh);
      log.debug("distance " + dist);
      Quat4d qt = locationPose.getRight();
      Quat4d qt4d = new Quat4d(qt.getX(), qt.getY(), qt.getZ(), qt.getW());
      Tuple3d angles = RotationHelpers.quatToXYZRotations(qt4d);
      log.debug("goalAngle " + angles.getZ());

      Quat4d qtCurrent = getPose().getRight();
      Quat4d qtCurrent4d = new Quat4d(qtCurrent.getX(), qtCurrent.getY(), qtCurrent.getZ(),
              qtCurrent.getW());
      Tuple3d anglesCurrent = RotationHelpers.quatToXYZRotations(qtCurrent4d);
      double angleDiff = SimpleGeometry.diffAngle(angles.getZ(), anglesCurrent.getZ());
      log.debug("currentAngle " + anglesCurrent.getZ());
      log.debug("angle thresh " + orientationThresh);
      log.debug("angle distance " + angleDiff);

      if (dist < pointDistThresh && Math.abs(angleDiff) < orientationThresh) {
        returnVal.add(new HashMap<>());
      } else {
        log.debug("[checkAt] not at location");
      }
    }
    return returnVal;
  }

  public void recordPose(Symbol poseName) {
    PoseReference ref = consultant.createReference(Factory.createVariable("VAR0", mapDescriptor), List.of(Factory.createPredicate(poseName.toString(), Factory.createVariable("VAR0", mapDescriptor))));
    Pair<Point3d, Quat4d> currPose = getPose();
    ref.setPose(currPose.getLeft(), currPose.getRight());
  }

  // Populates belief with the distances between zones, e.g. fluent_equals(dist(zone1, zone2), 5)
  private void calculateDistances() {
    List<Term> properties = new ArrayList<>();
    Set<Term> beliefs = new HashSet<>();
    properties.add(Factory.createPredicate("ground", Factory.createVariable("VAR0", mapDescriptor)));
    properties.add(Factory.createPredicate("truck", Factory.createVariable("VAR0", mapDescriptor)));
    properties.add(Factory.createPredicate("trailer", Factory.createVariable("VAR0", mapDescriptor)));
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
        log.error("Error waiting on assertBeliefs:", e);
      }
    }

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, beliefs);
    } catch (TRADEException e) {
      log.error("[calculateDistances]", e);
    }

  }

  // Populates belief with the weights of objects, e.g. fluent_equals(weight(palette1), 5)
  private void readWeights() {

  }

  // FIXME: This needs to change once we know what the area object looks like
  @TRADEService
  @Action
  public void initAreas() {
    List<GetAreasResponseMessage> areas = getAreas();
    Set<Term> beliefs = new HashSet<>();
    for (GetAreasResponseMessage area : areas) {


//      Category category = Category.values()[area.category];
//      // Create location reference for each area
//      List<Term> properties = new ArrayList<>();
//      properties.add(Factory.createPredicate(category.name(), Factory.createVariable("VAR0", mapDescriptor)));
//
//      Symbol name = Factory.createSymbol(category.name() + area.id, mapDescriptor);
//      PoseReference location = consultant.createReference(Factory.createVariable("VAR0", mapDescriptor), properties);
////      PoseReference location = consultant.createNamedReference(Factory.createVariable("VAR0", mapDescriptor), properties, name);

      Category category = Category.values()[area.category];
      // Create location reference for each area
      List<Term> properties = new ArrayList<>();
      properties.add(Factory.createPredicate(category.name(), Factory.createVariable("VAR0", mapDescriptor)));

//      if (area.id.contains("loadingzone")) {
//        properties.add(Factory.createPredicate("loadingzone", Factory.createVariable("VAR0", mapDescriptor)));
//      } else if (area.id.contains("truck")) {
//        properties.add(Factory.createPredicate("truck", Factory.createVariable("VAR0", mapDescriptor)));
//      }


      PoseReference location = consultant.createReference(Factory.createVariable("VAR0", mapDescriptor), properties);
      Point3d pt = new Point3d(area.x, area.y, 0);
      Quat4d qt = new Quat4d(0, 0, 0, 1);
      location.setPose(pt, qt);

      // TODO: is this needed anywhere?
      diarcToAitRefs.put(location.refId, Factory.createSymbol(area.id, mapDescriptor));

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
      log.error("[initAreas]", e);
    }

    calculateDistances();
  }

  private Integer getFluentAsInt(String fluent, Symbol location) {
    Variable x = Factory.createVariable("X");
    List<Map<Variable, Symbol>> bindings;
    try {
      bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class))
              .call(List.class, Factory.createPredicate("fluent_equals", Factory.createPredicate(fluent, location), x));
      Symbol val = bindings.get(0).get(x);
      return Float.valueOf(val.getName()).intValue();
    } catch (TRADEException e) {
      log.error("[getFluentAsInt]", e);
    }
    return null;
  }

  @Action
  @TRADEService
  public Predicate createWeightGoal(Symbol location) {
    List<Symbol> pallets = new ArrayList<>(visConsultant.getActivatedEntities().keySet());
    List<Integer> weights = new ArrayList<>();

    Variable x = Factory.createVariable("X");
    for (Symbol pallet : pallets) {
      weights.add(getFluentAsInt("weight", pallet));
    }

    List<Integer> indices = maximizeWeight(weights.stream().mapToInt(i -> i).toArray(), getFluentAsInt("capacity", location), getFluentAsInt("slots", location));
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
//    int capacity = 15;
//    int[] weights = new int[]{1, 4, 6, 9, 10, 15, 20};
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

    log.info("Items selected:");
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
  private List<GetAreasResponseMessage> getAreas() {

    //  node.addService(new GetAreasServiceDefinition(), "get_areas");
    //  GetAreasResponseMessage resp = node.callService("get_areas", new GetAreasRequestMessage());
    //  log.info("Received " + resp);

    List<GetAreasResponseMessage> areas = new ArrayList<>();
    GetAreasResponseMessage zonea = new GetAreasResponseMessage();
    zonea.id = "ground_1"; // ground_1
    zonea.category = 0;
    zonea.slots = 5;
    zonea.pallets = 3;
    zonea.x = 0;
    zonea.y = 0;
    areas.add(zonea);

    GetAreasResponseMessage zoneb = new GetAreasResponseMessage();
    zoneb.id = "ground_2"; // ground_2
    zoneb.category = 0;
    zoneb.slots = 5;
    zoneb.pallets = 3;
    zoneb.x = 5;
    zoneb.y = 5;
    areas.add(zoneb);

    GetAreasResponseMessage truck = new GetAreasResponseMessage();
    truck.id = "loading_platform"; // loading_platform
    truck.category = 2;
    truck.slots = 3;
    truck.pallets = 0;
    truck.x = 5;
    truck.y = 0;
    areas.add(truck);

    return areas;

  }

  @TRADEService
  @Action
  public Justification load_prim() {
    LoadRequestMessage msg = new LoadRequestMessage();
    LoadResponseMessage resp = node.callService("load", msg);
    if (resp != null && resp.success) {
      return new ConditionJustification(true);
    } else {
      log.error("Load failed. Continuing"); //todo: for testing

      return new ConditionJustification(true);
    }
  }

  @TRADEService
  @Action
  public Justification unload_prim() {
    UnloadRequestMessage msg = new UnloadRequestMessage();
    UnloadResponseMessage resp = node.callService("unload", msg);
    if (resp != null && resp.success) {
      return new ConditionJustification(true);
    } else {
      log.error("Unload failed. Continuing"); //todo: for testing
      return new ConditionJustification(true);
    }
  }

  public String sendRosCmd(String cmd) {
    StringBuilder output = new StringBuilder();
    ProcessBuilder processBuilder = new ProcessBuilder();

    String test_prefix = ros_cmd_as_echo ? "echo " : "";

    // Set the command to be executed
    processBuilder.command("bash", "-c", test_prefix + "source /home/ros/workspace/ros_environment/install/setup.bash && " + test_prefix + cmd);

    try {
      // Start the process
      Process process = processBuilder.start();

      // Read the output of the command
      BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
      String line;
      while ((line = reader.readLine()) != null) {
        output.append(line).append("\n");
      }

      // Wait for the process to complete and check the exit value
      int exitValue = process.waitFor();
      if (exitValue != 0) {
        // Read any error output if the command failed
        BufferedReader errorReader = new BufferedReader(new InputStreamReader(process.getErrorStream()));
        StringBuilder errorOutput = new StringBuilder();
        while ((line = errorReader.readLine()) != null) {
          errorOutput.append(line).append("\n");
        }
        throw new IOException("Error executing command: " + errorOutput.toString());
      }

    } catch (IOException | InterruptedException e) {
      log.error("[sendRosCmd]",e);
    }

    return output.toString();
  }

  boolean WaitForForkliftReady() {
    if (wait_for_ros_topic) {
      return forkliftStateListener.WaitForNextStatePublish();
    }

    return true;
  }

  @TRADEService
  @Action
  public Justification deliver_prim(Symbol from, Symbol to) {

    String result;

    DeliverRequestMessage msg = new DeliverRequestMessage(from.getName(), to.getName());

    String loading_area_id = msg.loading_area_id;
    String unloading_area_id = msg.unloading_area_id;

    result = sendRosCmd("ros2 service call /send_order crayler_nav_msgs/srv/SendOrder \"{loading_area_id: \"" + loading_area_id + "\", unloading_area_id: \"" + unloading_area_id + "\", nr_pallets: " + msg.nr_pallets + "}\"");
    log.warn("send order result: {}", result);

    String behavior_tree_to_execute = "loading_demo.xml";

    result = sendRosCmd("ros2 service call /start_bt crayler_nav_msgs/srv/SendMessage \"{message: \"" + behavior_tree_to_execute + "\"}\"");
    log.warn("send message result: {}", result);

    boolean mission_result = WaitForForkliftReady();

    return new ConditionJustification(mission_result);
  }

  private enum Category {
    ground,
    trailer,
    truck
  }

}
