/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.moveit;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;

import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.consultant.util.Utilities;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import edu.tufts.hrilab.moveit.config.gson.*;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.*;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.moveit.util.MoveItHelper;
import edu.tufts.hrilab.diarcros.util.Convert;
import edu.tufts.hrilab.diarcros.common.JointStateSub;
import edu.tufts.hrilab.diarcros.moveit.GenericMoveGroup;
import edu.tufts.hrilab.diarcros.msg.Time;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState;
import edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2;
import edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointField;
import edu.tufts.hrilab.diarcros.msg.shape_msgs.Mesh;
import edu.tufts.hrilab.diarcros.msg.shape_msgs.MeshTriangle;
import edu.tufts.hrilab.diarcros.msg.shape_msgs.SolidPrimitive;
import edu.tufts.hrilab.diarcros.msg.std_msgs.Header;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectoryPoint;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.MultiDOFJointTrajectory;
import edu.tufts.hrilab.util.RotationHelpers;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.vision.stm.MemoryObjectUtil;
import edu.tufts.hrilab.vision.stm.MemoryPrimitive;
import edu.tufts.hrilab.vision.util.CompressionUtil;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import edu.tufts.hrilab.vision.util.StereoProc;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.RosException;
import org.ros.exception.ServiceException;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.awt.Dimension;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.zip.DataFormatException;

import static edu.tufts.hrilab.diarcros.util.Convert.*;

public class MoveItComponent extends DiarcComponent implements MoveItInterface {

  private static final Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();
  private static final double DEFAULT_PRESS_OFFSET = 0.1; // meters
  private static final Quat4d DEFAULT_PRESS_ORIENT = new Quat4d(0, 0, 1, 0); // facing -x-axis direction

  /**
   * If you're extending this class you can set this one to the config name so it's the default.
   */
  protected String configName;
  protected String defaultConfigDir = "config/edu/tufts/hrilab/moveit";

  // This is the stuff set by the .JSON files. Add a new JSON config instead of writing to this stuff directly.
  // Most data is stored directly in the MoveItConfig, but some data requires additional processing and is stored
  // in local fields (e.g., groupNames, eeTransform, etc.).
  protected MoveItConfig moveItConfig; // main class populated from JSON file
  private String[] groupNames;        // A list of available group names.
  protected Map<String, GenericManipulator> grippers; // group name to gripper object
  protected Map<String, String> endEffectorNames;     // group name to end effector name
  protected Map<String, String[]> jointNames;         // group name to collection of joints
  protected Matrix4d eeTransform;        // transform between robots end-effector and expected end-effector transform (i.e., pr2 end-effector)

  /**
   * Data structure for remapping diarcros nodes rostopics. Currently never populated.
   */
  private Map<String, String> rosRemappings = new HashMap<>();

  /**
   * For storing and recording trajectories.
   */
  protected Map<String, JointTrajectory> recordedTrajectories = new HashMap<>();
  protected String trajectoriesFile;
  protected volatile boolean recordingTrajectory;

  /**
   * diarcros nodes.
   */
  protected GenericMoveGroup moveGroup;
  protected JointStateSub jointStateSub;

  private String posesFile;
  private String eePosesFile;
  /**
   * Pose name to robot joint states. Populated from JSON config file.
   */
  protected Map<String, RobotState> poses = new HashMap<>();
  /**
   * Name of initial pose, if one has been specified. NOTE: this pose name must be in the json configuration.
   */
  protected Symbol initStartPose;
  /**
   * Pose name to robot ee states. Populated at runtime by saving ee poses.
   */
  protected Map<String, Pair<Point3d, Quat4d>> eePoses = new HashMap<>();
  /**
   * Point cloud counter for publishing ROS point cloud messages.
   */
  private int pcCounter = 0;
  /**
   * Map of currently grasped objects (refIf -> T/F collision object added).
   */
  private Map<Symbol, Boolean> graspedObjects = new HashMap<>();

  //remember where you picked objects from so you can put them down.
  //TODO:brad: move this info outof the component and into belief?
  private Map<Symbol, Pair<Point3d, Quat4d>> graspedObjectLocations = new HashMap<>();

  PoseConsultant consultant;

  /**
   * If true, apply orientation constraints to motion planning that
   * attempt to keep end effector orientated appropriately for carrying.
   * <p>
   * TODO: The axis doesn't need to be constrained depends on the robot and also
   *       the type of carrying (cup, caddy, tray, etc), and is dynamiclally determined
   *       based on which end effector axis is aligned with the world z-axis.
   */
  private boolean applyCarryingConstraints = false;

  /**
   * Default constructor.
   */
  public MoveItComponent() {
    super();
  }

  @Override
  protected void init() {
    if (configName != null && !configName.isEmpty()) {
      // It's possible no args have been passed to specify what config to use, so just run the default.
      loadConfigFile(configName);
    } else {
      log.error("No config set. Shutting down component. Please set a config in your MoveItComponent sub-class," +
              "or via the command line with -config <configName>.");
    }

    // instantiate diarcros nodes
    connectToMoveGroup();
    jointStateSub = new JointStateSub(moveItConfig.rosNamespace, rosRemappings);

    // additional setup, after diarcros nodes have been instantiated
    addPosesFromConfig(moveItConfig.poses);
    addEEPosesFromConfig(moveItConfig.eeposes);
    addCollisionObjectsFromConfig(moveItConfig.collisions);

    // add local static transforms to TF component when it comes up
    try {
      TRADE.requestNotification(this, "joined", new TRADEServiceConstraints().name("addLocalStaticTransform"), null, "addLocalStaticTransforms");
      Collection<TRADEServiceInfo> services = TRADE.getAvailableServices(new TRADEServiceConstraints().name("addLocalStaticTransform"));
      if (services.size() == 1) {
        addLocalStaticTransforms(services.iterator().next());

        // cancel notification bc the addLocalStaticTransform service was already up
//        TRADE.cancelNotification(this, "joined", new TRADEServiceConstraints().name("addLocalStaticTransform"));
      }
    } catch (TRADEException e) {
      log.error("Error registering for notification: addLocalStaticTransform", e);
    }

    // load poses and traj files
    loadPosesFromFile(posesFile);
    loadEEPosesFromFile(eePosesFile);
    loadTrajectoriesFromFile(trajectoriesFile);

    // construct pose consultant *after* poses are loaded from file
    consultant = new PoseConsultant(PoseReference.class, "pose", new ArrayList<>());
    try {
      List<String> groups = this.getMyGroups();
      groups.add(consultant.getKBName());
      TRADE.registerAllServices(consultant, groups);
    } catch (TRADEException e) {
      log.error("Error registering with trade ", e);
    }
    log.info("\n\n\nDone constructing MoveItComponent! You're running the " + configName + " configuration on ROS " + moveGroup.getMoveGroupRosVersion() + "\n\n\n");

    if (initStartPose != null) {
      goToPose(initStartPose);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("poses").hasArg().argName("file").desc("load poses file").build());
    options.add(Option.builder("eeposes").hasArg().argName("file").desc("load eeposes file").build());
    options.add(Option.builder("traj").longOpt("trajectories").hasArg().argName("file").desc("load trajectories file").build());
    options.add(Option.builder("config").hasArg().argName("PR2/UR5/etc").desc("specify a config file to read").build());
    options.add(Option.builder("pose").longOpt("startPose").hasArg().argName("<pose-name>").desc("specify an initial arm(s) pose").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("poses")) {
      log.warn("DEPRECATED: define poses in the JSON configuration file.");
      posesFile = cmdLine.getOptionValue("poses");
    }
    if (cmdLine.hasOption("eeposes")) {
      log.warn("DEPRECATED: define poses in the JSON configuration file.");
      eePosesFile = cmdLine.getOptionValue("eeposes");
    }
    if (cmdLine.hasOption("traj")) {
      trajectoriesFile = cmdLine.getOptionValue("traj");
    }
    if (cmdLine.hasOption("config")) {
      configName = cmdLine.getOptionValue("config");
    }
    if (cmdLine.hasOption("startPose")) {
      initStartPose = Factory.createSymbol(cmdLine.getOptionValue("startPose"));
    }
  }

  /**
   * To add more ROS versions:
   * - node gen on the movegroup
   * - Drop the generated file into diarcros/moveit/(version)/MoveGroup.java
   * - Update the Ant target in the same way the other versions are built in src-built-targets
   * - Make sure the new class extends GenericMoveGroup (your IDE may complain about duplicate class names, that's OK)
   * - Potentially modify the generated file to include necessary functionality that was not
   * automatically implemented (move_group.shutdown(), for example, is not automatically generated for Kinetic)
   * With all of the functionality added, it should "just work": This setup looks for a package called
   * edu.tufts.hrilab.diarcros.moveit.MoveGroup(), but only the MoveGroup corresponding to your ROS version should come up (thanks
   * to those ant build targets).
   */
  protected void connectToMoveGroup() {
    try {
      moveGroup = new edu.tufts.hrilab.diarcros.moveit.MoveGroup(moveItConfig.rosNamespace, rosRemappings);
      moveGroup.waitForNode();
    } catch (NullPointerException e) {
      log.error("Move group failed to load! " + e);
    }
  }

  private void loadConfigFile(String localConfigName) {
    if (System.getenv().get("ROS_DISTRO") == null) {
      log.error("Trying to get your ROS_DISTRO returned null. You have probably not installed ROS properly." +
              "Nothing else will work here, so we will not continue to config.");
      return;
    }

    // Read the JSON object file and dump it into a MoveItConfig object
      Gson gson = new Gson();
      String configPath = Resources.createFilepath(defaultConfigDir, localConfigName);
      InputStream in = getClass().getResourceAsStream(configPath);
      BufferedReader br = new BufferedReader(new InputStreamReader(in));
      moveItConfig = gson.fromJson(br, MoveItConfig.class);

    // These ones are complex enough that we need to parse a special thing together for them.
    groupNames = moveItConfig.getGroupNames();
    jointNames = moveItConfig.getJointNamesMap();
    endEffectorNames = moveItConfig.getEndEffectorNameMap();
    grippers = moveItConfig.getGripperClassMap();
    eeTransform = moveItConfig.getEETransform();
  }

  private void addCollisionObjectsFromConfig(MoveItConfigCollisionObject[] collisionObjects) {
    if (collisionObjects == null || collisionObjects.length == 0)
      return;

    for (MoveItConfigCollisionObject collisionObject : collisionObjects) {
      /* FIXME, and an explanation
      This works around a bug where, after things are handed over to move_group in addCollisionObject(), the collision objects
      go over to the move group and MoveIt thinks we want to transform between  '' and '/world', which fails. So the parent link
      info gets lost in translation somewhere, I guess. However, the object does get created is attached to the parent
      link appropriately, it just gets spawned into the world at a location based on the origin plus the provided offset data.
      To work around this, a new transformation is created: we take the transform between the base link and parent link,
      and combine that transform with the transform we actually want. Now when we attach the object and it defaults
      instead to the base link, we've taken that into account and everything is OK. This works, but is clumsy in that
      we're kinda taking 1 step back before taking 2 forward using redundant information.

      To recreate this bug, replace the entire contents of this for-loop with simply:
       addCollisionObject(new Pose(new Point(collisionObject.x, collisionObject.y, collisionObject.z),
              convert(collisionObject.roll, collisionObject.pitch, collisionObject.yaw)),
              collisionObject.name, collisionObject.parent, collisionObject.width, collisionObject.depth, collisionObject.height);
      */

      //Todo: Will: Test more intensely - it appears as if the above bug no longer exists, but I've only tested on the
      //ur5 in simulation

      //Matrix4d transform = getTransform(moveItConfig.baseLinkString, collisionObject.parent);
      Matrix4d object = RotationHelpers.buildTransformMatrix(
              new Vector3d(collisionObject.x, collisionObject.y, collisionObject.z),
              new Vector3d(collisionObject.roll, collisionObject.pitch, collisionObject.yaw));
      //transform.mul(object);

      addCollisionObject(
              convertToPose(object),
              collisionObject.name,
              collisionObject.parent,
              collisionObject.width,
              collisionObject.depth,
              collisionObject.height);
    }
  }


  private void addEEPosesFromConfig(MoveItConfigEEPose[] poses) {
    if (poses == null || poses.length == 0)
      return;

    // For each of the poses in the config, create a new robot state with joint state values and corresponding frame ID names, and null for effort, etc.
    for (MoveItConfigEEPose p : poses) {
      this.eePoses.put(p.poseName, new MutablePair<>(new Point3d(p.px, p.py, p.pz), new Quat4d(p.qx, p.qy, p.qz, p.qw)));
    }
  }

  private void addPosesFromConfig(MoveItConfigPose[] poses) {
    if (poses == null || poses.length == 0)
      return;

    // For each of the poses in the config, create a new robot state with joint state values and corresponding frame ID names, and null for effort, etc.
    for (MoveItConfigPose p : poses) {
      this.poses.put(p.poseName, new RobotState(new JointState(Arrays.asList(jointNames.get(p.poseGroup)), p.jointStates, null, null), null, null, false));
    }
  }

  @TRADEService
  public void addLocalStaticTransforms(TRADEServiceInfo tsi) {
    log.warn("adding local transform");
    LocalStaticTransform[] localStaticTransforms = moveItConfig.localStaticTransforms;
    if (localStaticTransforms == null || localStaticTransforms.length == 0) {
      return;
    }

    for (LocalStaticTransform lst : localStaticTransforms) {
      Matrix4d transform = edu.tufts.hrilab.util.Convert.convertToMatrix4d(new Point3d(lst.x, lst.y, lst.z), RotationHelpers.xyzRotationsToQuaternion(new Vector3d(lst.roll, lst.pitch, lst.yaw)));
      try {
        tsi.call(void.class, lst.parent, lst.child, transform);
      } catch (TRADEException e) {
        log.error("Error calling addLocalStaticTransform.", e);
      }
    }
  }

  @Override
  public void enableCarryingConstraints() {
    applyCarryingConstraints = true;
  }

  @Override
  public void disableCarryingConstraints() {
    applyCarryingConstraints = false;
  }

  @TRADEService
  @Action
  public boolean interrupt() {
    //There does exist a stop command on the move group in move it source, but it is not currently implemented in
    //our GenericMoveGroup implementation. Since subsequent calls overwrite previous ones, I think that the following
    //should be a simpler way to achieve the stop, as long as it doesn't require planning time.
    Pair<Point3d, Quat4d> curPose = getPose(endEffectorNames.get("manipulator"));
    return moveToRelative("manipulator", curPose.getLeft(), curPose.getRight()).getValue();
  }

  @Override
  @OnInterrupt(onCancelServiceCall = "interrupt()", onSuspendServiceCall = "interrupt()")
  public Justification moveTo(String groupName, Symbol refId) {
    return moveTo(groupName, refId, new ArrayList<>());
  }

  public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
    log.debug("[moveTo(group,refId,constraints)] method entered with constraints: " + constraints);

    List<Grasp> graspOptions = getOrderedGraspOptions(refId, constraints);
    return moveToGraspOption(groupName, graspOptions);
  }

  protected List<Grasp> getOrderedGraspOptions(Symbol refId, List<? extends Term> constraints) {
    List<Grasp> graspOptions;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("calculateGraspOptions").argTypes(Symbol.class, List.class));
      graspOptions = tsi.call(List.class, refId, constraints);
    } catch (TRADEException e) {
      log.error("[moveTo] exception getting grasp options from reference, returning null", e);
      return null;
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

    return graspOptions;
  }

  protected Justification moveToGraspOption(String groupName, List<Grasp> graspOptions) {
    // sanity check
    if (graspOptions == null || graspOptions.isEmpty()) {
      return new ConditionJustification(false, Factory.createPredicate("found(graspoptions)"));
    }

    // turn on (or update if it's already on) collision avoidance
    enableCollisionAvoidance();

    // iterate through grasp options until one works, or we're out of options
    boolean approachSuccess = false;
    int graspIndex = 0;
    while (!approachSuccess && (graspIndex < graspOptions.size())) {
      // pick a grasp option
      log.debug("Grasp index: " + graspIndex);
      Grasp grasp = graspOptions.get(graspIndex++);

      // doing some basic grasp checks
      if (grasp.getNumPoints() == 0) {
        log.error("[moveTo(group,object)] invalid grasp option contains no grasp points.");
        continue;
      }

      if (log.isTraceEnabled()) {
        log.trace(String.format("[moveTo(group,object)] chosen grasp: %s", gson.toJson(grasp)));
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
      return new ConditionJustification(false, Factory.createPredicate("movedTo(graspoption)"));
    }

    log.debug("[moveTo(group,refId)] success!");
    return new ConditionJustification(true);
  }

  private Vector3d getVectorFromRobotToObj(MemoryObject o) {
    Vector3d vec;
    if (o.transformToBase()) {
      //Todo: Will: Make the base of the robot point configurable?
      vec = new Vector3d(o.getLocation());
      vec.sub(new Vector3d(0, 0, 0.02));
      //for now we only care about matching on 2 dimensions
      vec.setZ(0);
      return vec;
    } else {
      log.warn("[getVectorFromRobotToObj]Unable to transform memory object to base");
    }
    return null;
  }

  @Override
  public boolean moveToJointPositions(String[] jointNames, double[] positions) {
    return moveToJointPositions(jointNames, positions, null, null);
  }

  private MotionPlanRequest generateMotionPlanFromJointState(String[] jointNames, double[] positions, double[] velocities, double[] efforts, JointState currentState) {
    // Start where we're currently at
    RobotState start_state = new RobotState();
    start_state.setJointState(currentState);

    // Go through the positions, efforts, and velocities of all joints for our goal states
    // If the user has specified a new value, assign it. Otherwise, leave it alone by
    // setting it to currentState
    JointState goalJointState = new JointState();
    goalJointState.setName(currentState.getName());
    goalJointState.setPosition(new double[currentState.getName().size()]);
    goalJointState.setEffort(new double[currentState.getName().size()]);
    goalJointState.setVelocity(new double[currentState.getName().size()]);
    List<String> jointNameList = Arrays.asList(jointNames);
    for (String thisJoint : currentState.getName()) { // Run through all of our joint options
      int jointStateIndex = currentState.getName().indexOf(thisJoint);  // Turn that joint option into a corresponding index
      if (jointNameList.contains(thisJoint)) {                           // Check if it's something the user is trying to set
        int providedPointsIndex = jointNameList.indexOf(thisJoint);     // Turn the joint into a corresponding index for the jointNames
        goalJointState.getPosition()[jointStateIndex] = positions[providedPointsIndex]; // Set the position

        // It's possible velocities/efforts are null/0. If they are, leave them alone. Otherwise, set them to the default.
        if (velocities != null && velocities.length != 0) {
          goalJointState.getVelocity()[jointStateIndex] = velocities[providedPointsIndex];
        } else {
          goalJointState.getVelocity()[jointStateIndex] = currentState.getVelocity()[jointStateIndex];
        }

        if (efforts != null && efforts.length != 0) {
          goalJointState.getEffort()[jointStateIndex] = efforts[providedPointsIndex];
        } else {
          goalJointState.getEffort()[jointStateIndex] = currentState.getEffort()[jointStateIndex];
        }

      } else {
        goalJointState.getPosition()[jointStateIndex] = currentState.getPosition()[jointStateIndex];
        goalJointState.getEffort()[jointStateIndex] = currentState.getEffort()[jointStateIndex];
        goalJointState.getVelocity()[jointStateIndex] = currentState.getVelocity()[jointStateIndex];
      }
    }

    // Set up a goal to where we want to be based off of the joint_state changes we made earlier
    RobotState goalRobotState = new RobotState();
    goalRobotState.setJointState(goalJointState);
    goalRobotState.setIsDiff(true);

    edu.tufts.hrilab.diarcros.msg.moveit_msgs.Constraints joint_constraints = MoveItHelper.makeJointConstraints(goalRobotState, goalJointState.getName());
    List<Constraints> goal_constraints = new ArrayList<>();
    goal_constraints.add(joint_constraints);

    MotionPlanRequest req = new MotionPlanRequest();
    req.setStartState(start_state);
    req.setGoalConstraints(goal_constraints);
    req.setWorkspaceParameters(new WorkspaceParameters(new Vector3(0, 0, 0), new Vector3(0, 0, 0)));
    req.setNumPlanningAttempts(moveItConfig.maxMoveToAttempts);
    req.setAllowedPlanningTime(moveItConfig.maxPlanningTime);
    req.setGroupName(moveItConfig.defaultGroupName);

    return req;
  }

  @Override
  public boolean moveToJointPositions(String[] joint_names, double[] positions, double[] velocities, double[] efforts) {
    JointState currentState;
    try {
      currentState = jointStateSub.getCurrentJointState();
      if (!currentState.getName().containsAll(Arrays.asList(joint_names))) {
        log.error("You tried specified an invalid joint name! Valid options are " + Arrays.toString(currentState.getName().toArray()) + ", you provided " + Arrays.toString(joint_names));
        return false;
      }
    } catch (NullPointerException ne) {
      // In some very rare edge cases I haven't been able to nail down (only observed on PR2), this will throw null pointer.
      // It appears to be a rare race condition, and if you try again, it'll likely work the second time.
      log.error("Getting current joint state failed!");
      return false;
    }

    MotionPlanRequest firstAttempt = generateMotionPlanFromJointState(joint_names, positions, velocities, efforts, currentState);
    if (executeMotionPlanRequest(firstAttempt)) {
      return true;
    } else {
      // It's possible that the attempt to adjust all joints fails if one of the joints isn't where we think it should be. This can occur regularly
      // and continue to break things, such as with the PR2's head floppin' around. To get around that, we're going assume that the default move group
      // has all the joints we care about staying static, so we'll make a new 'current state' that ignores every other link.
      currentState = jointStateSub.getCurrentJointStateBlocking(); // blocking to make sure we have new information, the previous currentState may have been written over
      List<String> modifiedJointNames = new ArrayList<>();
      List<Double> modifiedPosition = new ArrayList<>();
      List<Double> modifiedVelocities = new ArrayList<>();
      List<Double> modifiedEfforts = new ArrayList<>();
      for (String thisJoint : currentState.getName()) {
        if (Arrays.asList(jointNames.get(moveItConfig.defaultGroupName)).contains(thisJoint) || Arrays.asList(joint_names).contains(thisJoint)) {
          // Great! The joint we're looking at is either 1) in the default group or 2) isn't in the default group but was one the user explicitly specified.
          // Either way, this is a joint we care about.
          int index = currentState.getName().indexOf(thisJoint);
          modifiedJointNames.add(thisJoint);
          modifiedPosition.add(currentState.getPosition()[index]);
          modifiedVelocities.add(currentState.getVelocity()[index]);
          modifiedEfforts.add(currentState.getEffort()[index]);
        }
      }

      if (modifiedJointNames.isEmpty()) {
        // Looks like we didn't get anything.
        log.warn("Parsing through available joints for your specified joints failed... returning false to be safe, " +
                "but heads up: something's probably broken in your request or config file.");
        return false;
      }

      JointState modifiedState = new JointState();
      modifiedState.setName(modifiedJointNames);
      modifiedState.setPosition(ArrayUtils.toPrimitive(modifiedPosition.toArray(new Double[0])));
      modifiedState.setVelocity(ArrayUtils.toPrimitive(modifiedVelocities.toArray(new Double[0])));
      modifiedState.setEffort(ArrayUtils.toPrimitive(modifiedEfforts.toArray(new Double[0])));

      MotionPlanRequest secondAttempt = generateMotionPlanFromJointState(new String[]{String.valueOf(modifiedJointNames)}, positions, velocities, efforts, modifiedState);
      return executeMotionPlanRequest(secondAttempt);
    }
  }

  protected boolean moveToApproach(String groupName, Grasp grasp, float offset) {
    log.debug("[moveToApproach] method entered.");
    int tryNum = 0;
    // Tries to plan to the first stage a few times, because movegroup's setNumPlanningAttempts is buggy.
    while (!moveTo(groupName, grasp, offset)) {
      if (++tryNum >= moveItConfig.maxMoveToAttempts) {
        // This number should be altered when you want more attempts at successful plans
        // We'll give up on this action as we can't plan to the pre-grasp position
        return false;
      }

      // Sometimes increasing the offset will allow the plan to succeed
      offset += 0.025;
    }

    return true;
  }

  protected boolean moveTo(String groupName, Grasp grasp, float grasp_offset) {
    log.debug("[moveTo(group,grasp)] method entered.");

    if (grasp.getNumPoints() == 1) {
      return moveToPinchTogetherGrasp(groupName, grasp, grasp_offset);
    } else if (grasp.getNumPoints() == 2) {
      return moveToPinchApartGrasp(groupName, grasp, grasp_offset);
    } else {
      log.error("Invalid number of grasp points in Grasp option {}", grasp.getNumPoints());
      return false;
    }
  }

  @Override
  public Justification pointTo(String groupName, Point3d location) {
    log.debug("[pointTo] target location: " + location);

    // get pointingFrame (or optical frame if not defined) relative to base link coord. frame
    Point3d linkLocation;
    if (moveItConfig.pointingFrame == null) {
      linkLocation = new Point3d(0, 0, 1);
      log.warn("Config does not have pointingFrame set. Defaulting to (0,0,1).");
    } else {
      String linkName = moveItConfig.pointingFrame;
//      if (!TRADE.isAvailable("getTransform")) {
//        log.warn("No getTransform service available in the system. Start a TFComponent.");
//        return false;
//      }
      Matrix4d linkTransform = null;
      try {
        TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTransform"));
        linkTransform = tsi.call(Matrix4d.class, moveItConfig.baseLinkString, linkName);

      } catch (TRADEException e) {
        log.error("Error calling getTransform.", e);
        return new ConditionJustification(false, Factory.createPredicate("received(transform)"));
      }
      linkLocation = new Point3d(linkTransform.m03, linkTransform.m13, linkTransform.m23);
    }
    log.debug(String.format("[pointTo] linkLocation %s.", linkLocation.toString()));

    //calc unit vector pointing from link to target
    Vector3d dirVec = new Vector3d(location);
    dirVec.sub(linkLocation);
    double targetDist = dirVec.length();  //from linkLoc to targetObject
    dirVec.normalize();

    //calc goal location of hand
    //scale length based on distance of object
    double goalArmDistance = 0.65 * targetDist;
    goalArmDistance = (goalArmDistance > moveItConfig.armReach) ? moveItConfig.armReach : goalArmDistance; //meters -- should be <= arm length
    Point3d goalLocation = new Point3d();
    goalLocation.x = linkLocation.x + goalArmDistance * dirVec.x;
    goalLocation.y = linkLocation.y + goalArmDistance * dirVec.y;
    goalLocation.z = linkLocation.z + goalArmDistance * dirVec.z;

    //calc orientation
    // TODO: consider calculating the new y-axis (or z-axis) orthogonal to new x-axis
//    Rotation rotation = new Rotation(new Vector3D(1,0,0), new Vector3D(0,1,0), new Vector3D(dirVec.x, dirVec.y, dirVec.z), new Vector3D());
    Rotation rotation = new Rotation(new Vector3D(dirVec.x, dirVec.y, dirVec.z), new Vector3D(1, 0, 0));
    Quat4d newQuat = new Quat4d(rotation.getQ1(), rotation.getQ2(), rotation.getQ3(), rotation.getQ0());
    log.debug(String.format("[pointTo] quat calculated quat: %s.", newQuat.toString()));

    //point to 3D location
    log.debug(String.format("[pointTo] sending %s to %s.", groupName, goalLocation.toString()));
    return moveTo(groupName, goalLocation, newQuat);
  }

  @Override
  public Justification pointTo(String groupName, Symbol refId) {
    //TODO:brad: this was previously calling the local convertToType method, which is part of the pose consultant, not the vision consultant. replacing with call to rr
//    Point3d location = convertToType(refId,Point3d.class);

    Point3d location = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      location = tsi.call(Point3d.class, refId, Point3d.class);

    } catch (TRADEException e) {
      log.error("[pointTo] exception getting memory object from reference, returning null", e);
    }

    // point to the center of the object
    return pointTo(groupName, location);
  }

  protected boolean notOneArm(String groupName) {
    // If the group name provided deals with more than one arm, this function should return true.
    // At the time of writing, the only group where this is known to be the case is in "arms".
    return groupName.equalsIgnoreCase("arms"); // || groupName.equalsIgnoreCase("some_other_two_armed_group_name");, etc
  }

  private boolean moveToGrasp(String groupName, Quat4d orient, Point3d point, float grasp_offset) {
    log.debug("[moveToGrasp] point: " + point + " orient: " + orient + " offset: " + grasp_offset);
    // TODO: is using (-1,0,0) general or does the eeTransform need to be used here?
    point = MoveItHelper.calcTargetOffset(point, orient, new Vector3d(-1.0, 0.0, 0.0), grasp_offset);
    return moveTo(groupName, point, orient).getValue();
  }

  private boolean moveToPinchTogetherGrasp(String groupName, Grasp grasp, float grasp_offset) {
    if (notOneArm(groupName)) {
      log.error("[moveToPinchTogetherGrasp] invalid group name and grasp type combination: " + groupName);
      return false;
    }
    return moveToGrasp(groupName, grasp.getOrientation(), new Point3d(grasp.getPoint(0)), grasp_offset);
  }

  private boolean moveToPinchApartGrasp(String groupName, Grasp grasp, float grasp_offset) {
    if (notOneArm(groupName)) {
      log.error("[moveToPinchApartGrasp] invalid group name and grasp type combination: " + groupName);
      return false;
    }
    // use point in the middle of two target points
    Point3d point = new Point3d((grasp.getPoint(0).x + grasp.getPoint(1).x) / 2.0,
            (grasp.getPoint(0).y + grasp.getPoint(1).y) / 2.0,
            (grasp.getPoint(0).z + grasp.getPoint(1).z) / 2.0);

    return moveToGrasp(groupName, grasp.getOrientation(), point, grasp_offset);
  }

  @Override
  public Justification moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
    log.debug("[moveTo(String, Point3d, Quat4d, Point3d, Quat4d)] method entered.");
    log.debug(String.format("[moveTo] \n\t\t point_l: %s \n\t\t orient_l: %s \n\t\t point_r: %s \n\t\t orient_r: %s \n\t\t",
            point_l, orientation_l, point_r, orientation_r));

    groupName = groupNameSafetyClean(groupName);

    Constraints constraints = new Constraints();

    String effectorLink = endEffectorNames.get(groupName);
    if (effectorLink == null) {
      log.error("groupName does not appear to be valid (no associated end effector to plan for)");
      return new ConditionJustification(false, Factory.createPredicate("valid(groupName)"));
    } else if (notOneArm(groupName)) {
      // This is a special case in which we need two independent arm movements.
      // TODO: there's no way to use ee_transform for more than one right at the moment
      Constraints tmpConstraints = MoveItHelper.makeGoalConstraints(convertToPose(point_l, orientation_l),
              moveItConfig.positionTolerance, moveItConfig.angleTolerance,
              endEffectorNames.get("left_arm"), moveItConfig.baseLinkString);
      MoveItHelper.appendGoalConstrains(constraints, tmpConstraints);
      tmpConstraints = MoveItHelper.makeGoalConstraints(convertToPose(point_r, orientation_r),
              moveItConfig.positionTolerance, moveItConfig.angleTolerance,
              endEffectorNames.get("right_arm"), moveItConfig.baseLinkString);
      MoveItHelper.appendGoalConstrains(constraints, tmpConstraints);

    } else { // This is the standard case, in which there is only one movement to be planned and executed.
      // Default to being left-handed, unless the left parameters are negative.
      Point3d point = (point_l == null) ? point_r : point_l;
      Quat4d quat = (orientation_l == null) ? orientation_r : orientation_l;

      // apply end-effector transform
      Matrix4d final_transform = new Matrix4d();
      final_transform.setIdentity();
      Matrix4d pose_transform = edu.tufts.hrilab.util.Convert.convertToMatrix4d(point, quat);
//      Matrix4d ee_transform = new Matrix4d(eeTransform);
//      ee_transform.invert();
      final_transform.mul(pose_transform);
//      final_transform.mul(ee_transform);
      Pose pose = convertToPose(final_transform);

      Constraints tmpConstraints = MoveItHelper.makeGoalConstraints(pose, moveItConfig.positionTolerance,
              moveItConfig.angleTolerance, effectorLink, moveItConfig.baseLinkString);
      MoveItHelper.appendGoalConstrains(constraints, tmpConstraints);
    }

    Constraints pathConstraints = null;
    if (applyCarryingConstraints) {
      log.error("[moveTo]: attempting to use 'carrying' path constraints!");
      //Get current position as a pose in the base frame
      Pair<Point3d, Quat4d> curPose = getPose(endEffectorNames.get(groupName));
      //Matrix4d final_transform = new Matrix4d();
      //final_transform.setIdentity();
      //Matrix4d pose_transform = Convert.convertToMatrix4d(curPose.getLeft(), curPose.getRight());
      //Matrix4d ee_transform = new Matrix4d(eeTransform);
      //ee_transform.invert();
      //final_transform.mul(pose_transform);
      //final_transform.mul(ee_transform);
      //Pose curPoseConverted = Convert.convertToPose(final_transform);
      Pose curPoseConverted = Convert.convertToPose(curPose.getLeft(), curPose.getRight());

      pathConstraints = MoveItHelper.makeCarryingPathConstraints(curPoseConverted,
              Math.PI / 3.0, endEffectorNames.get(groupName), moveItConfig.baseLinkString);
    }

    List<Constraints> goalConstraints = new ArrayList<>();
    goalConstraints.add(constraints);
    JointState jointState = jointStateSub.getCurrentJointState();
    // FIXME: change this to check gripper links? there is an issue with getting joint state, where it returns state of gripper links
    while (jointState.getName().size() == 2) {
      jointState = jointStateSub.getCurrentJointState();
    }
    log.debug("moveit making plan request");
    edu.tufts.hrilab.diarcros.msg.moveit_msgs.MotionPlanRequest req = MoveItHelper.makeRequest(groupName,
            goalConstraints,
            pathConstraints,
            null,
            Arrays.asList(jointNames.get(groupName)),
            jointState,
            getRobotState(),
            moveItConfig.maxPlanningTime,
            moveItConfig.maxMoveToAttempts);
    log.debug("finished moveit plan request");

    boolean motionPlanResult = executeMotionPlanRequest(req);

    //Todo: Will: Hack-ish double check to deal with the fact that moveit action servers seem to return early at low
    //robot speeds
    if (motionPlanResult) {
      boolean checked = false;
      int checkAttempts = 0;
      int checkMax = 30;
      Pair<Point3d, Quat4d> lastPose = getPose(endEffectorNames.get(groupName));
      while (!checked) {
        log.debug("Checking!!");
        Util.Sleep(200);
        Pair<Point3d, Quat4d> curPose = getPose(endEffectorNames.get(groupName));
        if (checkAttempts < checkMax) {
          double wDelta = Math.abs(curPose.getRight().w - lastPose.getRight().w);
          double xDelta = Math.abs(curPose.getRight().x - lastPose.getRight().x);
          double yDelta = Math.abs(curPose.getRight().y - lastPose.getRight().y);
          double zDelta = Math.abs(curPose.getRight().z - lastPose.getRight().z);
          if (curPose.getLeft().distance(lastPose.getLeft()) < 0.005 && wDelta + xDelta + yDelta + zDelta < 0.001) {
            log.debug("Done Checking!!");
            break;
          } else {
            lastPose = curPose;
            checkAttempts++;
          }
        } else {
          log.error("Took too long to halt after motion plan returned true, returning false.");
          return new ConditionJustification(false);
        }
      }
    }
    return new ConditionJustification(motionPlanResult);
  }

  @Override
  public Justification moveTo(String groupName, Point3d point, Quat4d orientation) {
    log.debug("Entered MoveItComponent moveTo(String, Point3d, Quat4d)");
    groupName = groupNameSafetyClean(groupName);

    if (!Arrays.asList(groupNames).contains(groupName)) {
      log.error("[moveTo] {} is not a valid groupName.", groupName);
      return new ConditionJustification(false, Factory.createPredicate("valid(groupName)"));
    }

    if (groupName.equalsIgnoreCase("arms")) {
      log.warn("Single pose sent for two arms. Applying a y offset between arms. Consider using one of the overloaded moveTo methods, this function is being used improperly.");
      // arbitrary offset of 20 cm
      float y_offset = 0.2f;
      Point3d point_r = new Point3d(point.x, point.y - y_offset, point.z);
      Point3d point_l = new Point3d(point.x, point.y + y_offset, point.z);
      return moveTo(groupName, point_r, orientation, point_l, orientation);
    } else {
      return moveTo(groupName, point, orientation, null, null);
    }
  }

  @Override
  public Justification moveToRelative(String groupName, Point3d point, Quat4d orientation) {
    groupName = groupNameSafetyClean(groupName);

    // TODO: this isn't sufficient, as all arms need to move at the same time
    if (notOneArm(groupName)) {
      // This recursion is safe because left_arm and right_arm never satisfy notOneArm().
      Justification left_val = moveToRelative("left_arm", point, orientation);
      Justification right_val = moveToRelative("right_arm", point, orientation);
      return new ConditionJustification(left_val.getValue() && right_val.getValue());
    }

    // get the current pose of the desired end effector
    Pair<Point3d, Quat4d> posePair = getEEPose(groupName);
    Pose curr_link_pose = Convert.convertToPose(posePair.getLeft(), posePair.getRight());

    // perform rotation (with no translation)
    Pose rel_pose_rotation = new Pose(new Point(0, 0, 0), new Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    Pose new_pose_rotation = Convert.mulPoses(rel_pose_rotation, curr_link_pose);

    // perform translation (with no rotation)
    Point new_pose_translation = new Point(curr_link_pose.getPosition().getX() + point.x,
            curr_link_pose.getPosition().getY() + point.y,
            curr_link_pose.getPosition().getZ() + point.z);

    //the link pose is the translated and rotation link position in base link frame
    Pose goal_link_pose = new Pose(new_pose_translation, new_pose_rotation.getOrientation());

    // apply ee_transform
    Matrix4d final_transform = new Matrix4d();
    final_transform.setIdentity();
    Matrix4d pose_transform = Convert.convertToMatrix4d(goal_link_pose);
    final_transform.mul(pose_transform);
    final_transform.mul(eeTransform);
    Pose pose = convertToPose(final_transform);

//    return moveTo(group_name, Convert.convertToPoint3d(pose.getPosition()), Convert.convertToQuat4d(pose.getOrientation()), null, null, true);

    //Todo: Will: Perhaps make this back up behavior configurable? It doesn't seem like we'd always want this to be a
    //back up plan
    // Go to the computed goal. Our general usage with moveToRelative is that we want a cartesian plan.
    Justification result = moveToCartesian(groupName, Convert.convertToPoint3d(pose.getPosition()), Convert.convertToQuat4d(pose.getOrientation()));
    if (!result.getValue()) {
      log.warn("[moveToRelative] moveToCartesian failed. Attempting non-cartesian moveTo.");
      result = moveTo(groupName, Convert.convertToPoint3d(pose.getPosition()), Convert.convertToQuat4d(pose.getOrientation()));
      if (!result.getValue()) {
        log.warn("[moveToRelative] non-cartesian moveTo also failed.");
      }
    }

    return result;
  }

  @Override
  public Pair<Point3d, Quat4d> getEEPose(String group_name) {
    Pair<Point3d, Quat4d> pose = getPose(endEffectorNames.get(group_name));

    // apply inverse end-effector transform
    Matrix4d final_transform = new Matrix4d();
    final_transform.setIdentity();
    Matrix4d pose_transform = edu.tufts.hrilab.util.Convert.convertToMatrix4d(pose.getLeft(), pose.getRight());
    Matrix4d ee_transform = new Matrix4d(eeTransform);
    final_transform.mul(pose_transform);
    final_transform.mul(ee_transform);
    Pose ee_adjusted_pose = Convert.convertToPose(final_transform);
    return Convert.convertToPose(ee_adjusted_pose);
  }

  @Override
  public Justification moveToCartesian(String groupName, Point3d point, Quat4d orientation) {
    log.debug("Starting moveToCartesian");
    groupName = groupNameSafetyClean(groupName);

    GetCartesianPathResponse response = new GetCartesianPathResponse();
    Pair<Point3d, Quat4d> posePair = getEEPose(groupName);
    Pose currentPose = Convert.convertToPose(posePair.getLeft(), posePair.getRight());

    // apply ee_transform to target pose (i.e., )
    Matrix4d final_transform = new Matrix4d();
    final_transform.setIdentity();
    Matrix4d pose_transform = edu.tufts.hrilab.util.Convert.convertToMatrix4d(point, orientation);
    final_transform.mul(pose_transform);
    final_transform.mul(eeTransform);
    Pose target_pose = convertToPose(final_transform);

    GetCartesianPathRequest request = MoveItHelper.makeCartesianPath(
            currentPose,
            target_pose,
            endEffectorNames.get(groupName),
            moveItConfig.baseLinkString,
            groupName);
    if (!moveGroup.callComputeCartesianPath(request, response)) {
      // Computation failed, so return false before trying the rest here
      return new ConditionJustification(false, Factory.createPredicate("found(path)"));
    }

    JointTrajectory plan = response.getSolution().getJointTrajectory();
    //MoveItHelper.smoothCartesianRotation(plan);

    try {
      moveGroup.executeKinematicPath(new RobotTrajectory(plan, new MultiDOFJointTrajectory()));
    } catch (InterruptedException | RosException e) {
      log.error("Cartesian path execution failed: " + e);
      return new ConditionJustification(false, Factory.createPredicate("execute(cartesianPath)"));
    }

    return new ConditionJustification(true);
  }

  @Override
  public Justification graspObject(String groupName, Symbol refId, float position) {
    boolean result = true;

    // move gripper(s)
    log.debug("Starting gripper movement in grasp object.." + position);
    moveGripper(groupName, position); // Check status of, and make less strict

    MemoryObject mo = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      mo = tsi.call(MemoryObject.class, refId, MemoryObject.class);

    } catch (TRADEException e) {
      log.error("[graspObject] exception getting memory object from reference, returning null", e);
    }

    if (mo != null) {
      attachCollisionObject(endEffectorNames.get(groupName), mo); // Attach a collision object
      setObjectCollisions(refId.getName(), true); // Allow collisions with grippers
      graspedObjects.put(refId, Boolean.TRUE);
    } else {
      log.error("[graspObject] could not convert ref to MemoryObject. Not attaching collision object.");
      graspedObjects.put(refId, Boolean.FALSE);
    }

    // NOTE: currently can't fail (i.e., results always true)
    return new ConditionJustification(result, Factory.createPredicate("succeeded(graspObject)"));
  }

  @Override
  public Justification releaseObject(String groupName, Symbol refId, float position) {
    log.debug("Release object called");

    if (!graspedObjects.containsKey(refId)) {
      log.error("[releaseObject] Trying to release object that hasn't been grasped. RefId: " + refId);
      return new ConditionJustification(false, Factory.createPredicate("grasping", refId));
    }

    if (position > moveItConfig.maxGripperPosition) {
      log.warn("[releaseObject] gripper cannot open to position " + position + ". Setting to maxGripperPosition " + moveItConfig.maxGripperPosition);
      position = moveItConfig.maxGripperPosition;
    }

    // move gripper(s)
    Justification result = moveGripper(groupName, position);
    log.debug(String.format("Finished releasing grippers by %f in releaseObject with result: %b", position, result));

    if (graspedObjects.get(refId)) {
      detachCollisionObject(endEffectorNames.get(groupName), refId);
      log.debug("[releaseObject] Finished detaching collision object.");
    }
    graspedObjects.remove(refId);
    return result;
  }

  public boolean pushObject(String group_name, float position, MemoryObject object, Pose destination) {
    /*
     First, let's compute a vector between where we are and where we want to be.
     We're going to make the assumption that we're on a flat-ish,
     level-ish surface (i.e., a table), so we assume a constant z.
    */
    // Get our object's position and orientation
    Pose objectToVisionTransform = convertToPose(object.getBaseTransform());
    // Figure out the orientation necessary to have the gripper 'pointing' in the
    // direction of the destination
    double final_x = objectToVisionTransform.getPosition().getX();
    double start_x = objectToVisionTransform.getPosition().getY();
    double final_y = destination.getPosition().getX();
    double start_y = destination.getPosition().getY();

    final_x -= start_x;
    final_y -= start_y;
    double final_z = 0;

    Quat4d quat = edu.tufts.hrilab.util.Convert.eulerToQuat4d(final_x, final_y, final_z);

    // Find a point sufficiently far away from the starting object from which to begin the push

    // TODO-- not done with this yet, will likely use in the learning action recovery demo

    return false;
  }

  public Pair<Point3d, Quat4d> getPose(String link_name) {
    // get pose from ros
    List<String> link_names = new ArrayList<>();
    link_names.add(link_name);
    List<Pose> poses_ros = getPoses(link_names);

    if (poses_ros.isEmpty()) {
      log.error("[getPose] pose not found: " + link_name);
      return null;
    }

    return Convert.convertToPose(poses_ros.get(0));
  }

  protected List<Pose> getPoses(List<String> list_names) {
    return getPoses(list_names, moveItConfig.baseLinkString);
  }

  protected List<Pose> getPoses(List<String> link_names, String base_link) {
    List<Pose> link_poses = new ArrayList<>();
    GetPositionFKRequest gpfk_req = new GetPositionFKRequest();
    GetPositionFKResponse gpfk_res = new GetPositionFKResponse();
    gpfk_req.setFkLinkNames(link_names);
    gpfk_req.setHeader(new Header(0, new Time(), base_link));

    gpfk_req.setRobotState(getRobotState());

    moveGroup.callComputeFk(gpfk_req, gpfk_res);

    gpfk_res.getPoseStamped().forEach(p -> link_poses.add(p.getPose()));
    return link_poses;
  }

  protected RobotState getRobotState() {
    GetPlanningSceneRequest gps_req = new GetPlanningSceneRequest();
    GetPlanningSceneResponse gps_res = new GetPlanningSceneResponse();
    gps_req.setComponents(new PlanningSceneComponents(PlanningSceneComponents.ROBOT_STATE));
    try {
      moveGroup.callGetPlanningScene(gps_req, gps_res);
    } catch (
            NullPointerException np) { // Sometimes move_group doesn't connect to the service for that ^, so maybe this will work?
      return new RobotState(jointStateSub.getCurrentJointState(), null, null, true);
    }
    PlanningScene planning_scene = gps_res.getScene();
    return planning_scene.getRobotState();
  }

  // turn collision avoidance with octomap off: objects viewed by the depth camera can be planned into.
  public void disableCollisionAvoidance() {
    setObjectCollisions("<octomap>", true);
  }

  // turn collision avoidance with the octomap back on
  public void enableCollisionAvoidance() {
    setObjectCollisions("<octomap>", false);
  }

  public void addCollisionObject(Pose pose, String name, String parentLink, float box_x, float box_y, float box_z) {
    log.debug("[addCollisionObject] method entered.");

    CollisionObject co = new CollisionObject();
    co.setOperation(CollisionObject.ADD);
    co.setId(name);
    co.getHeader().setFrameId(parentLink);

    SolidPrimitive box = new SolidPrimitive();
    box.setType(SolidPrimitive.BOX);
    box.setDimensions(new double[]{box_x, box_y, box_z});

    co.setPrimitives(Collections.singletonList(box));
    co.setPrimitivePoses(Collections.singletonList(pose));

    AttachedCollisionObject aco = new AttachedCollisionObject(parentLink, co, Collections.singletonList(parentLink), new JointTrajectory(), 0.1);

    moveGroup.sendAttachedCollisionObject(aco);
  }

  protected void removeCollisionObject(String id) {
    CollisionObject co = new CollisionObject();
    co.setId(id);
    Header h = new Header();
    h.setFrameId(moveItConfig.baseLinkString);
    co.setHeader(h);
    co.setOperation(CollisionObject.REMOVE);
    moveGroup.sendCollisionObject(co);
  }

  protected void setObjectCollisions(String obj_id, Boolean passes_through) {
//  InitializeCollisionMatrix();
    GetPlanningSceneRequest req = new GetPlanningSceneRequest();
    GetPlanningSceneResponse resp = new GetPlanningSceneResponse();
    req.setComponents(new PlanningSceneComponents(1023)); // get EVERYTHING (binary flag 1111111111)
    boolean getSceneResult;
    try {
      getSceneResult = moveGroup.callGetPlanningScene(req, resp);
    } catch (NullPointerException ne) {
      log.error("Call to getPlanningScene in MoveGroup was Null!");
      getSceneResult = false;
    }
    if (getSceneResult) {
      AllowedCollisionMatrix acm = resp.getScene().getAllowedCollisionMatrix();

      //we are filling a symmetrical matrix with whether the object should pass through or not
      log.debug("[setCollisionObject] ACM entry names: " + acm.getEntryNames());
      //log.debug("[setCollisionObject] ACM:\n" + gson.toJson(acm) + "\n\n");
      int obj_index = acm.getEntryNames().indexOf(obj_id);
      log.debug("[setCollisionObject] obj: " + obj_id + " index: " + obj_index);
      if (obj_index == -1) {

        int nnames = acm.getEntryNames().size();
        for (AllowedCollisionEntry entry_val : acm.getEntryValues()) {

          boolean[] vals = Arrays.copyOf(entry_val.getEnabled(), nnames + 1);
          vals[nnames] = passes_through;
          entry_val.setEnabled(vals);
        }

        boolean[] object_entry_vals = new boolean[nnames + 1];
        Arrays.fill(object_entry_vals, passes_through);
        object_entry_vals[nnames] = false;
        AllowedCollisionEntry object_entry = new AllowedCollisionEntry();
        object_entry.setEnabled(object_entry_vals);
        acm.getEntryValues().add(object_entry);
        acm.getEntryNames().add(obj_id);

      } else {
        //the row values
        for (AllowedCollisionEntry entry_val : acm.getEntryValues()) {
          entry_val.getEnabled()[obj_index] = passes_through;
        }

        //the column values
        boolean[] entry_values = acm.getEntryValues().get(obj_index).getEnabled();
        Arrays.fill(entry_values, passes_through);
        entry_values[obj_index] = false; //objects never pass through themselves

        acm.getEntryValues().get(obj_index).setEnabled(entry_values);
      }

      resp.getScene().setAllowedCollisionMatrix(acm);
      resp.getScene().setIsDiff(true);
      try {
        moveGroup.sendPlanningScene(resp.getScene());
      } catch (Exception e) {
        log.error("[setObjectCollisions]", e);
      }

    }
  }

  protected boolean removeAllCollisionObjects() {
    GetPlanningSceneRequest req = new GetPlanningSceneRequest();
    GetPlanningSceneResponse resp = new GetPlanningSceneResponse();
    req.setComponents(new PlanningSceneComponents(8));
    if (moveGroup.callGetPlanningScene(req, resp)) {
      List<CollisionObject> cos = resp.getScene().getWorld().getCollisionObjects();
      for (CollisionObject co : cos) {
        String id = co.getId();
        log.debug(String.format("Removing collision object %s", id));
        removeCollisionObject(id);
      }

      return true;
    } else {
      log.error("Service connection failed", new ServiceException("/move_group/get_planning_scene"));
      return false;
    }
  }

  protected void attachCollisionObject(String link_name, MemoryObject object, String second_link_name) {
    // TODO: remove object from planning scene before attaching to robot?
    //move_group.sendPlanningScene();

    // Before we do anything, make sure the visual object is in base_link coordinates
    object.transformToBase();

    AttachedCollisionObject attached_collision_object = new AttachedCollisionObject();
    attached_collision_object.setLinkName(link_name);
    attached_collision_object.setWeight(.01);

    CollisionObject collision_object = attached_collision_object.getObject();
    collision_object.getHeader().setFrameId(object.getCoordinateFrame());
    collision_object.setId(Long.toString(object.getTokenId()));
    collision_object.setOperation(CollisionObject.ADD);
    // Fill out primitive shapes info
    List<SolidPrimitive> primitive_shapes = new ArrayList<>();
    List<Pose> primitive_poses = new ArrayList<>();
    SolidPrimitive primitive_shape;
    Pose primitive_pose;
    for (MemoryPrimitive primitive : object.getPrimitives()) {
      primitive_shape = new SolidPrimitive((byte) primitive.shape.ordinal(), primitive.dims);
      primitive_pose = convertToPose(primitive.pose);
      primitive_shapes.add(primitive_shape);
      primitive_poses.add(primitive_pose);
    }
    collision_object.setPrimitivePoses(primitive_poses);
    collision_object.setPrimitives(primitive_shapes);

    // Fill out mesh info
    List<Mesh> meshes = new ArrayList<>();
    Mesh mesh = new Mesh();
    // add mesh vertices from MemoryObject
    for (int i = 0; i < object.getPointCloud().length; ++i) {
      mesh.getVertices().add(new Point(object.getPointCloud()[i][0], object.getPointCloud()[i][1], object.getPointCloud()[i][2]));
    }
    // add mesh triangles from MemoryObject
    for (int i = 0; i < object.getFaceIndices().length; ++i) {
      mesh.getTriangles().add(new MeshTriangle(object.getFaceIndices()[i]));
    }
    meshes.add(mesh);
    collision_object.setMeshes(meshes);

    // Fill out mesh pose info
    List<Pose> mesh_poses = new ArrayList<>();
    Pose mesh_pose = new Pose();
    // Octomap data assumes that it is relative to the base, so this should be a safe assumption to make.
    mesh_pose.setPosition(new Point(0.0, 0.0, 0.0));
    mesh_pose.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
    mesh_poses.add(mesh_pose);

    collision_object.setMeshPoses(mesh_poses);
    if (!second_link_name.equals("")) {
      attached_collision_object.getTouchLinks().add(second_link_name); //allow the object to touch
    }
    moveGroup.sendAttachedCollisionObject(attached_collision_object);
  }

  protected void attachCollisionObject(String link_name, MemoryObject object) {
    attachCollisionObject(link_name, object, "");
  }

  protected void detachCollisionObject(String link_name, Symbol refId) {
    AttachedCollisionObject attached_collision_object = new AttachedCollisionObject();
    attached_collision_object.setLinkName(link_name);

    CollisionObject collision_object = attached_collision_object.getObject();
    collision_object.getHeader().setFrameId(link_name);
    collision_object.setId(refId.getName());
    collision_object.setOperation(CollisionObject.REMOVE);

    moveGroup.sendAttachedCollisionObject(attached_collision_object);
  }

  protected void detachCollisionObject(String id) {
    CollisionObject co = new CollisionObject();
    co.setId(id);
    Header h = new Header();
    h.setFrameId(moveItConfig.baseLinkString);
    co.setHeader(h);
    co.setOperation(CollisionObject.REMOVE);
    AttachedCollisionObject aco = new AttachedCollisionObject();
    aco.setObject(co);
    moveGroup.sendAttachedCollisionObject(aco);
  }

  protected boolean detachAllAttachedCollisionObjects() {
    GetPlanningSceneRequest req = new GetPlanningSceneRequest();
    GetPlanningSceneResponse resp = new GetPlanningSceneResponse();
    req.setComponents(new PlanningSceneComponents(4));
    if (moveGroup.callGetPlanningScene(req, resp)) {
      List<AttachedCollisionObject> acos = resp.getScene().getRobotState().getAttachedCollisionObjects();
      for (AttachedCollisionObject aco : acos) {
        String id = aco.getObject().getId();
        log.debug(String.format("Detaching collision object %s", id));
        detachCollisionObject(id);
      }
      return true;
    } else {
      log.error("Service connection failed", new ServiceException("/move_group/get_planning_scene"));
      return false;
    }
  }

  public boolean executeMotionPlanRequest(MotionPlanRequest req) {

    req.setMaxVelocityScalingFactor(moveItConfig.speedFactor);
    req.setMaxVelocityScalingFactor(moveItConfig.accelFactor);

    GetMotionPlanRequest mp_req = new GetMotionPlanRequest();
    GetMotionPlanResponse mp_res = new GetMotionPlanResponse();

    try {
      mp_req.setMotionPlanRequest(req);
      log.debug("Calling move group...");
      boolean planning_result = moveGroup.callPlanKinematicPath(mp_req, mp_res);
      int error_code = mp_res.getMotionPlanResponse().getErrorCode().getVal();
      log.debug("error code for planning:" + error_code);
      log.debug("Planning result: " + planning_result);
      if (!planning_result || error_code != MoveItErrorCodes.SUCCESS) {
        return false;
      }
    } catch (Exception e) {
      log.error("[executeMotionPlanRequest] callPlanKinematicPath", e);
      return false;
    }

    try {
      return moveGroup.executeKinematicPath(mp_res.getMotionPlanResponse().getTrajectory());
    } catch (InterruptedException | RosException e) {
      log.error("Exception while trying to execute trajectory.", e);
      return false;
    }
  }

  //TODO: this needs to be connected to the learn method to update the component? or at least that logic needs to get transported here
  @Override
  public Justification recordPose(Symbol poseName) {
    GetPlanningSceneRequest gps_req = new GetPlanningSceneRequest();
    GetPlanningSceneResponse gps_res = new GetPlanningSceneResponse();
    gps_req.setComponents(new PlanningSceneComponents(PlanningSceneComponents.ROBOT_STATE));
    boolean result = moveGroup.callGetPlanningScene(gps_req, gps_res);
    if (result) {
      poses.put(poseName.getName(), gps_res.getScene().getRobotState());
    }
    return new ConditionJustification(result);
  }

  @TRADEService
  @Action
  public Justification recordEEPose(Symbol poseName) {
    Pair<Point3d, Quat4d> curPose = getEEPose("manipulator");
    eePoses.put(poseName.getName(), curPose);

    List<Term> props = new ArrayList<>();
    props.add(Factory.createPredicate(poseName.getName(), "X"));
    consultant.addPropertiesHandled(props);

    //Get a new refId from consultant
    List<Variable> vars = new ArrayList<>();
    Variable var = Factory.createVariable("X");
    vars.add(var);
    Map<Variable, Symbol> refIds = consultant.createReferences(vars);

    //Notify consultant we have an instance of new location type
    consultant.assertProperties(refIds.get(var), props);

    //add pose to created reference
    PoseReference poseReference = consultant.getReference(refIds.get(var));
    poseReference.setPose(curPose.getLeft(), curPose.getRight());
    //inform parser of new location
    log.debug("adding pose " + poseName + " to parser dict");
    //TODO reimplement homophone permutations
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("injectDictionaryEntry"));
      tsi.call(void.class, poseName.getName(), "POSE", poseName.getName(), "DEFINITE");
    } catch (TRADEException e) {
      log.error("unable to add dictionary entry for " + poseName, e);
    }
    return new ConditionJustification(true);
  }

  @TRADEService
  @Action
  public Justification goToEEPose(Symbol poseName) {
    Pair<Point3d, Quat4d> goalPose = eePoses.get(poseName.getName());
    if (goalPose == null) {
      log.error("[goToEEPose] Unable to go to ee pose that hasn't been saved yet, exiting.");
      return new ConditionJustification(false, Factory.createPredicate("known(endEffectorPose)"));
    }
    return moveTo("manipulator", goalPose.getLeft(), goalPose.getRight(), null, null);
  }


  @Override
  @OnInterrupt(onCancelServiceCall = "interrupt()", onSuspendServiceCall = "interrupt()")
  public Justification goToPose(Symbol pose_name) {
    return goToPose(moveItConfig.defaultGroupName, pose_name);
  }

  @Override
  @OnInterrupt(onCancelServiceCall = "interrupt()", onSuspendServiceCall = "interrupt()")
  public Justification goToPose(String groupName, Symbol poseName) {
    boolean result = false;
    groupName = groupNameSafetyClean(groupName);

    if (poses.containsKey(poseName.getName())) {

      // ignore world, caster, and gripper joints
      List<String> joints_to_use;
      String jointName[] = jointNames.get(groupName);
      if (jointName == null || jointName.length == 0) {
        log.error("Joint Name list for group " + groupName + " was invalid (empty or null)");
        return new ConditionJustification(false, Factory.createPredicate("known(jointNames)"));
      }

      joints_to_use = Arrays.asList(jointName);

      List<Constraints> goal_constraints = new ArrayList<>();
      Constraints joint_constraints = MoveItHelper.makeJointConstraints(poses.get(poseName.getName()), joints_to_use);
      goal_constraints.add(joint_constraints);

      MotionPlanRequest req = MoveItHelper.makeDefaultMotionPlanRequest(moveItConfig.maxPlanningTime);
      req.setGoalConstraints(goal_constraints);
      req.setGroupName(groupName);

      if (applyCarryingConstraints) {
        Pair<Point3d, Quat4d> gripperPose = getPose(endEffectorNames.get(groupName));
        Constraints oConstraints = MoveItHelper.makeCarryingPathConstraints(Convert.convertToPose(gripperPose.getLeft(), gripperPose.getRight()),
                Math.PI / 3.0, endEffectorNames.get(groupName), moveItConfig.baseLinkString);
        req.setPathConstraints(oConstraints);
      }

      result = executeMotionPlanRequest(req);

    } else {
      log.error("[goToPose] planning scene null.");
    }

    if (!result) {
      log.error("[goToPose] Unable to find and execute plan." + ((configName.equalsIgnoreCase("pr2")) ? " Have you modified your MoveIt! groups to include a whole_body_no_base group containing (arms,torso,head) sub-groups?" : ""));
    }
    return new ConditionJustification(true);
  }

  @Override
  final public Justification savePosesToFile(String filename) {
    try {
      File file = new File(filename);

      // if file doesnt exists, then create it
      if (!file.exists()) {
        if (!file.createNewFile())
          return new ConditionJustification(false); // Something failed somehow, so stop trying
      }

      FileOutputStream fout = new FileOutputStream(file);
      ObjectOutputStream oos = new ObjectOutputStream(fout);
      oos.writeObject(poses);
      oos.flush();
      oos.close();

    } catch (IOException e) {
      log.error("Error writing poses to file.", e);
      return new ConditionJustification(false);
    }
    return new ConditionJustification(true);
  }

  @Override
  final public Justification saveEEPosesToFile(String filename) {
    try {
      File file = new File(filename);

      // if file doesnt exists, then create it
      if (!file.exists()) {
        if (!file.createNewFile())
          return new ConditionJustification(false); // Something failed somehow, so stop trying
      }

      FileOutputStream fout = new FileOutputStream(file);
      ObjectOutputStream oos = new ObjectOutputStream(fout);
      oos.writeObject(eePoses);
      oos.flush();
      oos.close();

    } catch (IOException e) {
      log.error("Error writing poses to file.", e);
      return new ConditionJustification(false);
    }
    return new ConditionJustification(true);
  }

  @Override
  public void loadPosesFromFile(String filename) {
    if (filename == null || filename.isEmpty()) {
      return;
    }
    try {
      File file = new File(filename);
      FileInputStream fin = new FileInputStream(file);
      ObjectInputStream ois = new ObjectInputStream(fin);
      Map<String, RobotState> loadedPoses = (Map<String, RobotState>) ois.readObject();
      poses.putAll(loadedPoses);
    } catch (IOException | ClassNotFoundException e) {
      log.error("Error loading poses from file.", e);
    }
  }

  @Override
  public void loadEEPosesFromFile(String filename) {
    if (filename == null || filename.isEmpty()) {
      return;
    }
    try {
      File file = new File(filename);
      FileInputStream fin = new FileInputStream(file);
      ObjectInputStream ois = new ObjectInputStream(fin);
      Map<String, Pair<Point3d, Quat4d>> loadedPoses = (Map<String, Pair<Point3d, Quat4d>>) ois.readObject();
      eePoses.putAll(loadedPoses);
    } catch (IOException | ClassNotFoundException e) {
      log.error("Error loading poses from file.", e);
    }
  }


  @Override
  public boolean publishPointCloudToRos(byte[] depthData) {
    try {
      log.trace("[publishPointCloud] method entered.");
      byte[] depth;
      try {
        depth = CompressionUtil.decompress(depthData);
      } catch (IOException | DataFormatException e) {
        log.error("Error decompressing vision data.", e);
        return false;
      }
      //Dimension imgSize = visionConn.call(0, "getImageSize", Dimension.class);
      Dimension imgSize = new Dimension(320, 240);
      double[][] points3d = StereoProc.reprojectImageTo3D(depth, imgSize.width, imgSize.height, 580.0);

      ByteBuffer bb = ByteBuffer.allocate(points3d.length * 3 * 4);
      bb.order(ByteOrder.LITTLE_ENDIAN);
      for (double[] point : points3d) {
        for (double val : point) {
          bb.putFloat((float) val);
        }
      }

      ChannelBuffer channelBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, bb.array());

      edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2 pointCloud_msg = new PointCloud2();
      pointCloud_msg.setData(channelBuffer);
      List<PointField> fields = new ArrayList<>();
      fields.add(new PointField("x", 0, PointField.FLOAT32, 1));
      fields.add(new PointField("y", 4, PointField.FLOAT32, 1));
      fields.add(new PointField("z", 8, PointField.FLOAT32, 1));
      pointCloud_msg.setFields(fields);
      pointCloud_msg.setIsBigendian(false);
      pointCloud_msg.setPointStep(4 * 3);
      pointCloud_msg.setRowStep(320 * 4 * 3);
      pointCloud_msg.setIsDense(false);
      pointCloud_msg.setHeight(imgSize.height);
      pointCloud_msg.setWidth(imgSize.width);
      pointCloud_msg.setHeader(new Header(++pcCounter, Time.fromMillis(System.currentTimeMillis()), "head_mount_kinect_rgb_optical_frame"));

      moveGroup.sendPointCloud(pointCloud_msg);

    } catch (Exception e) {
      log.error("[publishPointCloudToRos]", e);
    }
    return true;
  }

  public boolean isMoveGroupConnected() {
    boolean result = (moveGroup != null && moveGroup.isConnected());
    log.debug("[isMoveGroupConnected]: " + result);
    return result;
  }

  public void reconnectToMoveGroup() {
    moveGroup.shutdown();
    connectToMoveGroup();
    moveGroup.waitForNode();
  }

  @Override
  public Justification moveGripper(String groupName, float position) {
    log.debug("[moveGripper] taking " + groupName + " to " + position + ".");

    groupName = groupNameSafetyClean(groupName);
    GenericManipulator gripper = getGripper(groupName);
    if (gripper != null) {
      if (!gripper.moveGripper(position)) {
        return new ConditionJustification(false, Factory.createPredicate("moved(gripper)"));
      }
    } else {
      return new ConditionJustification(false, Factory.createPredicate("known(gripper)"));
    }

    return new ConditionJustification(true);
  }

  /**
   * Helper methdod to get gripper instance for groupName.
   *
   * @param groupName
   * @return
   */
  private GenericManipulator getGripper(String groupName) {
    groupName = groupNameSafetyClean(groupName);
    GenericManipulator gripper = grippers.get(groupName);
    if (gripper == null) {
      log.warn("Gripper for group " + groupName + " not found! Returning null.");
    }
    return gripper;
  }

  @Override
  public boolean moveToJointPosition(String joint, double position) {
    return moveToJointPositions(new String[]{joint}, new double[]{position});
  }

  /**
   * This implementation of recording a trajectory is to save a bunch of trajectory points and remember the order,
   * allowing them to be replayed by going from point to point to point.
   *
   * @param trajectoryName
   */
  @Override
  public void startRecordingTrajectory(final String trajectoryName) {
    new Thread(() -> {
      JointTrajectory jointTrajectory = new JointTrajectory();
      List<JointTrajectoryPoint> jointTrajectoryPoints = new ArrayList<>();

      JointState currentState = jointStateSub.getCurrentJointState();
      jointTrajectory.setJointNames(currentState.getName());

      recordingTrajectory = true;
      while (recordingTrajectory) {
        JointTrajectoryPoint currentPoint = new JointTrajectoryPoint();

        // Note-- acceleration isn't being set here. That's probably OK (we have velocity data), but might not be.
        // It could be calculated by using the known velocities and time between points.
        currentPoint.setPositions(currentState.getPosition());
        currentPoint.setEffort(currentState.getEffort());
        currentPoint.setVelocities(currentState.getVelocity());

        jointTrajectoryPoints.add(currentPoint);
        Util.Sleep(500);
        currentState = jointStateSub.getCurrentJointState(); // refresh the joint state
      }

      log.debug("[startRecordingTrajectory] done recording.");
      if (jointTrajectoryPoints.size() == 0) {
        log.error("Trajectory had no points! '" + trajectoryName + "' will not be saved.");
        return;
      }
      jointTrajectory.setPoints(jointTrajectoryPoints);
      recordedTrajectories.put(trajectoryName, jointTrajectory);

    }).start();
  }

  @Override
  public void stopRecordingTrajectory() {
    recordingTrajectory = false;
  }

  @Override
  public Justification executeTrajectory(String trajectory_name) {
    if (!recordedTrajectories.containsKey(trajectory_name)) {
      log.error("[executeTrajectory] couldn't find a trajectory named '" + trajectory_name + "', so trajectory execution failed.");
      return new ConditionJustification(false, Factory.createPredicate("found(trajectory)"));
    }

    JointTrajectory trajectory = recordedTrajectories.get(trajectory_name);
    trajectory.setHeader(new Header(0, new Time(), moveItConfig.baseLinkString));

    // first plan safely to the starting position (i.e., first joint position in the trajectory)
    Constraints joint_constraints = MoveItHelper.makeJointConstraints(trajectory.getJointNames(), trajectory.getPoints().get(0), trajectory.getJointNames()); // of the JointNames we can use, use all JointNames
    List<Constraints> goal_constraints = new ArrayList<>();
    goal_constraints.add(joint_constraints);
    MotionPlanRequest req = MoveItHelper.makeDefaultMotionPlanRequest(moveItConfig.maxPlanningTime);
    req.setGoalConstraints(goal_constraints);
    req.setGroupName(moveItConfig.defaultGroupName);

    if (!executeMotionPlanRequest(req)) { // HACK-- sometimes we overshoot the point too far, so execute the request twice to make sure we get there.
      if (!executeMotionPlanRequest(req)) {
        log.error("[executeTrajectory] couldn't plan/execute to starting position.");
        return new ConditionJustification(false, Factory.createPredicate("execute(plan)"));
      }
    }

    try {
      // execute recorded trajectory
      ExecuteKnownTrajectoryResponse ex_res = new ExecuteKnownTrajectoryResponse();
      RobotTrajectory rt = new RobotTrajectory(trajectory, new MultiDOFJointTrajectory());
      if (moveGroup.callExecuteKinematicPath(new ExecuteKnownTrajectoryRequest(rt, true), ex_res)) {
        return new ConditionJustification(true);
      }
    } catch (NullPointerException ne) {
      log.warn("[executeTrajectory] Standard trajectory execution failed (threw null), trying fallback.");
    }

    log.warn("[executeTrajectory] Standard trajectory execution failed (returned false), trying fallback.");

    // That failed, so we'll try our fallback approach: calculating how to get to each point, and going there.
    for (JointTrajectoryPoint jtp : trajectory.getPoints()) {
      Constraints currentJointGoal = MoveItHelper.makeJointConstraints(trajectory.getJointNames(), jtp, trajectory.getJointNames());
      List<Constraints> currentConstraint = new ArrayList<>();
      currentConstraint.add(currentJointGoal);

      MotionPlanRequest currentRequest = MoveItHelper.makeMotionPlanRequest(getRobotState(), jointStateSub.getCurrentJointState(), moveItConfig.maxPlanningTime);
      currentRequest.setGoalConstraints(currentConstraint);
      currentRequest.setGroupName(moveItConfig.defaultGroupName);
      currentRequest.setStartState(getRobotState());
      if (!executeMotionPlanRequest(currentRequest) || !executeMotionPlanRequest(currentRequest)) { // Same hack as before: try to get there twice if need be
        log.error("[executeTrajectory] Fallback failed to execute the trajectory.");
        return new ConditionJustification(false, Factory.createPredicate("execute(fallbackTrajectory)"));
      }
    }

    log.debug("Fallback attempt succeeded!");
    return new ConditionJustification(true);
  }

  @Override
  public Justification saveTrajectoriesToFile(String filename) {
    try {
      File file = new File(filename);

      // if file doesn't exist, create it
      if (!file.exists()) {
        if (!file.createNewFile())
          return new ConditionJustification(false); // Unable to create new file
      }

      FileOutputStream fout = new FileOutputStream(file);
      ObjectOutputStream oos = new ObjectOutputStream(fout);
      oos.writeObject(recordedTrajectories);
      oos.flush();
      oos.close();

    } catch (IOException e) {
      log.error("Error writing poses to file.", e);
      return new ConditionJustification(false);
    }
    return new ConditionJustification(true);
  }

  @Override
  public void loadTrajectoriesFromFile(String filename) {
    if (filename == null || filename.isEmpty()) {
      return;
    }
    try {
      File file = new File(filename);
      FileInputStream fin = new FileInputStream(file);
      ObjectInputStream ois = new ObjectInputStream(fin);
      Map<String, JointTrajectory> loadedTrajectories = (Map<String, JointTrajectory>) ois.readObject();
      recordedTrajectories.putAll(loadedTrajectories);
    } catch (IOException | ClassNotFoundException e) {
      log.error("Error loading trajectories from file.", e);
    }
  }

  private String groupNameSafetyClean(String groupName) {
    // If no group name is specified, it may be passed in as null or "none". In that case, we want to correct it to the
    // default group name. If it's a valid group name (has an end effector we can manipulate), then that's OK to return
    // as it was passed in. If it's not valid, we'll need to return just that value.

    if (groupName == null || groupName.equalsIgnoreCase("none")) {
      return moveItConfig.defaultGroupName;  // User specified none/null as a flag to indicate we should use default
    } else if (endEffectorNames.containsKey(groupName)) {
      return groupName;         // User specified a valid group name, so just use it
    } else {
      log.warn("The requested group name (" + groupName + ") has not been configured with an end effector. The default group (" + moveItConfig.defaultGroupName + ") is being used instead.");
      return moveItConfig.defaultGroupName;  // User specified invalid group name, so be safe and use default. Improperly configured files could still fail here, but we've done what we can.
    }
  }

  @Override
  public Justification closeGripper(String groupName) {
    return (moveGripper(groupName, 0.0f));
  }

  @Override
  public Justification openGripper(String groupName) {
    GenericManipulator gripper = getGripper(groupName);
    boolean returnVal = false;
    if (gripper != null) {
      //TODO: Brad/Will - maybe we should move openGripper to the genericManipulator class, or standardize this so
      //there is no confusion between meters and percentage
      returnVal = gripper.moveGripper(gripper.maxGraspWidthMeters);
    }
    return new ConditionJustification(returnVal);
  }

  @Override
  public float getGripperPosition(String groupName) {
    return grippers.get(groupName).getCurrentGripperPosition();
  }

  //PR2 Stuff
  // TODO: can this be handled in moveGripper method?
  public boolean moveGrippers(String group_name, float f) { // moveGrippers (plural) implies two arms, so we assume left and right.
    if (!notOneArm(group_name) || !groupNameSafetyClean(group_name).equalsIgnoreCase(group_name)) {
      group_name = groupNameSafetyClean(group_name); // A bit of redundancy here: we're checking if the group name would be changed before changing it, which allows us to throw a warning.
      log.warn("moveGrippers has been left in for compatability. Single armed devices should be using moveGripper.");
      return grippers.get(group_name).moveGripper(f);
    }
    return grippers.get("left_arm").moveGripper(f) && grippers.get("right_arm").moveGripper(f);
  }

  @Override
  public Justification learn(Term learningTerm) {
    // separate the name and target description
    // should be of form "instanceOf(refId, descriptors)"
    Symbol refId = learningTerm.get(0);
    if (!Utilities.isReference(refId, consultant.getKBName())) {
      log.error("[learn] first argument must be reference: " + learningTerm);
      // TODO: create sensible failure predicate
      return new ConditionJustification(false, Factory.createPredicate("propertyOf(semantics,knownForm)"));
    }


    //TODO:brad: how does this coexist with the record pose primitive?
    // record current pose with name of reference ID
    // get the current pose of the desired end effector
    // TODO: how to generalize this if more than one end-effector?
    Pair<Point3d, Quat4d> pose = getPose(endEffectorNames.get(moveItConfig.defaultGroupName));
    Point3d posePosition = pose.getLeft();
    Quat4d poseOrient = pose.getRight();
    // TODO: is using (1,0,0) general or does the eeTransform need to be used here?
    posePosition = MoveItHelper.calcTargetOffset(posePosition, poseOrient, new Vector3d(1.0, 0.0, 0.0), moveItConfig.graspContactOffset);

    PoseReference poseReference = consultant.getReference(refId);
    poseReference.setPosition(posePosition);
    poseReference.setOrientation(poseOrient);

    // add properties to reference
    // replace variable in descriptors to match references's variable, e.g., knife(VAR5) --> knife(Y)
    List<Term> descriptors = PredicateHelper.convertToVisionForm(learningTerm.get(1));
    List<Variable> descriptorVars = edu.tufts.hrilab.fol.util.Utilities.getUnboundVariables(descriptors);
    if (descriptorVars.size() != 1) {
      log.error("[learn] cannot currently handle descriptors with more than one Variable: " + learningTerm);
      // TODO: create sensible failure predicate
      return new ConditionJustification(false, Factory.createPredicate("propertyOf(semantics,knownForm)"));
    }
    List<Term> convertedDescriptors = new ArrayList<>();
    descriptors.stream().forEach(d -> convertedDescriptors.add(PredicateHelper.replace(d, descriptorVars.get(0), poseReference.variable)));
    //List<Property> properties = Utilities.convertToProperties(convertedDescriptors);
    consultant.assertProperties(refId, convertedDescriptors);

    // ======================== START POWER HACK ================================
    // remove "this(X)" descriptor -- this is to enable learning of multiple references so that in
    // subsequent "this is a ..." utterances, "this" doesn't get resolved to the reference being
    // learned here, and instead will start a new "this" search
    List<Term> oldProperties = Arrays.asList(Factory.createPredicate("this", poseReference.variable));
    consultant.retractProperties(refId, oldProperties);
    // ======================== END POWER HACK ================================

    // update POWER with new concept
    if (!consultant.addPropertiesHandled(convertedDescriptors)) {
      //if (!consultant.addPropertiesHandled(Utilities.convertToPredicates(properties))) {
      // TODO: create sensible failure predicate
      return new ConditionJustification(false, Factory.createPredicate("propertyOf(component, connected)"));
    }

    return new ConditionJustification(true);
  }

  @Override
  public Justification unlearn(Term learningTerm) {
    throw new UnsupportedOperationException("[unlearn] is not yet supported.");
  }

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "interrupt()", onSuspendServiceCall = "interrupt()")
  public Justification pour(boolean forward) {
    String group_name = "manipulator";

    //Todo: Will: very very manual - fix later
    JointState curr = jointStateSub.getCurrentJointState();
    double[] currPositions = curr.getPosition();
    if (forward) {
      currPositions[currPositions.length - 1] += 2.0;
    } else {
      currPositions[currPositions.length - 1] += -2.0;
    }
    curr.setPosition(currPositions);

    // ignore world, caster, and gripper joints
    List<String> joints_to_use;
    String jointName[] = jointNames.get(group_name);
    if (jointName == null || jointName.length == 0) {
      log.error("Joint Name list for group " + group_name + " was invalid (empty or null)");
      return new ConditionJustification(false);
    }

    joints_to_use = Arrays.asList(jointName);

    Constraints joint_constraints = MoveItHelper.makeJointConstraints(new RobotState(curr, null, null, false), joints_to_use);
    List<Constraints> goal_constraints = new ArrayList<>();
    goal_constraints.add(joint_constraints);

    MotionPlanRequest req = MoveItHelper.makeDefaultMotionPlanRequest(moveItConfig.maxPlanningTime);
    req.setGoalConstraints(goal_constraints);
    req.setGroupName(group_name);

    // execute motion plan
    if (executeMotionPlanRequest(req)) {
      return new ConditionJustification(false, Factory.createPredicate("execute(plan)"));
    }

    return new ConditionJustification(true);
  }

  protected boolean moveTo(String groupName, Point3d point, Quat4d orientation, int maxAttempts) {
    int counter = 0;
    boolean succeeded = moveTo(groupName, point, orientation).getValue();
    while (!succeeded && counter < maxAttempts) {
      ++counter;
      succeeded = moveTo(groupName, point, orientation).getValue();
    }
    return succeeded;
  }

  public Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation) {
    log.debug("Starting [pressObject]");

    Vector3d offset_vect = new Vector3d(); // direction and magnitude of offset from object location
    offset_vect.scale(DEFAULT_PRESS_OFFSET, RotationHelpers.quaternionToNormVect(object_orientation)); // actually create the offset vector
    Point3d offset_location = new Point3d();
    offset_location.add(object_location, offset_vect); // Apply the offset

    Quat4d presser_orientation = RotationHelpers.mirrorOrientation(object_orientation); // ensuring that the presser faces towards the object

    if (log.isDebugEnabled()) {
      log.debug("[pressObject]" +
              "\n\t\tObject/presser location: " + object_location +
              "\n\t\tOffset         location: " + offset_location +
              "\n\t\tObject      orientation: " + object_orientation +
              "\n\t\tPresser     orientation: " + presser_orientation);
    }

    Justification failed = new ConditionJustification(false);
    try {
      goToPose(Factory.createSymbol("tuck"));
      if (!(moveTo(group_name, offset_location, presser_orientation, 4))) return failed;
      disableCollisionAvoidance(); // when interacting with objects, collision avoidance should be off
      if (!moveTo(group_name, object_location, presser_orientation, 4)) return failed;// Actually press the object
      if (!(moveTo(group_name, offset_location, presser_orientation, 4))) return failed;
    } finally {
      enableCollisionAvoidance();
      goToPose(Factory.createSymbol("tuck"));
    }

    log.debug("Finished pressObject");
    return new ConditionJustification(true);
  }

  public Justification pressObject(String group_name, Symbol refID) {
    log.info("Starting [pressObject] with refID: " + refID);

    MemoryObject mo;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      mo = tsi.call(MemoryObject.class, refID, MemoryObject.class);

    } catch (TRADEException e) {
      log.error("[pressObject] exception getting memory object from reference, returning null", e);
      return new ConditionJustification(false);
    }

    mo.transformToBase(); // Convert memory object relative to base_link
    Point3d location = mo.getLocation();
    Quat4d orientation = DEFAULT_PRESS_ORIENT;

    return pressObject(group_name, location, orientation);
  }

}
