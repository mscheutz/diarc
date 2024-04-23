/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.diarc.DiarcComponent;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Stream;

import edu.tufts.hrilab.map.PathAction;
import edu.tufts.hrilab.map.util.Pose;
import edu.tufts.hrilab.map.util.Utils;
import edu.tufts.hrilab.util.Convert;
import edu.tufts.hrilab.util.RosPackPathHelper;
import edu.tufts.hrilab.util.resource.Resources;
//import jakarta.servlet.http.HttpServletResponse;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import com.google.common.graph.*;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.consultant.pose.PoseReference;

import ai.thinkingrobots.trade.*;
//import org.springframework.core.io.ClassPathResource;
//import org.springframework.http.MediaType;
//import org.springframework.stereotype.Component;
//import org.springframework.util.StreamUtils;
//import org.springframework.web.bind.annotation.*;
//import org.springframework.web.bind.annotation.GetMapping;
//import org.springframework.web.bind.annotation.RequestParam;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;


public class MapComponent extends DiarcComponent {
  /**
   * Robot's current pose on the current map.
   */
  private Pose currRobotPose = new Pose(new Point3d(0, 0, 0), new Quat4d(0, 0, 0, 1));
  /**
   * Floor number to FloorMap.
   */
  private Map<Integer, FloorMap> floorMaps = new HashMap<>();
  /**
   * Floor map for the current floor.
   */
  private FloorMap currFloorMap = null;
  /**
   * Current floor.
   */
  private int currFloor = Integer.MIN_VALUE;
  /**
   * The robot's starting floor.
   */
  private int initFloor = 1;

  private String mapFolder;

  private String rosPackage = "";

  private RosPackPathHelper rosPackageHelper;

  private PoseConsultant consultant;

  private String poseKBName = "location";

  private String refsConfigFile;

  private String refsConfigDir = "config/edu/tufts/hrilab/map";

  public MapComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("map_folder").hasArg().argName("file").desc("filepath to folder with map files (can be relative path if ros_package is also specified)").build());
    options.add(Option.builder("ros_package").hasArg().argName("package").desc("name of ros package").build());
    options.add(Option.builder("map_folder").hasArg().argName("file").desc("filepath to folder with map files").build());
    options.add(Option.builder("start_floor").hasArg().argName("number").desc("floor the robot is starting on at launch").build());
    options.add(Option.builder("refs").longOpt("references").hasArg().argName("file").desc("load pre-defined object references and their properties").build());
    options.add(Option.builder("kb_name").hasArg().argName("file").desc("consultant kbName (default: location)").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("map_folder")) {
      mapFolder = cmdLine.getOptionValue("map_folder");
    }
    if (cmdLine.hasOption("ros_package")) {
      rosPackage = cmdLine.getOptionValue("ros_package");
      rosPackageHelper = new RosPackPathHelper();
    }
    if (cmdLine.hasOption("start_floor")) {
      initFloor = Integer.parseInt(cmdLine.getOptionValue("start_floor"));
    }
    if (cmdLine.hasOption("refs")) {
      refsConfigFile = cmdLine.getOptionValue("refs");
    }
    if (cmdLine.hasOption("kb_name")) {
      poseKBName = cmdLine.getOptionValue("kb_name");
    }
  }

  @Override
  protected void init() {
    // instantiate pose consultant
    consultant = new PoseConsultant(PoseReference.class, poseKBName, new ArrayList<>());
    try {
      Collection<String> consultantGroups= this.getMyGroups();
      consultantGroups.add(consultant.getKBName());
      TRADE.registerAllServices(consultant, consultantGroups);
    } catch (TRADEException e) {
      log.error("Error registering pose consultant.", e);
    }

    // load pre-defined references
    if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
      String filename = Resources.createFilepath(refsConfigDir, refsConfigFile);
      consultant.loadReferencesFromFile(filename);
    }

    // parse maps
    if (mapFolder != null && !mapFolder.isEmpty()) {
      parseAllMaps();
      setCurrentFloor(initFloor);
    }

    // add all map reference to consultant
    addReferencesFromMaps();
  }

  /**
   * Parse all maps by searching for all JSON files in the target mapFolder.
   */
  private void parseAllMaps() {
    String folderPath = mapFolder;
    if (!rosPackage.isEmpty()) {
      try {
        folderPath = rosPackageHelper.getRosPackPath(rosPackage) + File.separator + mapFolder;
      } catch (FileNotFoundException e) {
        log.error("Could not find ros package: " + rosPackage);
      }
    }

    try (Stream<Path> paths = Files.walk(Paths.get(folderPath))) {
      paths.filter(Files::isRegularFile).filter(path -> path.toString().endsWith(".json"))
              .forEach(path -> {
                FloorMap floorMap = new FloorMap(path);
                floorMaps.put(floorMap.getFloorNumber(), floorMap);
              });
    } catch (IOException e) {
      log.error("Error parsing map files.", e);
    }
  }

  /**
   * Populate the consultant with references from all known map locations.
   *
   * @return
   */
  private void addReferencesFromMaps() {
    Variable var = Factory.createVariable("VAR0", poseKBName);

    for (FloorMap floorMap : floorMaps.values()) {
      Set<MapObject> objects = floorMap.getAllObjects();
      for (MapObject curr : objects) {
        List<Term> properties = new ArrayList<>();
        String property = curr.getProperty();

        // TODO: EAK: why do we need this?
        if (curr.isA("room")) {
          List<MapObject> connections = floorMap.getConnectingRooms(curr);
          if (connections.size() == 1) {
            property = "outside";
          }
        }

        properties.add(Factory.createPredicate(property, var));
        properties.add(Factory.createPredicate("fluent_equals", Factory.createPredicate("on_floor", var), Factory.createSymbol(String.valueOf(curr.getFloor()))));

        // use consultant to create new refernce, and populate it
        PoseReference newRef = consultant.createReference(var, properties);
        newRef.setPose(curr.getPosition(), curr.getOrientation());

        // set refId in MapObject to make for easier refID <--> MapObject conversion
        floorMap.setReferenceId(curr, newRef.refId);
      }
    }
  }

  /**
   * switches the state information to match the new floor
   *
   * @param newFloor
   */
  @Action
  @TRADEService
  public void setCurrentFloor(int newFloor) {
    if (newFloor == currFloor) {
      return;
    }

    if (floorMaps.containsKey(newFloor)) {
      // set local diarc map
      currFloor = newFloor;
      currFloorMap = floorMaps.get(newFloor);

      // set ROS map
      try {
        if (!rosPackage.isEmpty()) {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("changeMap")).call(Boolean.class, rosPackage, currFloorMap.getMapYamlFile().substring(rosPackageHelper.getRosPackPath(rosPackage).length()));
        } else {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("changeMap")).call(Boolean.class, currFloorMap.getMapYamlFile());
        }
      } catch (FileNotFoundException | TRADEException e) {
        log.error("Error attempting to change underlying map.", e);
      }

      // TODO: set robot pose in new map
//      Pose newPose = new Pose();
//      try {
//        TRADE.callThe("setPoseGlobal", newPose.getPosition().x, newPose.getPosition().y, Utils.quat4dToEuler(newPose.getOrientation())[2]);
//      } catch (TRADEException e) {
//        log.error("Error setting global pose after switching floors.", e);
//      }
    } else {
      log.error("No floor plan for floor exists: " + newFloor);
    }
  }

  /**
   * get the current floor the robot is on
   *
   * @return the current floor of the robot
   */
  @TRADEService
  public int getCurrFloor() {
    return currFloor;
  }

  /**
   * Get the path from the curr pose to the destination refId.
   * This assumes that both curr and dest are on the same floor.
   *
   * @param curr         - current pose (meters)
   * @param dest         - destination pose (meters)
   * @param padding      - padding away from doors (meters)
   * @param canOpenDoors - whether the robot can open doors
   * @return
   */
  @TRADEService
  public List<PathAction> getPath(Matrix4d curr, Symbol dest, double padding, boolean canOpenDoors) {
    PoseReference dstRef = consultant.getReference(dest);
    if (dstRef == null) {
      log.error("Reference is not known: " + dest);
      return new ArrayList<>();
    }

    // if only JSON was loaded, and no maps, just return the reference pose
    if (floorMaps.isEmpty()) {
      List<PathAction> path = new ArrayList<>();
      path.add(new PathAction(PathAction.Type.ACTION_GOTO, dest, Convert.convertToMatrix4d(dstRef.getPosition(), dstRef.getOrientation())));
      return path;
    }

    // try get MapObject from reference
    MapObject dstMapObject = currFloorMap.getMapObjectFromReferenceId(dest);
    boolean appendDestPose = false;
    if (dstMapObject == null) {
      // if dest was not generated from the colored map (i.e., reference generated from a JSON file), look up MapObject
      // via ref's location. Need to append the dest pose in this case bc the MapObject pose will likely not match
      // the ref's pose
      dstMapObject = currFloorMap.getMapObjectAt(dstRef.getPosition());
      appendDestPose = true;
    }

    // plan a path
    MapObject startMapObject = currFloorMap.getMapObjectAt(Utils.convertToPose(curr).getPosition());
    List<MapObject> mapObjectPath = PathPlanner.getRoomPath(currFloorMap, startMapObject, dstMapObject);
    List<PathAction> path = PathPlanner.convertRoomPathToPathActions(currFloorMap, mapObjectPath, canOpenDoors, padding);
    if (appendDestPose) {
      path.add(new PathAction(PathAction.Type.ACTION_GOTO, dest, Convert.convertToMatrix4d(dstRef.getPosition(), dstRef.getOrientation())));
    }
    return path;
  }

  /**
   * Marks a portal between two rooms as either closed or open.
   *
   * @param portal  ref id of portal to mark as open/closed
   * @param closed boolean specifying if door is open or closed; true is closed, false is open
   */
  @TRADEService
  public void markDoor(Symbol portal, boolean closed) {
    currFloorMap.markDoor(portal, closed);
  }

  /**
   * Gets the pose of the robot and sets the local pose .
   */
  @TRADEService
  public void updateRobotPose() {

    try {
      double[] poseQuat = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPoseGlobalQuat")).call(double[].class);
      currRobotPose = new Pose(new Point3d(poseQuat[0], poseQuat[1], 0.0), new Quat4d(poseQuat[2], poseQuat[3], poseQuat[4], poseQuat[5]));
    } catch (TRADEException e) {
      log.error("Unable to set robot pose", e);
    }
  }

  /**
   * Get the two end points of a door
   *
   * @param door Name of door
   * @return A list with index 0 being one side of the door, and index 1 the other
   */
  @TRADEService
  public List<Point2d> getDoorEdges(Symbol door) {
    // get MapObject from ref
    MapObject doorObj = currFloorMap.getMapObjectFromReferenceId(door);

    double maxDist = 0;
    int idxP1 = 0;
    int idxP2 = 0;

    List<Point2d> doorPix = doorObj.getPixels();
    List<Point2d> doorMeters = new ArrayList<Point2d>();
    // converting from pixel coordinates to ros coordinates
    for (int i = 0; i < doorPix.size(); i++) {
      Point2d point = doorPix.get(i);
      Point2d newPoint = new Point2d();
      Point3d in3d = currFloorMap.getPixelMap().toMeter(point);
      newPoint.x = in3d.x;
      newPoint.y = in3d.y;
      doorMeters.add(newPoint);
    }

    for (int i = 0; i < doorMeters.size() - 1; i++) {
      Point2d p1 = doorMeters.get(i);
      for (int j = i + 1; j < doorMeters.size(); j++) {
        Point2d p2 = doorMeters.get(j);
        double distance = p1.distance(p2);
        if (distance > maxDist) {
          maxDist = distance;
          idxP1 = i;
          idxP2 = j;
        }
      }
    }

    List<Point2d> out = new ArrayList<>();
    out.add(doorMeters.get(idxP1));
    out.add(doorMeters.get(idxP2));
    return out;
  }

  /****************************************************************************
   *         ELEVATOR SPECIFIC METHODS
   *         TODO: can these be generalized?
   ***************************************************************************/

  /**
   * TODO: HACK: remove this method once we use the planner for multi-floor navigation
   *
   * @return
   */
  @TRADEService
  @Action
  public Justification goToNearestElevator() {
    List<Symbol> doors = getNearestElevatorDoors();
    if (doors.size() < 1) {
      log.error("No elevator door found");
      return new ConditionJustification(false);
    }
    Matrix4d transform = getElevatorButtonPose(doors.get(0), 0.5);
    Pose pose = Utils.convertToPose(transform);
    try {
      //double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait
      return  TRADE.getAvailableService(new TRADEServiceConstraints().name("goToLocation").argTypes(double.class,double.class,double.class,double.class,double.class,double.class,Boolean.class)).call(Justification.class, pose.getPosition().x, pose.getPosition().y,
              pose.getOrientation().x, pose.getOrientation().y, pose.getOrientation().z, pose.getOrientation().w,
              true);
    } catch (TRADEException e) {
      log.error("Error calling goToLocation.", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public List<Matrix4d> getElevatorEnterPosePath(Symbol elevatorDoor, double padding) {
    // get relevant map objects
    MapObject door = currFloorMap.getMapObjectFromReferenceId(elevatorDoor);
    MapObject elevator = getElevatorFromDoor(door);
    MapObject lobby = getLobbyFromDoor(door);

    List<Matrix4d> path = new ArrayList<>();
    path.add(currFloorMap.getPortalFromRoomPose(door, lobby, true, padding));
    path.add(currFloorMap.getPortalFromRoomPose(door, elevator, false, padding));
    path.add(getElevatorPanelPose(door, padding));
    return path;
  }

  @Action
  @TRADEService
  public List<Matrix4d> getElevatorExitPosePath(Symbol elevatorDoor, double padding) {
    // get relevant map objects
    MapObject door = currFloorMap.getMapObjectFromReferenceId(elevatorDoor);
    MapObject elevator = getElevatorFromDoor(door);

    List<Matrix4d> path = new ArrayList<>();
    path.add(currFloorMap.getPortalFromRoomPose(door, elevator, true, padding));
    path.add(getElevatorExitPose(door, padding));
    return path;
  }

  /**
   * Gets pose of robot on the elevator facing the inside button panel.
   *
   * @param elevatorDoor reference of elevator door
   * @param padding      how far pose should be from panel
   * @return Pose of robot if it were to be standing in front the elevator panel.
   */
  @Action
  @TRADEService
  public Matrix4d getElevatorPanelPose(Symbol elevatorDoor, double padding) {
    return getElevatorPanelPose(currFloorMap.getMapObjectFromReferenceId(elevatorDoor), padding);
  }

  /**
   * Gets pose of robot on the elevator facing the inside button panel.
   *
   * @param elevatorDoor elevator door MapObject
   * @param padding      how far pose should be from panel
   * @return Pose of robot if it were to be standing in front the elevator panel.
   */
  private Matrix4d getElevatorPanelPose(MapObject elevatorDoor, double padding) {
    MapObject elevator = getElevatorFromDoor(elevatorDoor);
    MapObject panel = currFloorMap.getPanelInElevator(elevator);
    return currFloorMap.getPortalFromRoomPose(panel, elevator, true, padding);
  }

  /**
   * Get the pose needed to exit the elevator into the elevator lobby.
   *
   * @param elevatorDoor reference to elevator door
   * @return
   */
  @Action
  @TRADEService
  public Matrix4d getElevatorExitPose(Symbol elevatorDoor, double padding) {
    return getElevatorExitPose(currFloorMap.getMapObjectFromReferenceId(elevatorDoor), padding);
  }

  /**
   * Get the pose needed to exit the elevator into the elevator lobby.
   *
   * @param elevatorDoor elevator door MapObject
   * @return
   */
  private Matrix4d getElevatorExitPose(MapObject elevatorDoor, double padding) {
    MapObject lobby = getLobbyFromDoor(elevatorDoor);
    return currFloorMap.getPortalFromRoomPose(elevatorDoor, lobby, false, padding);
  }

  /**
   * Get the pose needed to press the up/down button for the elevator door.
   *
   * @param elevatorDoor reference to elevator door
   * @return
   */
  @Action
  @TRADEService
  public Matrix4d getElevatorButtonPose(Symbol elevatorDoor, double padding) {
    return getElevatorButtonPose(currFloorMap.getMapObjectFromReferenceId(elevatorDoor), padding);
  }

  /**
   * Get the pose needed to press the up/down button for the elevator door.
   *
   * @param elevatorDoor elevator door MapObject
   * @return
   */
  private Matrix4d getElevatorButtonPose(MapObject elevatorDoor, double padding) {
    MapObject lobby = getLobbyFromDoor(elevatorDoor);
    MapObject button = currFloorMap.getButtonFromDoor(elevatorDoor);
    return currFloorMap.getPortalFromRoomPose(button, lobby, true, padding);
  }

  /**
   * gets a list of the MapObjects of the nearest elevator doors
   *
   * @return a list of MapObjects representing the elevator doors
   */
  private List<MapObject> getNearestElevatorDoorObjects() {
    MapObject nearestButton = currFloorMap.getNearestElevatorButton(currRobotPose.getPosition());
    log.debug("nearest button: " + String.valueOf(nearestButton.getPosition().x) + ", " + String.valueOf(nearestButton.getPosition().x));
    List<MapObject> doors = currFloorMap.getElevatorDoors(nearestButton);
    return doors;
  }

  /**
   * gets a list of the symbolic representations of the nearest elevator doors
   *
   * @return a list of Symbols representing the elevator doors
   */
  @Action
  @TRADEService
  public List<Symbol> getNearestElevatorDoors() {
    List<Symbol> out = new ArrayList<>();
    List<MapObject> ms = getNearestElevatorDoorObjects();
    for (MapObject m : ms) {
      out.add(m.getReferenceId());
    }
    return out;
  }

  /**
   * Get the lobby associated with the elevator door.
   *
   * @param door the MapObject of the elevator door
   * @return the MapObject of the lobby connected to provided door
   */
  private MapObject getLobbyFromDoor(MapObject door) {
    if (!door.isA("elevator-door")) {
      log.error("Can't get elevator lobby from non-elevator-door " + door);
      return null;
    }
    EndpointPair<MapObject> rooms = currFloorMap.getRoomsFromPortal(door);
    if (rooms.nodeU().isA("elevator")) {
      return rooms.nodeV();
    } else if (rooms.nodeV().isA("elevator")) {
      return rooms.nodeU();
    }
    log.error("Something went wrong with map parsing to get here");
    return null;
  }

  /**
   * gets the elevator room associated with the elevator-door
   *
   * @param door the MapObject of the elevator door
   * @return the MapObject of the elevator connected to provided door
   */
  private MapObject getElevatorFromDoor(MapObject door) {
    if (!door.isA("elevator-door")) {
      log.error("Can't get elevator room from non-elevator-door " + door);
      return null;
    }
    EndpointPair<MapObject> rooms = currFloorMap.getRoomsFromPortal(door);
    if (rooms.nodeU().isA("elevator")) {
      return rooms.nodeU();
    } else if (rooms.nodeV().isA("elevator")) {
      return rooms.nodeV();
    }
    log.error("Something went wrong with map parsing to get here");
    return null;
  }

  // TODO: EAK: it's not clear how learning/unlearning should be done in general. Commenting this
  //            out for now until a better solution is implemented.
  //TODO:brad: this should be worked into the record/save pose pipeline and Consultant
//  public Justification learn(Term learningTerm) {
//
//    // separate the name and target description
//    // should be of form "instanceOf(refId, descriptors)"
//    Symbol refId = learningTerm.get(0);
//    if (!Utilities.isReference(refId, consultant.getKBName())) {
//      log.error("[learn] first argument must be reference: " + learningTerm);
//      // TODO: create sensible failure predicate
//      return new ConditionJustification(false, Factory.createPredicate("propertyOf(semantics,knownForm)"));
//    }
//
//    edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose currPose = getPose();
//    Point3d posePosition = Convert.convertToPoint3d(currPose.getPosition());
//    Quat4d poseOrient = Convert.convertToQuat4d(currPose.getOrientation());
//
//    PoseReference poseReference = consultant.getReference(refId);
//    poseReference.setPosition(posePosition);
//    poseReference.setOrientation(poseOrient);
//
//    // add properties to reference
//    // replace variable in descriptors to match references's variable, e.g., knife(VAR5) --> knife(Y)
//    // location(a:LABEL)
//    List<Term> descriptors = PredicateHelper.convertToVisionForm(learningTerm.get(1));
//    List<Variable> descriptorVars = PredicateHelper.getUnboundVariables(descriptors);
//    if (descriptorVars.size() != 1) {
//      log.error("[learn] cannot currently handle descriptors with more than one Variable: " + learningTerm);
//      // TODO: create sensible failure predicate
//      return new ConditionJustification(false, Factory.createPredicate("propertyOf(semantics,knownForm)"));
//    }
//    List<Term> convertedDescriptors = new ArrayList<>();
//    descriptors.stream().forEach(d -> convertedDescriptors.add(PredicateHelper.replace(d, descriptorVars.get(0), poseReference.variable)));
//    consultant.assertProperties(refId, convertedDescriptors);
//
//    // ======================== START POWER HACK ================================
//    // remove "this(X)" descriptor -- this is to enable learning of multiple references so that in
//    // subsequent "this is a ..." utterances, "this" doesn't get resolved to the reference being
//    // learned here, and instead will start a new "this" search
//    List<Term> oldProperties = Arrays.asList(Factory.createPredicate("this", poseReference.variable));
//    consultant.retractProperties(refId, oldProperties);
//    // ======================== END POWER HACK ================================
//
//    return new ConditionJustification(true);
//  }
//
//  @Override
//  public Justification unlearn(Term learningTerm) {
//    throw new UnsupportedOperationException("[unlearn] is not yet supported.");
//  }

  @Override
  protected void shutdownComponent() {
    try {
      TRADE.deregister(consultant);
    } catch (Exception e) {
      log.error("[shutdown]", e);
    }
    floorMaps.clear();
  }
//
//  @GetMapping("/goodbye")
//  public String goodbye(@RequestParam(value = "name", defaultValue = "World") String name) {
//
//    return String.format("Goodbye %s!", name);
//  }
//
//  @RequestMapping(value = "/test_img", method = RequestMethod.GET,
//          produces = MediaType.IMAGE_JPEG_VALUE)
//  public void getImage(HttpServletResponse response) throws IOException {
//
//    var imgFile = new ClassPathResource("image/test.jpg");
//    response.setContentType(MediaType.IMAGE_JPEG_VALUE);
//    StreamUtils.copy(imgFile.getInputStream(), response.getOutputStream());
//  }

}


