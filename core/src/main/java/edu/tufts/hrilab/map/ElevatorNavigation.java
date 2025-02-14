/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.google.common.graph.EndpointPair;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.Convert;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/****************************************************************************
 *         ELEVATOR SPECIFIC METHODS
 *         TODO: can these be generalized?
 ***************************************************************************/

public class ElevatorNavigation {
  private Logger log = LoggerFactory.getLogger(ElevatorNavigation.class);
  private FloorMap currFloorMap;

  public void registerWithTrade(Collection<String> groups) {
    try {
      TRADE.registerAllServices(this, groups);
    } catch (TRADEException e) {
      log.error("Error registering ElevatorNavigation services with TRADE.", e);
    }
  }

  public void setCurrentFloorMap(FloorMap map) {
    this.currFloorMap = map;
  }

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
    Pair<Point3d, Quat4d> pose = Convert.convertToPair(transform);
    try {
      //double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("goToLocation").argTypes(double.class, double.class, double.class, double.class, double.class, double.class, Boolean.class))
              .call(Justification.class, pose.getLeft().x, pose.getLeft().y,
                      pose.getRight().x, pose.getRight().y, pose.getRight().z, pose.getRight().w,
                      true);
    } catch (TRADEException e) {
      log.error("Error calling goToLocation.", e);
      return new ConditionJustification(false);
    }
  }

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
    Pair<Point3d, Quat4d> currRobotPose = getRobotPose();
    if (currRobotPose == null) {
      log.error("Could not get robot pose. Returning empty door list.");
      return new ArrayList<>();
    }

    MapObject nearestButton = currFloorMap.getNearestElevatorButton(currRobotPose.getLeft());
    log.debug("nearest button: ({},{})", nearestButton.getPosition().x, nearestButton.getPosition().y);
    List<MapObject> doors = currFloorMap.getElevatorDoors(nearestButton);
    return doors;
  }

  /**
   * gets a list of the symbolic representations of the nearest elevator doors
   *
   * @return a list of Symbols representing the elevator doors
   */
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

  /**
   * Helper method to get the current pose of the robot.
   */
  private Pair<Point3d, Quat4d> getRobotPose() {
    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("getPose").argTypes()).call(Pair.class);
    } catch (TRADEException e) {
      log.error("Unable to set robot pose", e);
    }

    return null;
  }
}
