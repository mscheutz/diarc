/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map.util;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class Utils {
  private static Logger log = LoggerFactory.getLogger(Utils.class);

  /**
   * gets distance between two points
   *
   * @param p1 source point
   * @param p2 target point
   * @return distance between the points
   */
  public static double dist(Point3d p1, Point3d p2) {
    return p1.distance(p2);
  }

  /**
   * gets distance between two points
   *
   * @param x1 source point x
   * @param y1 source point y
   * @param x2 target point x
   * @param y2 target point y
   * @return distance between the points
   */
  public static double dist(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
  }

  // TODO: move tests to a JUNIT Test
  /**
   * Displays output of various functions to assess functionality. Assumes use of cummings floor 4 map
   */
//  private void tests() {
//    //blank default orientation for making new poses
//    Quat4d o = new Quat4d(0, 0, 0, 1);
//
//    /* standardValue and MapObject equality test */
//    try {
//      MapObject room1 = getMapObjectFromPixelCoords(2537, 1496);
//      MapObject room2 = getMapObjectFromPixelCoords(2509, 1481);
//      MapObject alsoRoom1 = getMapObjectFromPixelCoords(2538, 1496);
//      if (!room1.equals(alsoRoom1)) {
//        throw new RuntimeException("equality check failed");
//      }
//      if (room2.equals(room1)) {
//        throw new RuntimeException("equality check failed");
//      }
//      alsoRoom1 = getStandardValue(alsoRoom1);
//      room1 = getStandardValue(room1);
//      if (alsoRoom1.getID() != room1.getID()) {
//        throw new RuntimeException("standardValue check failed");
//      }
//      log.info("standardValue passed");
//    } catch (Exception e) {
//      log.error("standardValue failed", e);
//    }
//
//    /* symbolToMapObject test */
//    try {
//      Symbol s = Factory.createSymbol("location_1", "location");
//      MapObject m = referenceToMapObject(s);
//      log.info(m);
//      log.info("symbolToMapObject test passed");
//    } catch (Exception e) {
//      log.error("symbolToMapObject test failed", e);
//    }
//
//    /* getMapObjectAt test */
//    try {
//      int px = 2537;
//      int py = 1496;
//      MapObject at = getMapObjectAt(toMeter(new Point2d(px, py)));
//      boolean b = false;
//      for (Point2d p : at.getPixels()) {
//        if ((int) p.x == px && (int) p.y == py) {
//          b = true;
//          break;
//        }
//      }
//      if (!b) {
//        throw new RuntimeException("map object didn't contain px, py!");
//      }
//      log.info("getMapObjectAt test passed");
//    } catch (Exception e) {
//      log.error("getMapObjectAt test failed", e);
//    }
//
//    /* graph connectivity test */
//    try {
//      MapObject room1 = getMapObjectAt(toMeter(new Point2d(2500, 1500)));
//      List<MapObject> neighbors = getConnectingRooms(room1);
//      if (neighbors.size() != 7) {
//        throw new RuntimeException("wrong number of connecting rooms!");
//      }
//      MapObject room2 = getMapObjectAt(toMeter(new Point2d(2537, 1496)));
//      MapObject door = getPortalBetween(room1, room2);
//      if (door == null) {
//        throw new RuntimeException("there should be a door there!");
//      }
//      log.info("graph connectivity test passed");
//    } catch (Exception e) {
//      log.error("graph connectivty failed", e);
//    }
//
//    /* elevator functions test */
//    try {
//      setRobotPose(convertToMatrix4d(new Pose(toMeter(new Point2d(2537, 1496)), o)));
//      MapObject mo = getNearestElevatorButton(currRobotPose.getPosition());
//      if (mo == null) {
//        throw new RuntimeException("nearest elevator is null!");
//      }
//      List<MapObject> doors = getNearestElevatorDoorObjects();
//      if (doors == null) {
//        throw new RuntimeException("elevator doors are null!");
//      }
//      if (doors.size() != 2) {
//        throw new RuntimeException("wrong number of doors: " + doors.size());
//      }
//      Iterator<MapObject> keys = elevatorRoomsToPanels.keySet().iterator();
//      while (keys.hasNext()) {
//        if (!elevatorRoomsToPanels.get(keys.next()).isA("elevator-panel")) {
//          throw new RuntimeException("non panel in elevatorRoomsToPanels");
//        }
//      }
////      Matrix4d pose = getFaceElevatorPanelPose(getElevatorFromDoor(getMapObjectAt(toMeter(new Point2d(1819, 1659)))), 0.5);
////      Pose p = convertToPose(pose);
//      log.info("elevator functions passed");
//    } catch (Exception e) {
//      log.error("elevator functions test failed", e);
//    }
//
//    /* getToLocation test */
////    try {
////      Pose room1 = new Pose(toMeter(new Point2d(2537, 1496)), o);
////      Pose room2 = new Pose(toMeter(new Point2d(2500, 1500)), o);
////      Pose room3 = new Pose(getNearestElevatorLobby().getCenter(), o);
////
////      Matrix4d m1 = convertToMatrix4d(room1);
////      Matrix4d m2 = convertToMatrix4d(room2);
////      Matrix4d m3 = convertToMatrix4d(room3);
////      List<Matrix4d> path = getToLocation(m1, m2, 0.5);
////      List<PathAction> patha = getPath(m1, m2, 0.5, false);
////      path = getToLocation(m1, m3, 0.5);
////      patha = getPath(m1, m3, 0.5, false);
////      log.info("getToLocation passed");
////    } catch (Exception e) {
////      log.error("getToLocation test failed", e);
////    }
//  }

/************************** Conversion Functions **************************/

  /**
   * converts a point and orientation to matrix format
   *
   * @param pose
   * @return a Matrix4d representing the point and orientation
   */
  public static Matrix4d convertToMatrix4d(Pose pose) {
    Point3d p = pose.getPosition();
    Quat4d q = pose.getOrientation();
    Matrix4d tmp = new Matrix4d();
    tmp.setIdentity();
    tmp.setRotation(q);
    tmp.setTranslation(new Vector3d(p.x, p.y, p.z));
    return tmp;
  }

  /**
   * gets pose from a Matrix4d
   *
   * @param m matrix to get pose from
   * @return pose represented by m
   */
  public static Pose convertToPose(Matrix4d m) {
    Vector3d v = new Vector3d();
    m.get(v);
    Quat4d q = new Quat4d();
    q.set(m);
    return new Pose(new Point3d(v.x, v.y, v.z), q);
  }
  /**
   * converts a quat4d into euler angles
   *
   * @param q quat4d to convert
   * @return double array with euler angles {roll, pitch, yaw}
   */
  public static double[] quat4dToEuler(Quat4d q) {
    double[] angles = {0, 0, 0};

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles[0] = Math.atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1) {
      angles[1] = Math.copySign(Math.PI / 2, sinp); // use 90 degrees if out of range
    } else {
      angles[1] = Math.asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles[2] = Math.atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  /**
   * Converts a euler angle to a quat4d
   *
   * @param yaw   yaw of euler angle
   * @param pitch pitch of euler angle
   * @param roll  roll of euler angle
   */
  public static Quat4d eulerToQuat4d(double yaw, double pitch, double roll) {
    double cy = Math.cos(yaw * 0.5);
    double sy = Math.sin(yaw * 0.5);
    double cp = Math.cos(pitch * 0.5);
    double sp = Math.sin(pitch * 0.5);
    double cr = Math.cos(roll * 0.5);
    double sr = Math.sin(roll * 0.5);

    Quat4d q = new Quat4d();
    q.setW(cr * cp * cy + sr * sp * sy);
    q.setX(sr * cp * cy - cr * sp * sy);
    q.setY(cr * sp * cy + sr * cp * sy);
    q.setZ(cr * cp * sy - sr * sp * cy);

    return q;
  }
}
