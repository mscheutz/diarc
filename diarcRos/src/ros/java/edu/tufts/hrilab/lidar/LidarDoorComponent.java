/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.lidar;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarcros.laserscan.LaserScanSubscriber;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;

import java.util.*;
import javax.vecmath.Point2d;
import javax.vecmath.Quat4d;

import edu.tufts.hrilab.fol.Symbol;

public class LidarDoorComponent extends DiarcComponent {
  /**
   * ROS laser subscriber.
   */
  private LaserScanSubscriber laserScanSubscriber;


  @Override
  protected void init() {
    laserScanSubscriber = new LaserScanSubscriber("/base_scan");
    log.debug("LidarDoorComponent waiting on laser scan data...");
    laserScanSubscriber.waitForNode();
    log.debug("LidarDoorComponent ready to go!");
  }

  /**
   * Action to wait for a door to open and returns a reference to the open door.
   * Waits up to a minute.
   *
   * @return reference to open door
   */
  @Action
  @TRADEService
  public Symbol waitForOpenDoor() {
    List<Symbol> doors = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getNearestElevatorDoors"));
      doors = tsi.call(List.class);
    } catch (TRADEException e) {
      log.error("Couldn't get nearest elevator doors.", e);
    }

    Symbol openDoor = null;
    long oneMinute = 60000;
    long startTime = System.currentTimeMillis();
    while ((openDoor == null) && (System.currentTimeMillis() < startTime + oneMinute)) {
      for (Symbol door : doors) {
        if (isDoorOpen(door).getValue()) {
          openDoor = door;
        }
      }
    }

    if (openDoor == null) {
      log.warn("[waitForOpenDoor] no open elevator door found in allotted time");
    }
    return openDoor;
  }

  /**
   * Action to query if a door is open
   *
   * @param door a symbolic representation of the door being queried
   * @return if door is open or not
   */
  @Action
  @TRADEService
  public Justification isDoorOpen(Symbol door) {
    log.debug("=============== IsDoorOpen ===============");

    List<Point2d> doorEdges = new ArrayList<>();
      try {
        TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getDoorEdges"));
        doorEdges = tsi.call(List.class, door);
      } catch (TRADEException e) {
        log.error("Trade Call failed: ", e);
        return new ConditionJustification(false);
      }


    double[] robotpose;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPoseGlobalQuat"));
      robotpose = tsi.call(double[].class);
    } catch (TRADEException e) {
      log.error("Trade Call failed: ", e);
      return new ConditionJustification(false);
    }

    log.debug("got door edges " + doorEdges.get(0).toString() + " and " + doorEdges.get(1).toString() + " in isDoorOpen");
    log.debug("got robot pose: (" + String.valueOf(robotpose[0]) + ", " + String.valueOf(robotpose[1]) + ")");

    // get 5 points along door, and their distances
    List<Point2d> doorPoints = getPointsAlongDoor(doorEdges.get(0), doorEdges.get(1));
    Point2d robopoint = new Point2d(robotpose[0], robotpose[1]);
    List<Double> doorPointDistances = getDoorPointDistances(doorPoints, robopoint);
    doorPointDistances.sort(Comparator.naturalOrder());

    // log.debug("checking door points: " + doorPoints.toString());

    List<Double> angles = getDoorAngles(doorEdges.get(0), doorEdges.get(1), robotpose);
    double minAng = Math.min(angles.get(0), angles.get(1));
    double maxAng = Math.max(angles.get(0), angles.get(1));
    double fifth_of_range = (maxAng - minAng) / 5;

    log.debug("got angles " + String.valueOf(minAng * 180 / Math.PI) + " and " + String.valueOf(maxAng * 180 / Math.PI));

    double theta = minAng;
    boolean doorOpen = false;
    boolean isBlocked = false;
    for (double distance : doorPointDistances) {
      double scandist = getAvgLaserScanInRange(theta, theta + fifth_of_range);

      log.debug("distance lidar reports: " + String.valueOf(scandist));
      log.debug("expected distance of this part of the door: " + String.valueOf(distance));

      if (Double.isNaN(scandist)) {
        log.warn("not enough scan data, assuming door is closed");
        doorOpen = false;
      }
      // if the scandist is within .3 meters of the expected location of the door, its probably closed
      // TODO: calibrate offset (I just guessed .3 meters)
      else if (scandist > distance + .3) {
        doorOpen = true;
        break;
      } else if (scandist < distance - .3) {
        isBlocked = true;
      }

      theta += fifth_of_range;
    }

    if (isBlocked) {
      log.warn("part of the door appears to be blocked");
    }
    log.debug("=============== ==== ===============");

    return new ConditionJustification(doorOpen);
  }

  /**
   * gets the average value of a scan bounded by a min and max angle, relative to the center
   *
   * @param min minimum angle of range in radians
   * @param max maximum angle of range in radians
   * @return average laser scan between two angles
   */
  private double getAvgLaserScanInRange(double min, double max) {
    while (laserScanSubscriber.getScan() == null) {
    }
    ;
    float[] ls = laserScanSubscriber.getScan().getRanges();
    // log.debug("giant array of scans: " + Arrays.toString(ls));
    int minIdx = (int) Math.round((ls.length / 2) + (min / laserScanSubscriber.getScan().getAngleIncrement()));
    int maxIdx = (int) Math.round((ls.length / 2) + (max / laserScanSubscriber.getScan().getAngleIncrement()));
    float avgScan = avgScanCalculator(ls, minIdx, maxIdx);

    // log.debug("door was calculated to be between indices " + minIdx + " and " + maxIdx + " , relative to the robot");

    return avgScan;
  }

  /**
   * gets the average value of a laser scan between two indicies
   *
   * @param minIdx minimum index
   * @param maxIdx maximum index
   * @return average distance determined by scan (if 40% or more is NaN, will return NaN)
   */
  private float avgScanCalculator(float[] ls, int minIdx, int maxIdx) {
    if (maxIdx >= ls.length) {
      log.warn("Warning: provided max angle was out of range");
      maxIdx = ls.length - 1;
    }
    if (minIdx < 0) {
      log.warn("Warning: provided min angle was out of range");
      minIdx = 0;
    }

    int nans = 0;
    int count = 0;
    float sum = 0;
    for (int i = minIdx; i <= maxIdx; i++) {
      float curr = ls[i];
      // log.debug("index " + String.valueOf(i) + " has distance value " + String.valueOf(curr));
      if (Float.isNaN(curr) || Float.isInfinite(curr)) {
        nans += 1;
      } else {
        sum += curr;
        count++;
      }
    }
    // if (nans >= (0.4) * (maxIdx - minIdx)) {
    //   return Float.NaN;
    // }
    sum /= (count);
    return sum;
  }


  /**
   * Get the scannable angles from a door given the robots relative position
   *
   * @param p1        one end of the door
   * @param p2        the other end of the door
   * @param robotpose pose of robot
   * @return A list with index 0 being the minimum angle to scan, and index 1 the maximum
   */
  @TRADEService
  private List<Double> getDoorAngles(Point2d p1, Point2d p2, double[] robotpose) {

    Quat4d roboQuat = new Quat4d(robotpose[2], robotpose[3], robotpose[4], robotpose[5]);
    double robAng = edu.tufts.hrilab.util.Convert.convertToEuler(roboQuat).getY();

    Point2d robPoint = new Point2d(robotpose[0], robotpose[1]);
    double ang1 = Math.atan2(p1.y - robPoint.y, p1.x - robPoint.x);
    ang1 -= robAng;

    double ang2 = Math.atan2(p2.y - robPoint.y, p2.x - robPoint.x);
    ang2 -= robAng;

    // in lidar data, clockwise of center is positive and ccw is negative, unlike most conventions
    ang1 *= -1;
    ang2 *= -1;
    List<Double> out = new ArrayList<>();
    out.add(ang1);
    out.add(ang2);
    return out;
  }

  /**
   * Get several points along a door
   *
   * @param p1 one end of the door
   * @param p2 the other end of the door
   * @return A list of 5 points on the door
   */
  private List<Point2d> getPointsAlongDoor(Point2d p1, Point2d p2) {
    // TODO: not working
    double dx = (double) (p2.x - p1.x);
    double dy = (double) (p2.y - p1.y);

    Point2d fifthVector = new Point2d(dx / 5, dy / 5);
    p1.x -= dx / 10;
    p1.y -= dy / 10;

    List<Point2d> doorPoints = new ArrayList<>();
    for (int i = 0; i < 5; i++) {
      Point2d toAdd = new Point2d(p1.x + fifthVector.x, p1.y + fifthVector.y);
      doorPoints.add(toAdd);
      p1.x += fifthVector.x; // wrong!
      p1.y += fifthVector.y;
    }

    return doorPoints;
  }


  /**
   * Get several points along a door
   *
   * @param doorPoints List of points on the door
   * @param robotLoc   x, y of the robot
   * @return array of corresponding distances to each point
   */
  private List<Double> getDoorPointDistances(List<Point2d> points, Point2d robotLoc) {
    List<Double> distances = new ArrayList<>();
    for (Point2d pt : points) {
      double distance = pt.distance(robotLoc);
      distances.add(distance);
    }
    return distances;
  }

}