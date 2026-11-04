/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.generic;

import edu.tufts.hrilab.vision.stm.Grasp;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Vector3d;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;

import static java.util.Objects.requireNonNull;

public abstract class GenericManipulator {
  public float maxGraspWidthMeters;
  protected Logger log;

  /* * * * * *
   * To add a new manipulator:
   * 1) extend this class
   * 2) implement the three below methods
   * 3) update src-build-targets.xml (see manipulator-kortex as an example of how)
   * It should now be possible to set the manipulator type in the JSON config.
   * The rest of this file beyond those three functions is helpful utility stuff that can be pretty standard anyway.
   * * * * * */

  /**
   * Move the gripper. `position` represents a position in meters. 0.0 is closed, and 0.05 is open 5 cm.
   */
  public abstract boolean moveGripper(float position);

  /**
   * Return the current gripper position, using the same concept of `position` from moveGripper().
   */
  public abstract float getCurrentGripperPosition();

  /**
   * Return the goal gripper position, if known, in meters. 0.0 is closed, and 0.05 is open 5 cm.
   */
  public abstract float getGoalGripperPosition();

  /**
   * Return the current effort, if known
   */
  public abstract float getCurrentGripperEffort();

  /**
   * Shut down any threads or nodes to make everything nice and clean.
   */
  public abstract void shutdown();

  public GenericManipulator() {
    log = LoggerFactory.getLogger(this.getClass());
  }

  /**
   * Instantiate the target GenericManipulator class. Will return null if the class could not be instantiated.
   * @param classPath fully qualified class path
   * @return
   */
  public static GenericManipulator instantiateGenericManipulator(String classPath) {
    try {
      Class<?> clazz = Class.forName(classPath);
      Class<? extends GenericManipulator> newClass = clazz.asSubclass(GenericManipulator.class);
      Constructor<? extends GenericManipulator> constructor = newClass.getConstructor();

      // If things haven't exploded yet, they worked! Return a new instance of the gripper off that constructor.
      return constructor.newInstance();
    } catch (ClassNotFoundException | NoSuchMethodException | IllegalAccessException | InstantiationException | InvocationTargetException e) {
      LoggerFactory.getLogger(GenericManipulator.class).error("Invocation exception thrown while attempting to create" +
              "a new instance of your gripper type. This can occur when the gripper's constructor has failed.", e);
    }

    return null;
  }

  //FIXME: This function assumes that we're not dealing with a two-handed grasp (that there's only two points for one gripper to handle)

  /**
   * From a Grasp object, get the distance between the two grasp points in millimeters.
   */
  public int graspToDistance(Grasp grasp) {
    // Just a simple pythagorean calculation
    Vector3d p0 = grasp.getPoint(0);
    Vector3d p1 = grasp.getPoint(1);
    double x = (p0.x - p1.x) * 1000; // While we're here-- this is in meters, we want mm
    double y = (p0.y - p1.y) * 1000;
    double z = (p0.z - p1.z) * 1000;

    double dist_xy = Math.sqrt(x * x + y * y);
    double dist = Math.sqrt(dist_xy * dist_xy + z * z);

    return (int) Math.round(dist); // We're in mm, so losing some data is OK. Casting b/c round() is long
  }

}
