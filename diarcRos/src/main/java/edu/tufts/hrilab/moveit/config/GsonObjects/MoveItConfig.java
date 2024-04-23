/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.moveit.config.GsonObjects;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.HashMap;
import java.util.Map;

public class MoveItConfig {
  public String configName = "Unset";
  public String rosNamespace = "/";
  public String defaultGroupName;
  public float positionTolerance = 0.01f;
  public float angleTolerance = 0.01f;
  public int maxMoveToAttempts = 1;
  public double maxPlanningTime = 5.0f;
  public float maxGripperPosition = 1.0f;
  public float armReach = 0.4f;
  public float speedFactor = 1f;
  public float accelFactor = 1f;
  public String baseLinkString;
  public String pointingFrame;
  public String pressingFrame;
  public Vector3d graspAngle = new Vector3d(0, 0, -1); // Default to grasping from above
  public float graspApproachOffset;
  public float graspContactOffset;
  public LocalStaticTransform[] localStaticTransforms;
  public MoveItConfigPose[] poses;

  public MoveItConfigEEPose[] eeposes;

  public MoveItConfigCollisionObject[] collisions;
  public MoveItConfigPositionQuaternion eeTransform = new MoveItConfigPositionQuaternion();
  public boolean allowDisableCollisionAvoidance;
  private Logger log = LoggerFactory.getLogger(this.getClass());

  private MoveItConfigGroup[] groups;


  public MoveItConfig() {
  }

  public String[] getGroupNames() {
    String returnme[] = new String[groups.length];
    for (int n = 0; n < groups.length; ++n)
      returnme[n] = groups[n].groupName;

    return returnme;
  }

  public Map<String, String[]> getJointNamesMap() {
    Map<String, String[]> returnme = new HashMap<>();
    for (MoveItConfigGroup g : groups)
      returnme.put(g.groupName, g.jointNames);

    return returnme;
  }

  public Map<String, String> getEndEffectorNameMap() {
    Map<String, String> returnme = new HashMap<>();
    for (MoveItConfigGroup g : groups)
      returnme.put(g.groupName, g.effectorLinkName);

    return returnme;
  }

  public Map<String, GenericManipulator> getGripperClassMap() {
    Map<String, GenericManipulator> returnme = new HashMap<>();
    for (MoveItConfigGroup g : groups) {
      if (g.gripperClassName == null) {
        log.error("Gripper class name provided for " + g.groupName + " was null! Skipping.");
        continue;
      }
      GenericManipulator gm = GenericManipulator.instantiateGenericManipulator(g.gripperClassName);
      if (gm == null) {
        log.error("That manipulator spec was not available: " + g.gripperClassName + " on group " + g.groupName);
      } else {
        returnme.put(g.groupName, gm);
      }
    }

    return returnme;
  }

  public Matrix4d getEETransform() {
    Matrix4d transform = new Matrix4d();
    transform.setIdentity();
    transform.setRotation(new Quat4d(
            eeTransform.quaternion_x,
            eeTransform.quaternion_y,
            eeTransform.quaternion_z,
            eeTransform.quaternion_w));
    transform.setTranslation(new Vector3d(
            eeTransform.position_x,
            eeTransform.position_y,
            eeTransform.position_z));
    return transform;
  }
}
