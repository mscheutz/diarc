/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao.movement;

/**
 * Enumeration for specifying Nao joints. For safety (historically naoqi has not
 * done safety checking on passed strings, possibly causing damage to robot).
 */
public enum NaoJoint {
  /* body */ Body,
  /* chains */ Head, LArm, RArm, LLeg, RLeg, Torso,
  /* joints/actuators */ JointActuators, Joints, Actuators,
  /* indiv joints */ HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll,
  LElbowYaw, LElbowRoll, LWristYaw, LHipYawPitch, LHipRoll, LHipPitch,
  LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll, RHipPitch,
  RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll,
  RElbowYaw, RElbowRoll, RWristYaw, RHand, LHand
}
