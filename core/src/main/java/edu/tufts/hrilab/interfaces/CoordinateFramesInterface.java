/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import java.util.List;
import javax.vecmath.Matrix4d;

import ai.thinkingrobots.trade.TRADEService;

/**
 * Core interface for robot coordinate frames and their transformations.
 */
public interface CoordinateFramesInterface {

  /**
   * Get the current transform from the base of the robot to whichever frame you
   * specify. This is just a convenience method for the more general
   * getTransform(src,dest).
   *
   * @param dstFrame
   * @return
   *
   */
  @TRADEService
  Matrix4d getTransform(String dstFrame);

  /**
   * Get the current transform to convert from one local coordinate frame to a
   * different one.
   *
   * @param srcFrame source frame
   * @param dstFrame destination frame
   * @return
   *
   */
  @TRADEService
  Matrix4d getTransform(String srcFrame, String dstFrame);

  /**
   * Get a list of all coordinate frames currently in the system.
   *
   * @return
   *
   */
  @TRADEService
  List<String> getCoordinateFrames();

  /**
   * Add a static transform to the TF tree. This only adds the transform
   * to the local TF instance, and does not add it to the ROS TF tree.
   *
   * @param parent
   * @param child
   * @param transform
   */
  @TRADEService
  void addLocalStaticTransform(String parent, String child, Matrix4d transform);
}
