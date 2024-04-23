/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import java.io.Serializable;
import javax.vecmath.Matrix4d;

public class MemoryPrimitive implements Serializable {
  
  public enum Shape { BOX, SPHERE, CYLINDER, CONE };
  
  public Shape shape;
  public double[] dims;
  /**
   * Pose relative to coordinate frame of containing MemoryObject
   */
  public Matrix4d pose;
  
  public MemoryPrimitive() {
  }
  
  public MemoryPrimitive(Shape shape, double[] dims, Matrix4d pose) {
    this.shape = shape;
    this.dims = dims;
    this.pose = pose;
  }
  
}