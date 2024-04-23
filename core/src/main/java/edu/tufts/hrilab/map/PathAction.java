/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PathAction {

  public enum Type {
    ACTION_GOTO,
    ACTION_OPEN_DOOR,
    ACTION_CHECK_DOOR
  }

  private Type type;
  private Symbol reference;
  private Matrix4d pose;

  protected static Logger log = LoggerFactory.getLogger(PathAction.class);

  public PathAction(Type type, Symbol reference, Matrix4d pose) {
    this.type = type;
    this.reference = reference;
    this.pose = pose;
  }

  public PathAction(Type type, Symbol reference) {
    this.type = type;
    this.reference = reference;
    this.pose = null;
  }

  public Type getType() {
    return type;
  }

  public Symbol getReference() {
    return reference;
  }

  public Matrix4d getPose() {
    if (type != Type.ACTION_GOTO) {
      log.error("Cannot get pose from PathAction with type other than ACTION_GOTO");
      return null;
    }
    return pose;
  }

  public Point3d getPosition() {
    if (type != Type.ACTION_GOTO) {
      log.error("Cannot get position from PathAction with type other than ACTION_GOTO");
      return null;
    }

    Vector3d vec = new Vector3d();
    pose.get(vec);
    return new Point3d(vec);
  }

}
