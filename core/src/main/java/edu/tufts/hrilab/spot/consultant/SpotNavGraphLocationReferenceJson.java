/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.consultant;

import edu.tufts.hrilab.fol.Term;

import javax.vecmath.Point2d;
import javax.vecmath.Quat4d;
import java.util.List;
import java.util.stream.Collectors;

public class SpotNavGraphLocationReferenceJson {
  protected List<String> properties;
  protected String variable;
  protected String graphId;
  protected double x;
  protected double y;
  protected double orientX;
  protected double orientY;
  protected double orientZ;
  protected double orientW;

  public SpotNavGraphLocationReferenceJson(SpotNavGraphLocationReference reference) {
    this.variable = reference.variable.toString();
    this.properties = reference.properties.stream().map(Term::toString).collect(Collectors.toList());
    this.graphId = reference.getGraphId();
    this.x = reference.getPosition().x;
    this.y = reference.getPosition().y;
    this.orientX = reference.getOrientation().x;
    this.orientY = reference.getOrientation().y;
    this.orientZ = reference.getOrientation().z;
    this.orientW = reference.getOrientation().w;
  }

  public String getVariable() {
    return variable;
  }
  public String getGraphId() {
    return graphId;
  }

  public List<String> getProperties() {
    return properties;
  }

  public Point2d getPosition() {
    return new Point2d(x, y);
  }

  public Quat4d getOrientation() {
    return new Quat4d(orientX, orientY, orientZ, orientW);
  }

}
