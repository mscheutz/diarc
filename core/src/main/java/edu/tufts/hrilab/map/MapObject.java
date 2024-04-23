/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import edu.tufts.hrilab.util.Convert;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.fol.Symbol;

import java.lang.*;
import java.util.*;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class MapObject {

  /**
   * Unique ID for only for MapObjects on the same floor map. IDs are not guaranteed to be unique across maps.
   */
  private int id;
  /**
   * Semantic property based on color coded map.
   */
  private String property;
  /**
   * The building floor number.
   */
  private int floor;
  /**
   * All the pixels on the map that belong to this MapObject.
   */
  private List<Point2d> pixels;
  /**
   * The "center" pixel on the map.
   */
  private Point2d centerPixel;
  /**
   * The metric position (x,y,z) where z==0
   */
  private Point3d position;
  /**
   * The Orientation on the map. This can be null.
   */
  private Quat4d orientation = null;
  /**
   * The reference ID attaching a MapObject to the ReferenceResolution system.
   */
  private Symbol referenceId = null;

  protected static Logger log = LoggerFactory.getLogger(MapObject.class);

  public MapObject(int id, String property, int floor, List<Point2d> pixels) {
    this.id = id;
    this.property = property;
    this.floor = floor;
    this.pixels = pixels;
    Collections.sort(this.pixels, new SortPoint());
    this.centerPixel = pixels.get(pixels.size() / 2); // TODO: this isn't a good way to set the center
  }

  public int getID() {
    return this.id;
  }

  public String getProperty() {
    return this.property;
  }

  public int getFloor() {
    return this.floor;
  }

  public Point3d getPosition() {
    return this.position;
  }

  public void setPosition(Point3d newCenter) {
    this.position = newCenter;
  }

  public Matrix4d getPose() {
    if (orientation == null) {
      return Convert.convertToMatrix4d(position, new Quat4d(0, 0, 0, 1));
    } else {
      return Convert.convertToMatrix4d(position, orientation);
    }
  }

  public Quat4d getOrientation() {
    return this.orientation;
  }

  public void setOrientation(Quat4d orientation) {
    this.orientation = orientation;
  }

  public Point2d getCenterPixel() {
    return this.centerPixel;
  }

  public List<Point2d> getPixels() {
    return this.pixels;
  }

  public void setReferenceId(Symbol refId) {
    this.referenceId = refId;
  }

  public Symbol getReferenceId() {
    return referenceId;
  }

  public String toString() {
    return "Floor " + floor + " - " + property + id + " - (" + this.pixels.get(0).x + ", " + this.pixels.get(0).y + ")";
  }

  /**
   * asks if this MapObject is of a certain type
   *
   * @param property objecty type to check for
   * @return true if MapObject is of type property, false otherwise
   */
  public boolean isA(String property) {
    return this.property.equals(property);
  }

  @Override
  public boolean equals(Object obj) {
    MapObject other = (MapObject) obj;
    return this.id == other.id;
  }

  @Override
  public int hashCode() {
    return this.id;
  }
}

class SortPoint implements Comparator<Point2d> {

  public int compare(Point2d a, Point2d b) {
    double diff = a.x - b.x;
    int out;
    if (diff == 0) {
      diff = a.y - b.y;
    }
    if (Math.abs(diff) < 1) {
      out = (int) Math.round(Math.abs(diff) / diff);
    } else {
      out = (int) Math.round(diff);
    }

    return out;
  }
}
