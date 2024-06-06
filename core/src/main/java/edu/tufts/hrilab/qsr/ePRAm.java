package edu.tufts.hrilab.qsr;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;

/**
 * A rudimentary implementation of ePRAm spatial calculus.
 * Used for generating qualitative symbolic relations between points, e.g. left(a,b) or far(a,b)
 */
public class ePRAm {

  /**
   * Calculates the ePRAm distance of the target from the source.
   * We consider 5 distances
   * min distance currently hard coded as 1.0
   *
   * @param a The source for the relation
   * @param b The target for the relation
   * @return The distance of the target from the source
   */
  public static Distance getDist(Point3d a, Point3d b) {
    double min = 1.0;
    double dist = a.distance(b);

    if (dist > min * 8) {
      return Distance.FARTHEST;
    } else if (dist > min * 4) {
      return Distance.FAR;
    } else if (dist > min * 2) {
      return Distance.NEAR;
    } else if (dist > min) {
      return Distance.CLOSE;
    } else {
      return Distance.AT;
    }
  }

  /**
   * Calculates the ePRAm angle of the target from the source.
   * m is currently hard coded to 2
   *
   * @param a The source for the relation
   * @param b The target for the relation
   * @return The direction of the target from the source
   */
  public static Direction getAngle(Point3d a, Point3d b) {
    double angle = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    // Normalize angle to range [-180, 180]
    angle = (angle + 360) % 360;
    if (angle < 0)
      angle += 360;

    // Define directional ranges
    if ((angle >= 337.5 && angle <= 360) || (angle >= 0 && angle < 22.5))
      return Direction.N;
    else if (angle >= 22.5 && angle < 67.5)
      return Direction.NE;
    else if (angle >= 67.5 && angle < 112.5)
      return Direction.E;
    else if (angle >= 112.5 && angle < 157.5)
      return Direction.SE;
    else if (angle >= 157.5 && angle < 202.5)
      return Direction.S;
    else if (angle >= 202.5 && angle < 247.5)
      return Direction.SW;
    else if (angle >= 247.5 && angle < 292.5)
      return Direction.W;
    else if (angle >= 292.5 && angle < 337.5)
      return Direction.NW;
    else
      System.err.println("Malformed angle.");
    return null;
  }

  /**
   * Calculates ePRAm relations of the target from the source
   *
   * @param a The source for the relation
   * @param b The target for the relation
   * @return A list of relations from the target to the source
   */
  public static List<String> calculateRelations(Point3d a, Point3d b) {
    List<String> relations = new ArrayList<>();
    relations.add(getAngle(a, b).name());
    relations.add(getDist(a, b).name());
    return relations;
  }

  public enum Direction {
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW
  }

  public enum Distance {
    AT,
    CLOSE,
    NEAR,
    FAR,
    FARTHEST
  }
}
