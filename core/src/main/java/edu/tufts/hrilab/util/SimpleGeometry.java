/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.util;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import javax.vecmath.Vector3d;

/**
Provides static methods for geometry calculations.

@author Jim Kramer
*/
public final class SimpleGeometry {
	
	/** Return a double indicating the <i>direction</i> of a point relative
	 * to a line (as defined by two other points). This is done by translating
	 * the points such that one end of the line is treated as the origin, then
	 * finding the cross-product of the other two (modified from Cormen, et
	 * al., Intro to Alg's, 2001, p936).
	 * @param x1 The x coordinate of point 1 (the line's origin)
	 * @param y1 The y coordinate of point 1 (the line's origin)
	 * @param x2 The x coordinate of point 2 (the line's endpoint)
	 * @param y2 The y coordinate of point 2 (the line's endpoint)
	 * @param x3 The x coordinate of point 3 (the relative point)
	 * @param y3 The y coordinate of point 3 (the relative point)
	 * @return A value that is:
	 *   <ol>
	 *   <li><b>negative</b> if the point is counter-clockwise to the line</li>
	 *   <li><b>zero</b> if the point is on the line</li>
	 *   <li><b>positive</b> if the point is clockwise to the line</li>
	 *   </ol> */
	public static final double direction(
			double x1, double y1, double x2, double y2, double x3, double y3) {
		return(((x2 - x1) * (y3 - y1)) - ((x3 - x1) * (y2 - y1)));
	}

	/** Return a double indicating the <i>direction</i> of a point relative
	 * to a line (as defined by two other points).
	 * @param p1 Point 1 (the line's origin)
	 * @param p2 Point 2 (the line's endpoint)
	 * @param p3 Point 3 (the relative point)
	 * @return <b>negative</b> for counter-clockwise, <b>zero</b> for
	 *   coincident, <b>positive</b> for clockwise */
	public static final double direction(Point2D p1, Point2D p2, Point2D p3) {
		return direction(p1.getX(), p1.getY(), p2.getX(), p2.getY(), p3.getX(), p3.getY());
	}
	
	/** Return a double indicating the <i>direction</i> of a point relative
	 * to a line.
	 * @param ln The line
	 * @param pt The relative point
	 * @return <b>negative</b> for counter-clockwise, <b>zero</b> for
	 *   coincident, <b>positive</b> for clockwise */
	public static final double direction(Line2D ln, Point2D pt) {
		return direction(ln.getX1(), ln.getY1(), ln.getX2(), ln.getY2(), pt.getX(), pt.getY());
	}
	
	/** Return a double indicating the <i>direction</i> of one angle relative
	 * to another (angles measured in radians and assumed to share origins).
	 * Note that the value returned is the negation of the difference between
	 * the angles (in radians).
	 * @param a1 The source angle
	 * @param a2 The target angle
	 * @return <b>negative</b> for counter-clockwise, <b>zero</b> for
	 *   coincident, <b>positive</b> for clockwise */
	public static final double direction(double a1, double a2) {
		return -(diffAngle(a1, a2));
	}
	
	/** Return the (smallest) difference between two angles (in radians),
	 * assumed to have the same origin.
	 * @param a1 An angle
	 * @param a2 Another angle
	 * @return An angle ranging from {@code -PI &lt angle &lt= PI} */
	public static final double diffAngle(double a1, double a2) {
		double a1cos = Math.cos(a1), a1sin = Math.sin(a1);
		double a2cos = Math.cos(a2), a2sin = Math.sin(a2);
		
		double retcos = a1cos * a2cos + a1sin * a2sin;
		double retsin = a1sin * a2cos - a1cos * a2sin;
		return -Math.atan2(retsin, retcos);
	}
	
	/** Normalize an angle.
	 * @param ang The angle
	 * @return An angle ranging from {@code -PI &lt angle &lt= PI} */
	public static final double normAngle(double ang) {
		double a = ang % (2*Math.PI);
		if (a > Math.PI) a -= (2*Math.PI);
		else if (a < -Math.PI) a+= (2*Math.PI);
		return a;
	}
	
	/** Given the length of two legs of a triangle and the angle
	 * between them, calculates the length of the third leg.
	 * @param a The length of one triangle's leg
	 * @param b The length of another triangle's leg
	 * @param ca The angle between {@code a} and {@code b}
	 * @return The length of the triangle's third leg */
	public static final double lawOfCosDist(double a, double b, double ca) {
		return Math.sqrt((a*a) + (b*b) - 2*a*b*Math.cos(ca));
	}
	
	/** Given a global location {@code g} and a relative polar point
	 * {@code P}, return the global location of {@code P} as an (x,y) array.
	 * {@code P} is specified in polar representation, (i.e., distance and
	 * angle) relative to {@code g}, which is taken as the origin of the
	 * line having length {@code d} at angle {@code a}.
	 * @param g The global coordinates (x, y, theta), theta in radians
	 * @param d {@code P}'s distance
	 * @param a {@code P}'s angle (in radians)
	 * @return An (x,y) array representing {@code P}'s global coordinates */
	public static final double[] polarToGlobal(double[] g, double d, double a) {
		if (g.length != 3)
			throw new IllegalArgumentException("Wrong coordinate specification");
		double[] retArr = new double[2];
		double newT;
		
		newT = normAngle(g[2] + a);
		retArr[0] = g[0] + (d * Math.cos(newT));
		retArr[1] = g[1] + (d * Math.sin(newT));
		return retArr;
	}

	/**
	 * Calculates the vector projection onto a plane defined by its normal vector
	 *
	 * @param residual_vector The vector that lies outside the projection/plane
	 * @param plane_normal    The vector that is normal to the plane being projected onto
	 * @return The projection vector from the residual_vector onto the plane
	 */
	public static Vector3d projectVectorOntoPlane(Vector3d residual_vector, Vector3d plane_normal) {
		double dot_prod = residual_vector.dot(plane_normal);

		Vector3d projection = new Vector3d(plane_normal);
		projection.scale(dot_prod);

		Vector3d projected_vector = new Vector3d(residual_vector);
		projected_vector.sub(projection);

		return projected_vector;
	}
}
