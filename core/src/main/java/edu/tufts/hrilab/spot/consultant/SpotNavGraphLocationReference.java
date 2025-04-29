/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.consultant;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import javax.vecmath.Point2d;
import javax.vecmath.Quat4d;
import java.util.List;

public class SpotNavGraphLocationReference extends Reference {

    protected String graphId;
    protected Point2d position;
    protected Quat4d orientation;

    public SpotNavGraphLocationReference(Symbol ref, Variable variable) {
        super(ref, variable);
    }

    public SpotNavGraphLocationReference(Symbol ref, Variable variable, List<Term> properties, String graphId) {
        super(ref,variable,properties);
        this.graphId = graphId;
    }

    public SpotNavGraphLocationReference(Symbol ref, Variable variable, List<Term> properties, String graphId, Point2d position, Quat4d orientation) {
        super(ref,variable,properties);
        this.graphId = graphId;
        this.position = position;
        this.orientation = orientation;
    }

    public void setPose(Point2d position, Quat4d orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Point2d getPosition() {
        return position;
    }

    public Quat4d getOrientation() {
        return orientation;
    }

    public String getGraphId() {
        return graphId;
    }

}
