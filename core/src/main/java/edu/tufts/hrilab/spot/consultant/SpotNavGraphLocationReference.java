/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.consultant;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import javax.vecmath.Point2d;
import java.util.List;

public class SpotNavGraphLocationReference extends Reference {

    protected String graphId;
    protected Point2d location;

    public SpotNavGraphLocationReference(Symbol ref, Variable variable) {
        super(ref, variable);
    }

    public SpotNavGraphLocationReference(Symbol ref, Variable variable, List<Term> properties, String graphId) {
        super(ref,variable,properties);
        this.graphId = graphId;
    }

    public void setLocation(Point2d location) {
        this.location = location;
    }

    public Point2d getLocation() {
        return location;
    }

    public String getGraphId() {
        return graphId;
    }

}
