/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.consultant.pose;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.mtracs.util.MPose;

import java.util.List;

public class MPoseReference extends Reference {
    MPose pose;

    public MPoseReference(Symbol ref, Variable variable) {
        super(ref, variable);
    }

    public MPoseReference(Symbol ref, Variable variable, MPose pose) {
        super(ref, variable);
        this.pose = pose;
    }

    public MPoseReference(Symbol ref, Variable variable, MPose pose, List<Term> properties) {
        super(ref, variable, properties);
        this.pose = pose;
    }

    public MPose getPose() {
        return pose;
    }

    public void setPose(MPose pose) {
        this.pose = pose;
    }

    public boolean hasPose() {
        return pose != null;
    }

    @Override
    public String toString() {
        return super.toString() + " pose = " + ((pose == null) ? " (is null)" : pose);
    }
}
