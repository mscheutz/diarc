/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.temiv3;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayList;
import java.util.List;

public class TemiV3PoseReference extends Reference {
    protected List<Float> rawPose;
    protected List<Float> mapPose;
    protected Symbol poseName;

    public TemiV3PoseReference(Symbol refID, Variable variable) {
        super(refID, variable);
    }

    public TemiV3PoseReference(Symbol ref, Variable variable, ArrayList<Float> rawPose) {
        super(ref, variable);
        this.rawPose = rawPose;
    }

    public TemiV3PoseReference(Symbol ref, Variable variable, List<Float> rawPose, List<Float> mapPose) {
        super(ref, variable);
        this.rawPose = rawPose;
        this.mapPose = mapPose;
    }

    public Symbol getName(){
        return poseName;
    }

    public List<Float> getPose() {
        return rawPose;
    }

    public List<Float> getMapPose() {
        return mapPose;
    }

    public void setName(Symbol name){
        this.poseName=name;
    }

    public void setPose(List<Float> pose) {
        this.rawPose = pose;
    }

    public void setMapPose(List<Float> pose) {
        this.mapPose = pose;
    }

    public boolean hasPose() {
        return rawPose != null;
    }

    @Override
    public String toString() {
        return super.toString() + " pose = " + ((rawPose == null) ? " (is null)" : rawPose);
    }
}
