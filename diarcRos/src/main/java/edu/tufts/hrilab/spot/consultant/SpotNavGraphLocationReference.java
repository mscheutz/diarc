/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.consultant;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;

public class SpotNavGraphLocationReference extends Reference {

    protected String nodeName;
    protected String graphId;

    public SpotNavGraphLocationReference(Symbol ref, Variable variable) {
        super(ref, variable);
    }

    public SpotNavGraphLocationReference(Symbol ref, Variable variable, List<Term> properties, String nodeName, String graphId) {
        super(ref,variable,properties);
        this.nodeName = nodeName;
        this.graphId = graphId;
    }


    //todo: need a distinction between the node name needed for the API on the spot and the name given
    //  to and used for that node by humans, etc.
    public String getNodeName() {
        return nodeName;
    }


    public String getGraphId() {
        return graphId;
    }

}
