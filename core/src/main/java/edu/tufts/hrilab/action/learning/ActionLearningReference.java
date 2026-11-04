/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

public class ActionLearningReference extends Reference {

    String localVarName;

    public ActionLearningReference(Symbol ref, Variable variable) {
        super(ref, variable);
        this.localVarName= "";
    }
    public ActionLearningReference(Symbol ref, Variable variable, String localVarName) {
        super(ref, variable);
        this.localVarName= localVarName;
    }

    public String getLocalVar(){
        return localVarName;
    }

    //TODO:brad:I don't think we should be able to do this
    public void addProperty(String propName, String propType){
        properties.add(new Term(propName,Factory.createVariable("X",propType)));
    }
}
