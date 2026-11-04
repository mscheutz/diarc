/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

//TODO:brad: should this actually extend reference? We don't care about the Variable and properties. Then again, should the base Reference class should have those?
public class ContextReference extends Reference {

    protected Context context;


    public ContextReference(Symbol refID, Variable v){
        super(refID,v);
    }
    public ContextReference(Symbol refID, Variable v, Context context){
        super(refID,v);
        this.context = context;
    }

    public ContextReference(Symbol refID, Context context){
        super(refID,new Variable("X:context"));
        this.context = context;
    }

    public void setContext(Context context) {
        this.context = context;
    }

    public Context getContext() {
        return context;
    }
}
