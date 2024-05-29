package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public interface PythonRLInterface {

    @TRADEService
    @Action
    public boolean callPolicy(int policyNumber);
}
