package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;

public interface RLInterface {

    @TRADEService
    @Action
    public Justification callPolicy(String action);

    @TRADEService
    @Action
    public Justification learnPolicy(String action);

    @TRADEService
    @Action
    public Justification updatePolicy(String failedOperator);
}
