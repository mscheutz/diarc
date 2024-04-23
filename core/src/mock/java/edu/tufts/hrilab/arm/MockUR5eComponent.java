/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.arm;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public class MockUR5eComponent extends MockArmComponent {


    // If we want/need pour to be a static angle we can eliminate the ang parameter
    //TODO: Will: Remove angle parameter?
    @TRADEService
    @Action
    public boolean startPouring(double ang) {
        log.info("[startPouring] ang:" + ang);
        return true;
    }

    @TRADEService
    @Action
    public boolean stopPouring() {
        log.info("[stopPouring]");
        return true;
    }

}
