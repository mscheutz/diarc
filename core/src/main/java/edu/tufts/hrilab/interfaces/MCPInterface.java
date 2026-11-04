package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public interface MCPInterface {
    /**
     * Send goal to planner
     */
    @TRADEService
    @Action
    void send_to_planner(String parsedText);
}
