/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public interface SpotNavigationInterface {

    /**
     * Attempt to reach a named goal location.
     *
     * @param location - name of the goal location
     */
    @TRADEService
    @Action
    Justification goToLocation(Symbol location) ;

    @TRADEService
    @Action
    Justification initLocation(Symbol location);

}
