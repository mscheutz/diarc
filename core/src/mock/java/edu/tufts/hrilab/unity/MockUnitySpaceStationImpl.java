/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import java.util.ArrayList;
import java.util.List;

public class MockUnitySpaceStationImpl extends DiarcComponent {



    /**
     * Gets the health of the space station as a float [0:100].
     */
    @TRADEService
    @Action
    public synchronized float getSpaceStationHealth () {
        //todo: pete not sure about the actual behavior on unity side. this is just taking an average.
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationHealth")).call(Float.class);

        } catch (TRADEException e) {
            log.error("[getSpaceStationHealth]", e);
            return -1;
        }
    }

    /**
     * Gets a list of damaged tubes, optionally limited to the area.
     */
    @TRADEService
    @Action
    public synchronized List<String> getSpaceStationTubesDamaged () {
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesDamaged")).call(List.class);
        } catch (TRADEException e) {
            log.error("[getSpaceStationHealth]", e);
            return new ArrayList<>();
        }
    }

    @TRADEService
    @Action
    public synchronized List<String> getSpaceStationTubesDamaged (Symbol area) {
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesDamaged")).call(List.class);
        } catch (TRADEException e) {
            log.error("[getSpaceStationHealth]", e);
            return new ArrayList<>();
        }
    }

    @TRADEService
    @Action
    public synchronized List<Symbol> getWingsWithAlerts () {
        try {
            List<String> alertWingStrings = TRADE.getAvailableService(new TRADEServiceConstraints().name("getWingsWithDamagedTubes")).call(List.class);
            ArrayList<Symbol> alertWings = new ArrayList<>();
            for (String sWing : alertWingStrings) {
                alertWings.add(Factory.createSymbol(sWing));
            }
            return alertWings;
        } catch (TRADEException e) {
            log.error("[getSpaceStationHealth]", e);
            return new ArrayList<Symbol>();
        }
    }

    /**
     * Gets a list of broken tubes, optionally limited to the area.
     */
    @TRADEService
    @Action
    public synchronized List<String> getSpaceStationTubesBroken () {
        return new ArrayList<String>();
    }

    @TRADEService
    @Action
    public synchronized List<String> getSpaceStationTubesBroken (Symbol area) {
        return new ArrayList<String>();
    }

    /**
     * Gets a list of off tubes, optionally limited to a area.
     */
    @TRADEService
    @Action
    public synchronized List<String> getSpaceStationTubesOff () {
        return new ArrayList<String>();
    }

    @TRADEService
    @Action
    public synchronized List<String> getSpaceStationTubesOff (Symbol area) {
        return new ArrayList<String>();
    }

    /**
     * Gets the health of a specific tube via name as a float [0:100].
     */
    @TRADEService
    @Action
    public synchronized float getSpaceStationTubeHealth (Symbol tube) {
        return 0.0F;
    }

    /**
     * Gets the damaged state of a tube, true if damaged.
     */
    @TRADEService
    @Action
    public synchronized boolean getSpaceStationTubeDamaged (Symbol tube) {
        return true;
    }

    /**
     * Gets the broken state of a tube, true if broken.
     */
    @TRADEService
    @Action
    public synchronized boolean getSpaceStationTubeBroken (Symbol tube) {
        return true;
    }

    /**
     * Gets the off state of a tube, true if off.
     */
    @TRADEService
    @Action
    public synchronized boolean getSpaceStationTubeOff (Symbol tube) {
        return true;
    }
}
