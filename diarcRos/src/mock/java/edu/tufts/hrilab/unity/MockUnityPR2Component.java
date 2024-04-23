/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MockUnityPR2Component extends MockMoveBaseComponent {
    @TRADEService
    @Action
    public void repairStepOne(Symbol actor, Symbol tube) {
        log.info("[repairStepOne]" + actor);
    }

    @TRADEService
    @Action
    public void repairStepTwo(Symbol actor, Symbol tube) {
        log.info("[repairStepTwo]" + actor);
    }

    @TRADEService
    @Action
    public void repairStepThree(Symbol actor, Symbol tube) {
        log.info("[repairStepThree]" + actor);
        PoseReference ref = getPoseReference(tube);
        String name = ref.properties.get(0).getName();
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("repairTube")).call(void.class, name);
        } catch (TRADEException e) {
            log.error("[repairStepOne]", e);
        }
    }

    @TRADEService
    @Action
    public List<String> senseTubeStatusInWing(Symbol wing) {
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesDamagedInWing")).call(List.class, wing);
        } catch (TRADEException e) {
            log.error("[senseTubeStatusInWing]", e);
            return new ArrayList<>();
        }
    }

    @TRADEService
    @Action
    public boolean senseIfTubeBroken (Symbol tubeId) {

        PoseReference ref = getPoseReference(tubeId);
        StringBuilder semanticNameBuilder = new StringBuilder();
        for (int i = 1; i < ref.properties.size(); i++) {
            String tmp = ref.properties.get(i).getArgs().get(1).getName();//todo: this is very hacky. required to reassemble the semantic name since name is no longer provided in the json reference file.
            if (i != 1) {
                tmp = tmp.substring(0,1).toUpperCase() + tmp.substring(1);
            }
            semanticNameBuilder.append(tmp);
        }
        log.info("[senseIfTubeBroken] querying for location: " + semanticNameBuilder.toString());
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("isTubeBrokenWithSemanticName")).call(Boolean.class, semanticNameBuilder.toString());
        } catch (TRADEException e) {
            log.error("[senseIfTubeBroken]", e);
            return Boolean.FALSE;
        }
    }

    @TRADEService
    public Point3d getPosition() {
        return new Point3d(950.0,984.0,0.0);
    }

    @TRADEService
    @Observes({"in(X:location,Y:location)","side(X:location,Y:direction)","wingid(X:location,Y:id)"})
    public List<Map<Variable,Symbol>> observeBindings(Term state) {
        //TODO:brad: this is mostly used for verifying effects of steps in plans. If we want it to be more than that we should make it actually work bor bound and und bound queries.
        for(Symbol s:state.getArgs()){
            if(s.isVariable()){
                log.warn("[observeBindings] querries with free vaiables not supported, returning \"true\" "+s+" in state: "+state);
            }
        }
        List<Map<Variable, Symbol>> results = new ArrayList<>();
        Map<Variable, Symbol> m = new HashMap<>();
        results.add(m);
        return results;
    }

    @TRADEService
    @Action
    public void repairLocation(Symbol locationRef) {
        log.info("[repairLocation] location " + locationRef);
    }
}
