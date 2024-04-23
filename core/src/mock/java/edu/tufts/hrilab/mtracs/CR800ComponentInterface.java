/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.mtracs.consultant.vision.CognexReference;
import edu.tufts.hrilab.mtracs.util.CognexJob;
import edu.tufts.hrilab.mtracs.util.CognexResult;
import edu.tufts.hrilab.mtracs.util.MPose;

import javax.vecmath.Point3d;
import java.util.List;
import java.util.Map;

public interface CR800ComponentInterface {
    @TRADEService
        //TODO: Will: This isn't really useful because the controller side can't be connected to multiple times - this might be fixable
        //TODO: could this be done with the onError functionality in cr800?
    boolean reconnect();

    @TRADEService
    boolean moveInPlane( Symbol x,  Symbol y);

    @TRADEService
    boolean moveAbove(Symbol s);

    @TRADEService
    boolean moveZRelative( double dist);

    @TRADEService
    boolean moveXRelative( double dist);

    @TRADEService
    boolean closeGripper();

    @TRADEService
    boolean openGripper();

    @TRADEService
    boolean pickupScrew();

    @TRADEService
    boolean tightenScrew(Symbol screwType);

    @TRADEService
    boolean shankOut();

    @TRADEService
    boolean shankIn();

    @TRADEService
    MPose getCurrentPose();

    @TRADEService
    Symbol recordPose(Symbol poseName, Symbol surfaceHeight);

    @TRADEService
    public Symbol recordPose(Symbol poseName, MPose pose, Symbol surfaceHeight);

    @TRADEService
    MPose getPoseFromSymbol(Symbol poseName);

    @TRADEService
    boolean goToPose(MPose goal);

    @TRADEService
    boolean goToPoseLong( MPose goal);

    @TRADEService
    //Todo: Will: Perhaps this helper code should just be implemented in asl? If so,
    //we'd need to transfer this offset information to belief
    MPose adjustPoseToCameraHeight(MPose goal, Symbol cameraHeight);

    @TRADEService
    void bindToSurface(MPose goal, Symbol surfaceHeight);

    @TRADEService
    MPose getTCPForEE(Symbol eeType);

    @TRADEService
    boolean alternateEE(MPose newTCP);

    @TRADEService
    boolean acceptEE();

    @TRADEService
    boolean ejectEE();

    @TRADEService
    MPose getGripDropoff(Symbol s);

    @TRADEService
    boolean rotateForToolRack();

    @TRADEService
    boolean moveAwayFromJointLimit();

    @TRADEService
    boolean undoCognexOffset();

    @TRADEService
    boolean doCognexOffset();

    @TRADEService
    List<CognexResult> getCameraData( String jobName);

    @TRADEService
    boolean moveToCognexTarget(List<CognexResult> results, int jobIndex);

    @TRADEService
    boolean moveToCognexTarget( Symbol refID);

    @TRADEService
    boolean moveAndOrientToCognexTarget( Symbol refID);

    @TRADEService
    void bindCognexResult(CognexReference ref, CognexResult result, int indexIntoCognexResult);

    @TRADEService
    CognexResult getMatchingResult( CognexReference toReBind, List<CognexResult> results);

    @TRADEService
    List<Term> getEmptyProps();

    @TRADEService
    CognexReference createCogRefWithProps(CognexJob j, List<Term> additionalProperties);

    @TRADEService
    CognexJob getCognexJobForCognexReference(CognexReference ref);

    @TRADEService
    CognexReference getCognexReferenceForID(Symbol refId);

    @TRADEService
    Symbol getDescriptorForID(Symbol refId);

    @TRADEService
    CognexJob getCognexJobForDescriptor(Symbol descriptor);


    boolean eeOrientTo(Point3d g);

    @TRADEService
    List<String> getMHeader();

    @TRADEService
        //Todo: Will: This could possibly be a bit overboard, and introduces more dependencies across annotations,
        //but upon learning that you cannot have variables in annotations, I think this is the best way to make sure
        //each constant is only changed in one place. I also think this would be easier to move to belief if we do
        //that in the future
    Map<String, String> getConstantMappings();

    @TRADEService
    String getModelType();

    @TRADEService
    boolean delay( double seconds);

    boolean moveToRelative(MPose g);

    //todo:(pete) should this be done here? it feels like there should be a better way to force a reference to no longer be salient?
    @TRADEService
    boolean removeCognexReferenceWithID(Symbol refID);
}
