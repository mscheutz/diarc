import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import ai.thinkingrobots.mtracs.util.MPose;
import java.util.List;
import java.lang.Long;
import java.lang.Double;
import java.lang.Float;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

() = goToPoseDirect(Symbol ?pose) {
    act:goToPosePrimitiveNoAdjust(?pose);
}

() = doDemo() {
    MPose !pose;
    act:goToPosePrimitive("robotonepose_0:pose");
    act:lookForAndMoveToBox();
    tsc:moveZRelative(-0.05);
    tsc:closeGripper();
    tsc:moveZRelative(0.10);
    act:goToPosePrimitive("robotonepose_1:pose");
    act:goToPosePrimitive("robotonepose_2:pose");
    act:putDownInCrate();
}

() = lookForAndMoveToBox() {
    List !tokenIds;
    Long !tokenId;
    Symbol !refId;
    MemoryObject !mo;
    List !mos;
    Point3d !location;
    MPose !goal;
    Double !x;
    Double !y;
    Double !z;
    Double !theta;
    Long !typeId;
    Term !props;
    Symbol !robotone="robotone:agent";

    act:rotateToEE("default");
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(object(X:physobj))");
    op:log(info, "reference for an object to get the search moving with props: !props");
    !refId = !robotone.act:positReference(!props);
    op:log(info, "refId: !refId");

    op:log(info, "[lookForAndMoveToBox] looking for box...");
    !tokenIds = act:lookForObject(!refId);
    !typeId = tsc:getTypeId(!refId);
    op:log(info, "[lookForAndMoveToBox] done looking for box");
    if (op:isEmpty(!tokenIds)) {
        exit(FAIL, not(found()));
    }
    !tokenId = op:get(!tokenIds, 0);
    op:log(info, "[lookForAndMoveToBox] search succeeded");
    !mos = tsc:getTokens(!tokenId);
    if (op:isEmpty(!mos)) {
        exit(FAIL, not(found()));
    }
    !mo = op:get(!mos, 0);
    !location = op:invokeMethod(!mo, "getLocation");
    op:log(warn, "location reported from MO is !location");
//    act:calculateAndMoveToGrasp(!mo);
    !theta = tsc:calculateAngleOfObject(!mo);

    act:stopAllSearches();
    op:log(info, "[lookForAndMoveToBox] angle calculated to be: !theta");
    !x = op:invokeMethod(!location, "getX");
    !x = op:*(!x, 1000);
    !y = op:invokeMethod(!location, "getY");
    !y = op:*(!y, 1000);
    !z = op:invokeMethod(!location, "getZ");
    !z = op:*(!z, 1000);
    !goal = tsc:calculateRelativeMovement(!x, !y, !z, 0.0);
    act:rotateToEE("gripper");
//    op:log(info, "[lookForAndMoveToBox] detected object at location: !x, !y, !z");
//
    tsc:goToPose(!goal);
    tsc:rotateToVertical();
    tsc:moveCRelative(!theta);
}

() = rotateToGripper() {
    act:rotateToEE(gripper);
}

() = rotateToDefault() {
    act:rotateToEE(default);
}

() = putDownInCrate() {
    //tsc:moveZRelative(-0.2242);
    tsc:openGripper();
    //tsc:moveZRelative(0.2242);
}

() = gripper() {
    act:rotateToEE("gripper");
}

() = camera() {
    act:rotateToEE("default");
}
