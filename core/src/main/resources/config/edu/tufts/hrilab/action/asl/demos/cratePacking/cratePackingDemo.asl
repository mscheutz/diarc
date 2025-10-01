import edu.tufts.hrilab.fol.Symbol;
import ai.thinkingrobots.mtracs.util.MPose;
import java.util.List;
import java.lang.Long;
import java.lang.Double;
import java.lang.Float;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import javax.vecmath.Point3d;

() = goToPoseDirect(Symbol ?pose) {
    act:goToPosePrimitiveNoAdjust(?pose);
}

() = doDemo() {
    MPose !pose;
    act:goToPosePrimitive("robotonepose_0:pose");
    act:lookForAndMoveToBox();
}

() = lookForAndMoveToBox() {
    List !tokenIds;
    Long !tokenId;
    Symbol !refId = "physobj_0:physobj";
    MemoryObject !mo;
    List !mos;
    Point3d !location;
    MPose !goal;
    Double !x;
    Double !y;
    Double !z;
    Float !a = 0.0;
    Float !b = 0.0;
    Float !c = 0.0;

    op:log(info, "[lookForAndMoveToBox] looking for box...");
    !tokenIds = act:lookForObject("physobj_0:physobj");
    op:log(info, "[lookForAndMoveToBox] done looking for box");
    !tokenId = op:get(!tokenIds, 0);
    if (op:isNull(!tokenId)) {
        exit(FAIL, not(found()));
    }
    op:log(info, "[lookForAndMoveToBox] search succeeded");
    !mos = tsc:getTokens(!tokenId);
    !mo = op:get(!mos, 0);
    if (op:isNull(!mo)) {
        exit(FAIL, not(found()));
    }
    !location = op:invokeMethod(!mo, "getLocation");
    !x = op:invokeMethod(!location, "getX");
    !y = op:invokeMethod(!location, "getY");
    !z = op:invokeMethod(!location, "getZ");
    op:log(info, "[lookForAndMoveToBox] detected object at location: !x, !y, !z");

    !goal = op:newObject("ai.thinkingrobots.mtracs.util.MPose", !x, !y, !z, !a, !b, !c);
    tsc:moveToRelative(!goal);
}
