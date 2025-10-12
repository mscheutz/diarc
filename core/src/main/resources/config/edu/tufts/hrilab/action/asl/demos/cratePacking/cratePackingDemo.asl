import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import ai.thinkingrobots.mtracs.util.MPose;
import java.util.List;
import java.lang.Long;
import java.lang.Double;
import java.lang.Float;
import java.lang.Integer;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import edu.tufts.hrilab.packing.PackedItem;
import edu.tufts.hrilab.packing.PackingResult3D;
import edu.tufts.hrilab.packing.Quadrilateral;
import edu.tufts.hrilab.packing.RobotAdjustment;
import java.awt.Rectangle;

() = testVision() {
    List !tokenIds;
    Long !tokenId;
    Symbol !refId;
    MemoryObject !mo;
    Point3d !location;
    MPose !goal;
    Double !x;
    Double !y;
    Double !z;
    Double !theta;
    Term !props;
    Symbol !robotone="robotone:agent";
    List !box;
    RobotAdjustment !adjustment;
    MPose !currentPose;

    act:goToPosePrimitive("robotonepose_0:pose");

    act:rotateToEE("camera");

//    !box = tsc:generateOBB();
//    !adjustment = tsc:calculateRobotAdjustment(!box);
//    !x = op:invokeMethod(!adjustment, "getX");
//    !x = op:/(!x, 1000);
//    !y = op:invokeMethod(!adjustment, "getY");
//    !y = op:/(!y, 1000);
    !theta = op:invokeMethod(!adjustment, "getRotation");

    op:log(debug, "[lookForAndMoveToBox] x: !x, y: !y, theta: !theta");
}

() = goToPoseDirect(Symbol ?pose) {
    act:goToPosePrimitiveNoAdjust(?pose);
}

() = doDemo() {
    PackedItem !item;
    Double !centerX;
    Double !centerY;
    Double !angle;
    PackingResult3D !result;
    Float !boxHeight = 0.060;
    Float !zOffset;
    Integer !tmp;
    Integer !itemsPerLayer;
    Integer !numLayers;

    //get crate packing information (including number if items)
    !result = tsc:solvePalletPacking3D();
    op:log(warn, "[doDemo] packing result: !result");
    !numLayers = tsc:getNumLayers();
    !itemsPerLayer = tsc:getNumberOfItemsPerLayer();
    for (!i=0; !i lt !numLayers; !i ++) {
        for (!j=0; !j lt !itemsPerLayer; !j ++) {
            act:goToPosePrimitive("robotonepose_0:pose");
            act:lookForAndMoveToBox();
            tsc:moveZRelative(-0.05);
            tsc:closeGripper();
            tsc:moveZRelative(0.3);
            act:goToPosePrimitive("robotonepose_1:pose");

            //get values from packing info
            !item = tsc:getNextPackedItem();
            !centerY = op:invokeMethod(!item, "getCenterX");
            !centerX = op:invokeMethod(!item, "getCenterY");
            !angle = op:invokeMethod(!item, "getAngle");
            op:log(info, "[doDemo] putting down item at !centerX, !centerY, !angle");
            !tmp = op:+(!i,1);
            !zOffset = op:*(!boxHeight, !tmp);
            act:putDownInCrate(!centerX, !centerY, !zOffset, !angle);
        }
    }
    act:goToPosePrimitive("robotonepose_0:pose");
}

() = lookForAndMoveToBox() {
    List !tokenIds;
    Long !tokenId;
    Symbol !refId;
    MemoryObject !mo;
    Point3d !location;
    MPose !goal;
    Double !x;
    Double !y;
    Double !z;
    Double !theta;
    Term !props;
    Symbol !robotone="robotone:agent";
    List !box;
    RobotAdjustment !adjustment;
    MPose !currentPose;


    act:rotateToEE("camera");

    //52 and 108 are the "threshold" values that seem to work for the contrast-based detection


//    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(object(X:physobj))");
//    op:log(info, "reference for an object to get the search moving with props: !props");
//    !refId = !robotone.act:positReference(!props);
//    op:log(info, "refId: !refId");
//
//    op:log(info, "[lookForAndMoveToBox] looking for box...");
//    (!tokenIds, !tokenId) = act:findObject(!refId);
//    op:log(info, "[lookForAndMoveToBox] done looking for box");
//    op:log(info, "[lookForAndMoveToBox] tokenId: !tokenId, tokenIds: !tokenIds");
//    op:log(info, "[lookForAndMoveToBox] search succeeded");
//    !mo = tsc:getToken(!tokenId);
//
//    !box = op:invokeMethod(!mo, "getBoundingBox");
    !box = tsc:generateOBB();
//    op:log(info, "[lookForAndMovetoBox] bounding box is: !box");
    !adjustment = tsc:calculateRobotAdjustment(!box);
    !x = op:invokeMethod(!adjustment, "getX");
    !x = op:/(!x, 1000);
    !y = op:invokeMethod(!adjustment, "getY");
    !y = op:/(!y, 1000);
    !theta = op:invokeMethod(!adjustment, "getRotation");

    op:log(debug, "[lookForAndMoveToBox] x: !x, y: !y, theta: !theta");

    tsc:moveXRelative(!x);
    tsc:moveYRelative(!y);
    !currentPose = tsc:getCurrentPose();
    !goal = tsc:calculateRelativeMovement(!currentPose, !theta);
    act:rotateToEE("gripper");
    tsc:goToPose(!goal);
}

() = rotateToGripper() {
    act:rotateToEE(gripper);
}

() = rotateToDefault() {
    act:rotateToEE(default);
}

() = putDownInCrate(Float ?dx, Float ?dy, Float ?dz, Float ?dc) {

    MPose !origin;
    Double !height;
    Symbol !robotone = "robotone:agent";

    Float !zero = 0.0f;

    !origin = tsc:getPoseFromSymbol(robotonepose_2:pose);
    act:moveToAdjustedPose(!origin, ?dx, ?dy, ?dz, !zero, !zero, ?dc); //todo: rotate if needed

    !height = op:+(-0.38, ?dz);
    tsc:moveZRelative(!height);
    tsc:openGripper();
    !height = op:*(!height, -1);
    tsc:moveZRelative(!height);
}

() = gripper() {
    act:rotateToEE("gripper");
}

() = camera() {
    act:rotateToEE("camera");
}
