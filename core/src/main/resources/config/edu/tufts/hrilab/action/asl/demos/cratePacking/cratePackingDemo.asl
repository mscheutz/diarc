import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
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
import edu.tufts.hrilab.packing.Vec3;
import java.lang.Boolean;

() = goToPoseDirect(Symbol ?pose) {
    act:goToPosePrimitiveNoAdjust(?pose);
}

() = packCrate(Integer ?numItems) {
     onInterrupt :{
         cancel: tsc:setCratePackingStatus("canceled");
     }
    PackedItem !item;
    Double !centerX;
    Double !centerY;
    Double !angle;
    PackingResult3D !result;
    Float !boxHeight = 0.060;
    Float !zOffset;
    Integer !tmp;
    Integer !itemsPerLayer;

    //get crate packing information (including number if items)
    !result = tsc:getCachedPackingResult3D();
    op:log(warn, "[doDemo] packing result: !result");
    !itemsPerLayer = tsc:getNumberOfItemsPerLayer();
    tsc:setCratePackingStatus("packing");
    act:goToPosePrimitive("robotonepose_0:pose");
    for (!i=0; !i lt ?numItems; !i ++) {
        //act:goToPosePrimitive("robotonepose_0:pose");
        act:lookForAndMoveToBox();
        tsc:moveZRelative(-0.088);
        tsc:closeGripper();
        tsc:moveZRelative(0.3);
        act:goToPosePrimitive("robotonepose_1:pose");

        //get values from packing info
        !item = tsc:getNextPackedItem();
        !centerY = op:invokeMethod(!item, "getCenterX");
        !centerX = op:invokeMethod(!item, "getCenterY");
        !angle = op:invokeMethod(!item, "getAngle");
        op:log(info, "[doDemo] putting down item at !centerX, !centerY, !angle");

        !tmp = op:/(!i,!itemsPerLayer);
        !tmp = op:+(!tmp,1);
        !zOffset = op:*(!boxHeight, !tmp);
        act:putDownInCrate(!centerX, !centerY, !zOffset, !angle);
        tsc:incrementNumPackedItems();
        //TODO: test on real robot
        //      1. this doesn't break functionality
        //      2. setting the number to pack to the number which is in the crate while the arm is on the way back to
        //          the conveyor breaks from this loop before packing more
        //Putting this at the bottom of the loop so that changing the number of items to pack per crate has a chance
        //  to be caught before picking up the next item
        act:goToPosePrimitive("robotonepose_0:pose");
        ?numItems = tsc:getCurrentCrateCapacity();
    }
    tsc:setCratePackingStatus("complete");
    //TODO: whether we do this or not depends partially on whether we ack or not in main loop
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
    Vec3 !adjustment;
    MPose !currentPose;
    Predicate !failCond;


    act:rotateToEE("camera");

    !adjustment = tsc:calculateRobotAdjustment();
    if (op:isNull(!adjustment)) {
        op:log(error, "[lookForAndMoveToBox] no object found in frame");
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(see(?actor, item))");
        exit(FAIL, !failCond);
    }
    !x = op:invokeMethod(!adjustment, "getX");
    !y = op:invokeMethod(!adjustment, "getY");
    !theta = op:invokeMethod(!adjustment, "getZ");
    !theta = op:-(!theta, 1.5705);

    op:log(debug, "[lookForAndMoveToBox] x: !x, y: !y, theta: !theta");

    !currentPose = tsc:getCurrentPose();
    !goal = tsc:calculateRelativeMovement(!currentPose, !x, !y, !theta);
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

    !height = op:+(-0.392, ?dz);
    tsc:moveZRelative(!height);
    tsc:openGripper();
    !height = op:*(!height, -1);
    tsc:moveZRelative(!height);
}

() = checkHolding() {
    Boolean !holding;
    Predicate !failurePred;

    !holding = tsc:checkHolding();
    !failurePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(holding)");
    if (op:equals(!holding, false)) {
        op:log(debug, "[checkHolding] NOT holding item");
        exit(FAIL, !failurePred);
    }
    op:log(debug, "[checkHolding] holding item");
}

() = gripper() {
    act:rotateToEE("gripper");
}

() = camera() {
    act:rotateToEE("camera");
}

 () = openGripper() {
    tsc:openGripper();
 }

 () = closeGripper() {
    tsc:closeGripper();
 }