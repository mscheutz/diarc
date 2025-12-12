import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import ai.thinkingrobots.mtracs.util.MPose;
import java.util.List;
import java.lang.Long;
import java.lang.Double;
import java.lang.Float;
import java.lang.Integer;
import java.lang.String;
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
    Float !boxHeight;
    Integer !tmp;
    Float !zOffset;
    Integer !itemsPerLayer;

    !tmp = tsc:getPackedItemHeight();
    !boxHeight = op:*(1.0f, !tmp);

    //get crate packing information (including number if items)
    !result = tsc:getCachedPackingResult3D();
    op:log(warn, "[doDemo] packing result: !result");
    !itemsPerLayer = tsc:getNumberOfItemsPerLayer();
    tsc:setCratePackingStatus("packing");
    act:rotateToEE(default);
    act:goToPosePrimitive("robotonepose_0:pose");
    for (!i=0; !i lt ?numItems; !i ++) {
        tsc:incrementPackingItemNum();
        act:lookForAndMoveToBox();
        act:graspItemWrapper();
        //TODO: replace with moving to intermediate safe point
        //act:goToPosePrimitive("robotonepose_0:pose");
        tsc:moveZRelative(0.2);

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
        //Putting this at the bottom of the loop so that changing the number of items to pack per crate has a chance
        //  to be caught before picking up the next item
        act:rotateToEE(default);
        act:goToPosePrimitive("robotonepose_0:pose");
        ?numItems = tsc:getCurrentCrateCapacity();
    }
    tsc:setCratePackingStatus("complete");
    act:goToPosePrimitive("robotonepose_0:pose");
}

() = graspItemWrapper() {
    act:graspItem();
}

() = graspItem() {
    act:closeGripper();
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
    Integer !tmp;

    Double !boxHeight;
    Double !zOffset = 0.0;
    Float !surfaceHeight;

    !adjustment = tsc:calculateRobotAdjustment();

    if (op:isNull(!adjustment)) {
        //todo: actually say something useful here. these semantics don't have generation examples currently
        act:generateResponse(and(not(see(item, waitingToSee(it)))));
    }
    while (op:isNull(!adjustment)) {
        op:log(error, "[lookForAndMoveToBox] no object found in frame");
        op:log(warn, "[lookForAndMoveToBox] waiting for motion condition to re-detect");
        act:blockingDetectMotionStopStart();
        !adjustment = tsc:calculateRobotAdjustment();
    }
    !x = op:invokeMethod(!adjustment, "getX");
    !y = op:invokeMethod(!adjustment, "getY");
    !theta = op:invokeMethod(!adjustment, "getZ");


    op:log(debug, "[lookForAndMoveToBox] x: !x, y: !y, theta: !theta");

    !tmp = tsc:getPackedItemHeight();
    !boxHeight = op:*(1.0, !tmp);
    op:log(debug, "[lookForAndMoveToBox] boxHeight !boxHeight");
    !currentPose = tsc:getPoseFromSymbol(robotonepose_0:pose);
    op:log(debug, "[lookForAndMoveToBox] currentPose (conveyor) !currentPose");
    !surfaceHeight = op:invokeMethod(!currentPose, "getSurfaceHeight");
    !surfaceHeight = op:*(!surfaceHeight, 1000);
    op:log(debug, "[lookForAndMoveToBox]  surfaceHeight !surfaceHeight");
    !boxHeight = op:+(!boxHeight, !surfaceHeight);
    op:log(debug, "[lookForAndMoveToBox] boxHeight !boxHeight");
    !boxHeight = op:-(!boxHeight, 8);
    op:log(debug, "[lookForAndMoveToBox] boxHeight !boxHeight");

    !currentPose = tsc:getCurrentPose();
    op:log(debug, "[lookForAndMoveToBox] currentPose (robot) !currentPose");
    !goal = tsc:calculateRelativeMovement(!currentPose, !x, !y, !theta, !boxHeight, !zOffset);
    op:log(debug, "[lookForAndMoveToBox] goal !goal");
    act:rotateToEE("gripper");
    tsc:goToPose(!goal);
}

() = rotateToGripper() {
    act:rotateToEE(gripper);
}

() = rotateToDefault() {
    act:rotateToEE(default);
}

() = putDownInCrateCancel(Integer ?tracker, Double ?height, MPose ?origin, Float ?dx, Float ?dy, Float ?dz, Float ?zero, Float ?zeroAgain, Float ?dc) {
    if (op:equals(?tracker, 1)) {
        act:moveToAdjustedPose(!origin, ?dx, ?dy, !zero, !zero, !zero, ?dc);
    }
}

() = putDownInCrate(Float ?dx, Float ?dy, Float ?dz, Float ?dc) {
    onInterrupt :{
        cancel: act:putDownInCrateCancel(!tracker, !heightUp, !origin, ?dx, ?dy, ?dz, !zero, !zero, ?dc);
    }

    MPose !origin;
    Double !height;
    Double !heightUp;
    Symbol !robotone = "robotone:agent";
    Integer !tracker = 0;
    Float !surfaceHeight;
    Float !currentZ;

    Float !zero = 0.0f;
    Double !difference;

    !origin = tsc:getPoseFromSymbol(robotonepose_1:pose);
    act:moveToAdjustedPose(!origin, ?dx, ?dy, !zero, !zero, !zero, ?dc);

    !surfaceHeight = op:invokeMethod(!origin, "getSurfaceHeight");
    !surfaceHeight = op:*(!surfaceHeight, 1000);
    !height = op:+(!surfaceHeight, ?dz);
    op:log(error, "height !height");

    !currentZ = op:invokeMethod(!origin, "getZ");

    !difference = op:-(!currentZ, !height);
    !difference = op:-(!difference, 25); //To prevent items from flipping during tight packing
    !difference = op:/(!difference, 1000);


    !heightUp = op:*(!difference, -1);
    !tracker = op:++(!tracker);
    op:log(info, "[putDownInCrate] dz: ?dz, surfaceHeight: !surfaceHeight, currentZ: !currentZ, height: !height, difference: !difference, heightUp: !heightUp");
    tsc:moveZRelative(!heightUp);
    tsc:openGripper();
    //TODO: can be removed potentially - or replaced with moveTo safe spot on crate edge
    tsc:moveZRelative(!difference);
    !tracker = op:++(!tracker);
    op:log(debug, "done putting down crate");
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
    act:rotateToEE(default);
}

 () = openGripper() {
    tsc:openGripper();
 }

 () = closeGripper() {
    Predicate !failCond;
    op:log(debug, "[closeGripper] closing gripper");
    Boolean !result;
    !result = tsc:closeGripper();
//    if (op:equals(!result, false)) {
//      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(holding(?actor, item))");
//      exit(FAIL_POSTCONDITIONS, !failCond);
//    }
 }

// Demo Utilities //
//Submit these using a separate "simspeech" GUI which doesn't go through dialogue? Or have "mute" action?
() = resetDemo() {
    //TODO: have safe onCancel method for packing? or at least open/closeGripper in LLM?
    //Cancel current task if exists? - or cant submit this while packing? - if having separate component then can submit this separately

    op:log("info","[resetDemo] in method");

    //Cancel all question asking
    act:clearQuestions();

    //Clear belief and packing component state
    act:clearPackingParams();

    //Move back to conveyor location?
    act:goToPosePrimitive("robotonepose_0:pose");
}

() = clearCurrentCrate() {
    Predicate !fact;

    op:log("info","[clearCurrentCrate] in method");

    !fact = op:createPredicate("numItemsPerCrate(X)");
    act:retractBelief(!fact);

    tsc:resetCurrentCrate();
}

() = clearPackingParams() {
    Predicate !fact;

    op:log("info","[clearPackingParams] in method");

    //Clear belief
    !fact = op:createPredicate("numItemsPerCrate(X)");
    act:retractBelief(!fact);
    !fact = op:createPredicate("ranPackingAlgorithmWithUpdatedSettings(?actor)");
    act:retractBelief(!fact);

    tsc:resetPackingState();
}

//Setting item/crate params can be done through the GUI

() = setNumItemsPerCrate(Integer ?numItems) {
    Predicate !fact;

    op:log("info","[setNumItemsPerCrate] in method ?numItems");

    //Clear belief
    !fact = op:createPredicate("numItemsPerCrate(?numItems)");
    act:assertBelief(!fact);
}

() = clearQuestions() {
    String !question;
    List !responseForms;

    op:log("info","[clearQuestions] in method");

    //Cancel all question asking
    !question = tsc:getLastQuestion(?actor);
    while (~op:invokeMethod(!question,"isEmpty")) {
        !responseForms = tsc:getLastQuestionResponseForm(?actor);
        op:log("info","[resetDemo] canceling question !question");
        tsc:cancelWaitForResponse(!question,!responseForms,?actor);
        !question = tsc:getLastQuestion(?actor);
    }
}

(edu.tufts.hrilab.packing.PackingResult3D ?result) = runSolver() {
    Predicate !fact;
    op:log("info","[runSolver] in method");

    ?result = tsc:solveCratePacking3D();
    if (op:isNull(?result)) {
        op:log("info","[runSolver] result is null");
    } else {
        !fact = op:createPredicate("numItemsPerCrate(X)");
        act:retractBelief(!fact);
        !fact = op:createPredicate("numItemsPerCrate(-1)");
        act:assertBelief(!fact);
        !fact = op:createPredicate("ranPackingAlgorithmWithUpdatedSettings(?actor)");
        act:retractBelief(!fact);
    }
}

() = closeGripperFailureRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {graspItem(A)}
            postRecovery : postrecovery_retry
        }
        failureReasons : {holding(X,Y)}
        maximumAttempts : 99
    }

    op:log(debug, "[closeGripperFailureRecovery] starting recovery");
    act:openGripper();
    act:rotateToEE(default);
    act:goToPosePrimitive("robotonepose_0:pose");
    act:generateResponse("report(and(failedToGrasp(?actor,item),tryingAgain))");
    act:lookForAndMoveToBox();
    op:log(debug, "[closeGripperFailureRecovery] finished recovery");
}

() = getCalibrationImages() {

    MPose !currentPose;

    act:goToPoseDirect(robotonepose_0);
    !currentPose = tsc:getPoseFromSymbol(robotonepose_0:pose);

    act:rotateToEE("default");

    tsc:saveImage();
    tsc:moveCRelative(1.5708);
    tsc:saveImage();
    tsc:moveCRelative(-3.1415);
    tsc:saveImage();
    tsc:moveCRelative(1.5708);

    tsc:moveZRelative(-0.05);
    tsc:saveImage();
    tsc:moveCRelative(1.5708);
    tsc:saveImage();
    tsc:moveCRelative(-3.1415);
    tsc:saveImage();
    tsc:moveCRelative(1.5708);

    tsc:moveZRelative(0.10);
    tsc:saveImage();
    tsc:moveCRelative(1.5708);
    tsc:saveImage();
    tsc:moveCRelative(-3.1415);
    tsc:saveImage();
    tsc:moveCRelative(1.5708);

}

() = getCalibrationImagesManual() {

    MPose !currentPose;

    act:goToPoseDirect(robotonepose_0);
    !currentPose = tsc:getPoseFromSymbol(robotonepose_0:pose);

    act:rotateToEE("default");

    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();

    tsc:moveZRelative(0.075);
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();

    tsc:moveZRelative(-0.150);
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();
    !bindings = act:askQuestionFromString(?actor, "is it ready", val(X));
    tsc:saveImage();

}

() = runGraspCalibration() {
    MPose !pose;
    Float !surfaceHeight;
    Double !destHeight;
    Integer !tmp;
    Double !goalZ;
    Vec3 !adjustment;
    Boolean !true = true;
    Float !poseZ;
    Double !x;
    Double !y;
    Double !theta;
    List !responseFormList;
    Predicate !responseForm;
    Predicate !failure;

    act:rotateToEE(default);
    act:goToPoseDirect(robotonepose_0:pose);

    !pose = tsc:getPoseFromSymbol(robotonepose_0:pose);
    !tmp = tsc:getPackedItemHeight();
    !destHeight = op:*(1.0, !tmp);
    !surfaceHeight = op:invokeMethod(!pose, "getSurfaceHeight");
    !surfaceHeight = op:*(!surfaceHeight, 1000);
    !destHeight = op:+(!destHeight, !surfaceHeight);

    !pose = tsc:getCurrentPose();
    //TODO: just setZ instead
    !pose = tsc:calculateRelativeMovement(!pose, 0.0, 0.0, 0.0, !destHeight, 0.0);

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("val(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("replaced(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("didIt(X)");
    op:add(!responseFormList,!responseForm);

    act:closeGripper();
    act:askQuestion(?actor, ask(ackWhen(placed(item, gripper))), !responseFormList);
    act:rotateToEE(gripper);
    tsc:goToPose(!pose);
    act:openGripper();
    tsc:moveZRelative(0.05);
    act:rotateToEE(default);
    act:goToPoseDirect(robotonepose_0:pose);
    !adjustment = tsc:calculateRobotAdjustment(!true);
    if (op:isNull(!adjustment)) {
        //todo: actually say something useful here. these semantics don't have generation examples currently
        !failure = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(see(item))");
        exit(FAIL_PRECONDITIONS, !failure);
    }
    !x = op:invokeMethod(!adjustment, "getX");
    !y = op:invokeMethod(!adjustment, "getY");
    !theta = op:invokeMethod(!adjustment, "getZ");

    tsc:setTCPAdjustFromError(!x, !y, !theta);
    op:log(info, "[adjustForDetectionOffset] adjustment set to detection result: !x (x), !y (y), !theta (theta)");
}


() = testAutoCalibration() {
    tsc:runFullAutoCalibration(robotonepose_0:pose,-50.0);
}
