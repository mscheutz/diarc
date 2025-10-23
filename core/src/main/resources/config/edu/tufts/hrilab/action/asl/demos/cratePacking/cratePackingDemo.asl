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

() = testGrasp() {
    act:goToPosePrimitive("robotonepose_0:pose");
    act:lookForAndMoveToBox();
    tsc:moveZRelative(-0.087);
}

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
        tsc:incrementPackingItemNum();
        //act:goToPosePrimitive("robotonepose_0:pose");
        act:lookForAndMoveToBox();
        act:graspItemWrapper();
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
        //TODO TODO: can we undo this now that we added incrementPackingItemNum?
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

() = graspItemWrapper() {
    act:graspItem();
}

() = graspItem() {
    tsc:moveZRelative(-0.086);
    act:closeGripper();
    tsc:moveZRelative(0.3);
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
    Integer !failToDetectCount = 0;

    act:rotateToEE("camera");

    !adjustment = tsc:calculateRobotAdjustment();

    if (op:isNull(!adjustment)) {
        //todo: actually say something useful here. these semantics don't have generation examples currently
        act:generateResponse(and(not(see(item, waitingToSee(it)))));
    }
    while (op:isNull(!adjustment)) {
        !failToDetectCount = op:++(!failToDetectCount);
        op:log(error, "[lookForAndMoveToBox] no object found in frame");
        op:log(warn, "[lookForAndMoveToBox] waiting for motion condition to re-detect");
        act:blockingDetectMotionStopStart();
        !adjustment = tsc:calculateRobotAdjustment();
        if (op:gt(!failToDetectCount, 20)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(see(?actor, item))");
            exit(FAIL, !failCond);
        }
    }
    !x = op:invokeMethod(!adjustment, "getX");
    !y = op:invokeMethod(!adjustment, "getY");
    !y = op:+(!y, 25);
    !theta = op:invokeMethod(!adjustment, "getZ");
//    !theta = op:-(!theta, 1.5705);

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

() = putDownInCrateCancel(Integer ?tracker, Double ?height, MPose ?origin, Float ?dx, Float ?dy, Float ?dz, Float ?zero, Float ?zeroAgain, Float ?dc) {
    if (op:equals(?tracker, 1)) {
        act:moveToAdjustedPose(!origin, ?dx, ?dy, ?dz, !zero, !zero, ?dc);
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

    Float !zero = 0.0f;

    !origin = tsc:getPoseFromSymbol(robotonepose_2:pose);
    act:moveToAdjustedPose(!origin, ?dx, ?dy, ?dz, !zero, !zero, ?dc);

    !height = op:+(-0.41, ?dz);
    !heightUp = op:*(!height, -1);
    !tracker = op:++(!tracker);
    tsc:moveZRelative(!height);
    tsc:openGripper();
    tsc:moveZRelative(!heightUp);
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
    act:rotateToEE("camera");
}

 () = openGripper() {
    tsc:openGripper();
 }

 () = closeGripper() {
    Predicate !failCond;
    op:log(debug, "[closeGripper] closing gripper");
    Boolean !result;
    !result = tsc:closeGripper();
    if (op:equals(!result, false)) {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(holding(?actor, item))");
      exit(FAIL_POSTCONDITIONS, !failCond);
    }
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
    }

    op:log(debug, "[closeGripperFailureRecovery] starting recovery");
    act:openGripper();
    tsc:moveZRelative(0.3);
    act:goToPosePrimitive("robotonepose_0:pose");
    act:generateResponse("report(and(failedToGrasp(?actor,item),tryingAgain))");
    act:lookForAndMoveToBox();
    op:log(debug, "[closeGripperFailureRecovery] finished recovery");
}