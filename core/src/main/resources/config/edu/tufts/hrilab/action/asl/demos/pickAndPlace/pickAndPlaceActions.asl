import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.util.Map;
import java.lang.String;
import edu.tufts.hrilab.action.justification.Justification;
import java.lang.Double;
import java.lang.Integer;

() = openGripper() {
    Symbol !heldObject;

    !heldObject = act:getHeldObject();
    if (~op:isNull(!heldObject)) {
        exit(FAIL,holding(?actor,!heldObject));
    }
    tsc:openGripper();
}

() = closeGripper() {
    tsc:closeGripper();
}

(List ?bindings) = perceiveEntities() {
    //runs a job that looks for all items known to the system and returns the set of (orderd) results. then binds the associated refrerences.

    Symbol !location;
    Predicate !fact;

    java.lang.Double !objectHeightM = 0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED
    java.lang.Double !negObjectHeightM = -0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED

    !location = act:getCurrentLocation();

    tsc:moveZRelative(!objectHeightM);
    ?bindings = tsc:perceiveAllEntitiesInView("blocksDet");
    tsc:moveZRelative(!negObjectHeightM);

    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,!location)");
    act:retractBelief(!fact);
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,!location)");
    act:assertBelief(!fact);
    foreach (!result : ?bindings) {
        !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor,!result)");
        act:retractBelief(!fact);
        !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result,X)");
        act:retractBelief(!fact);
        !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result,!location)");
        act:assertBelief(!fact);
    }
}

//todo: (pete) allows the LLM target for semantics to be more sane than "getOn". should be true everywhere.
() = getObjectTo(Symbol ?object, Symbol ?destination) {
    Symbol !objectPose;
    Predicate !failCond;
    Predicate !objectAtPred;
    Symbol !descriptor;
    Symbol !holding;
    Predicate !goalPred;

    act:assertIsPhysobj(?object);
    act:assertIsPose(?destination);
    !objectPose = act:getObjectPose(?object);
    !holding = act:checkHolding(?object);
    if (op:equalsValue(!holding, false)) {
        if (op:isNull(!objectPose)) {
            !descriptor = tsc:getDescriptorForID(?object);
            !objectPose = act:askForObjectPose(!descriptor);
            if (op:equals(!objectPose,none)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,theLocationOf,?object))");
                exit(FAIL, !failCond);
            } else {
                !objectAtPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,!objectPose)");
                act:assertBelief(!objectAtPred);
            }
        }
    }

    Symbol !heldObject;
    Symbol !openPose;
    //First put down any held object because the planner doesn't handle that gracefully (e.g. could try to put it down
    //  at a full location and fail/enter recovery for something totally unnecessary)
    !heldObject = act:getHeldObject();
    if (~op:isNull(!heldObject)) {
        if(~op:equalsValue(!heldObject, ?object)) {
            !openPose = act:getPoseWithSpace(?destination);
            if (op:isNull(!openPose)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,putDownObject)");
                exit(FAIL, !failCond);
            }
            op:log(info,"moving held object to open location");
            !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, !openPose)");
            goal:!goalPred;
            act:putDown();
        }
    }

    //TODO: get rid of this doubly nested action wrapping of goalspecs. Interacts badly with recovery - need to
    //      specifically exclude the parent from any goal recovery policies
    //  ^Is this actually just due to bad implementation for goal recovery w/ goalspecs in general?
    act:getOn(?object, ?destination);
}

() = pickUp() {
    Symbol !default = "default";
    Symbol !gripper = "gripper";
    List !objectList;
    Symbol !object;
    Symbol !location;
    Integer !resultsSize;
    Predicate !failCond;
    Symbol !heldObject;

    !heldObject = act:getHeldObject();
    if (~op:isNull(!heldObject)) {
        exit(FAIL_PRECONDITIONS,holding(?actor,!heldObject));
    }

    op:log(info, "[pickUp]");
    !objectList = act:perceiveEntities();

    !resultsSize = op:invokeMethod(!objectList, "size");
    op:log(debug, "[pickUp] results size: !resultsSize for object list !objectList");

    if (op:equals(!resultsSize, 0)) {
        //No results
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "dont(?actor,see,anyObjects,atMyLocation)");
        exit(FAIL_RETURNVALUE,!failCond);
    } elseif (op:equals(!resultsSize, 1)) {
        //Single result - pick up object
        !object = op:get(!objectList, 0);
        !location = act:getCurrentLocation();
        act:grabPerceivedEntity(!object,!location);
    } else {
        //Multiple results - ask which one to pick up
        !object = act:askForObjectToPickUp();
        if (~op:equalsValue(!object, none)) {
            !location = act:getCurrentLocation();
            act:grabPerceivedEntity(!object,!location);
        } else {
            act:reportNotPickingUpObject();
        }
    }
}

() = pickUp(Symbol ?object) {
    Symbol !objectPose;
    Symbol !armPose;
    Symbol !heldObject;
    Predicate !failCond;
    Predicate !thisStep;
    edu.tufts.hrilab.action.justification.Justification !failureJustification;

    op:log(info, "[pickUp] ?object");
    act:assertIsPhysobj(?object);

    !heldObject = act:getHeldObject();
    if (~op:isNull(!heldObject)) {
        !thisStep =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "pickUp(?actor, ?object)");
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(holding(?actor,!heldObject))");
        !failureJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
        op:invokeMethod(!failureJustification, "setStep", !thisStep);
        exit(FAIL_PRECONDITIONS,!failureJustification);
    }

    !objectPose = act:getObjectPose(?object);
    !armPose = act:getCurrentLocation();
    if (~op:isNull(!objectPose) && ~op:equals(!armPose,!objectPose)) {
        op:log(debug, "going to !objectPose");
        act:goToPose(!objectPose);
        act:perceiveEntity(?object);
        act:grabPerceivedEntity(?object,!objectPose);
    } else {
        op:log(debug, "looking at !armPose");
        act:perceiveEntity(?object);
        act:grabPerceivedEntity(?object,!armPose);
    }
}

() = putDown(Symbol ?object:physobj) {
    Symbol !heldObject;
    act:assertIsPhysobj(?object);

    !heldObject = act:getHeldObject();
    if (~op:equals(?object,!heldObject)) {
        exit(FAIL_PRECONDITIONS,not(holding(?actor,?object)));
    }

    act:putDown();
}

() = putDown() {
    Symbol !location;
    Symbol !heldObject;

    op:log(info, "[putDown]");

    !heldObject = act:getHeldObject();
    op:log(debug, "[putDown] held object is !heldObject");
    if (op:isNull(!heldObject)) {
        exit(FAIL_PRECONDITIONS,not(holding(?actor,anything)));
    }

    !location = act:getCurrentLocation();
    act:putdown(!heldObject,!location);
}

() = moveObjectTo(Symbol ?object:physobj, Symbol ?start:pose, Symbol ?destination:pose) {
    Symbol !objectLocation;

    !objectLocation = act:getObjectPose(?object);
    if (~op:equalsValue(!objectLocation, ?start)) {
        exit(FAIL_PRECONDITIONS,not(at(?object,?start)));
    }

    act:assertIsPhysobj(?object);
    act:assertIsPose(?start);
    act:assertIsPose(?destination);

    act:goToPose(?start);
    act:pickUp(?object);
    act:goToPose(?destination);
    act:putDown();
}

() = pickUpWhileHoldingObjectRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : moveObjectTo(A,X,Y,Z)
            postRecovery : postrecovery_success
        }
        failureReasons : {not(holding(X,Y))}
    }

    Symbol !response;
    Symbol !pose;
    Symbol !currentPose;
    Symbol !blockToPickUp;

    op:log("debug","[pickUpWhileHoldingObjectRecovery");
    op:log("debug","[pickUpWhileHoldingObjectRecovery] ?failedActionPredicates");
    op:log("debug","[pickUpWhileHoldingObjectRecovery] ?failureReasons");
    op:log("debug","[pickUpWhileHoldingObjectRecovery] ?failedGoal");

    !currentPose = act:getCurrentLocation();
    !response = act:reportHoldingObjectAndAskToPutDown();
    if (op:equals(!response,yes)) {
        !pose = act:askForPose();
        if (op:equals(!pose,none)) {
            exit(FAIL_RECOVERY);
        } else {
            act:goToPose(!pose);
            act:putDown();
            act:goToPose(!currentPose);
            //only here because of ASL bug
            !blockToPickUp = op:invokeMethod(?failedGoal,"get",1);
            act:pickUp(!blockToPickUp);
        }
    } else {
        exit(FAIL_RECOVERY);
    }
}

() = pickUpCantDetectObjectRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {at(X,Y)}
        failureReasons : {not(dont(A,see,X,atMyLocation))}
    }
    //TODO: should fix value/negation of failure condition

    Symbol !pose;
    Symbol !blockToPickUp;
    Symbol !blockDescriptor;

    op:log("debug","[pickUpCantDetectObjectRecovery");
    op:log("debug","[pickUpCantDetectObjectRecovery] ?failedActionPredicates");
    op:log("debug","[pickUpCantDetectObjectRecovery] ?failureReasons");
    op:log("debug","[pickUpCantDetectObjectRecovery] ?failedGoal");

    !blockToPickUp = op:invokeMethod(?failedGoal,"get",0);
    !blockDescriptor = tsc:getDescriptorForID(!blockToPickUp);
    !pose = act:reportFailedDetectionAndAskForObjectPose(!blockDescriptor);
    if (op:equals(!pose,none)) {
        exit(FAIL_RECOVERY);
    } else {
        act:believeObjectAt(!blockToPickUp,!pose);
        act:pickUp(!blockToPickUp);
    }
}

() = swapObjects(Symbol ?objectOne, Symbol ?objectTwo) {
    Predicate !goalPred;
    Predicate !failCond;
    Symbol !openPose;
    Symbol !objectOnePose;
    Symbol !objectTwoPose;
    Symbol !heldObject;
    java.lang.Integer !numObjects1;
    java.lang.Integer !numObjects2;
    java.lang.Boolean !true = True;
    java.lang.Boolean !false = False;

    if (op:equals(?objectOne,?objectTwo)) {
        act:reportObjectsToSwapAreTheSame();
        return;
    }

    //Make sure object locations are known (and check if one of them is currently being held)
    !heldObject = act:getHeldObject();
    !objectOnePose = act:getObjectPose(?objectOne);
    !objectTwoPose = act:getObjectPose(?objectTwo);
    if (~op:isNull(!heldObject)) {
        if (op:equals(!heldObject,?objectOne)) {
            !objectOnePose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "holding");
        } elseif (op:equals(!heldObject,?objectTwo)) {
            !objectTwoPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "holding");
        }
    }

    //Fail if either object is not present in the work area
    if (op:isNull(!objectOnePose)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,theLocationOf,?objectOne))");
        exit(FAIL, !failCond);
    } elseif (op:isNull(!objectTwoPose)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,theLocationOf,?objectTwo))");
        exit(FAIL, !failCond);
    } elseif (op:equalsValue(!objectOnePose,!objectTwoPose)) {
        act:reportObjectsAtSameLocation();
        return;
    }

    if (~op:isNull(!heldObject)) {
        if (op:equals(?objectOne,!heldObject)) {
            act:swapHeldObject(!heldObject,?objectTwo,!objectTwoPose);
            return;
        } elseif (op:equals(?objectTwo,!heldObject)) {
            act:swapHeldObject(!heldObject,?objectOne,!objectOnePose);
            return;
        } else {
            //Move held object to open location
            !openPose = act:getPoseWithSpace(!objectOnePose);
            if (op:isNull(!openPose)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapObjects)");
                exit(FAIL, !failCond);
            }
            op:log(info,"moving held object to open location");
            !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, !openPose)");
            goal:!goalPred;
            act:putDown();
        }
    }

    !numObjects1 = act:getNumberOfObjectsAt(!objectOnePose);
    !numObjects2 = act:getNumberOfObjectsAt(!objectTwoPose);
    if (op:gt(!numObjects1,1) && op:gt(!numObjects2,1)) {
        act:swapObjectsHelper(?objectOne,!objectOnePose,?objectTwo,!objectTwoPose, !true);
    } elseif (op:gt(!numObjects1,1)) {
        act:swapObjectsHelper(?objectOne,!objectOnePose,?objectTwo,!objectTwoPose, !false);
    } else {
        act:swapObjectsHelper(?objectTwo,!objectTwoPose,?objectOne,!objectOnePose, !false);
    }

    //Pick original held object back up
    if (~op:isNull(!heldObject)) {
        op:log(info,"picking original object back up");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, !heldObject)");
        goal:!goalPred;
    }
}

//TODO: is there a bug with assigning args in calls between asl scripts where args have the same names but in a different order?
() = swapObjectsHelper(Symbol ?object1, Symbol ?object1Pose, Symbol ?object2, Symbol ?object2Pose, java.lang.Boolean ?poseFull) {
    Predicate !goalPred;
    Predicate !failCond;
    Symbol !openPose;

    if (op:equals(?poseFull,true)) {
        //Move object two temporarily to an open location
        !openPose = act:getPoseWithSpace(?object2Pose);
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapObjects)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving object two to open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object2, !openPose)");
        goal:!goalPred;
    }

    //move object one to object two's pose
    op:log(info,"moving object one to desired location");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object1, ?object2Pose)");
    goal:!goalPred;

    //Move object two to object one's location
    op:log(info,"moving object two to desired location");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object2, ?object1Pose)");
    goal:!goalPred;
}

() = swapHeldObject(Symbol ?heldObject, Symbol ?objectToSwap, Symbol ?objectToSwapPose) {
    Predicate !goalPred;
    java.lang.Integer !numObjects;
    Predicate !failCond;

    !numObjects = act:getNumberOfObjectsAt(?objectToSwapPose);
    if (op:gt(!numObjects,1)) {
        //Move held object to open location
        !openPose = act:getPoseWithSpace(?objectToSwapPose);
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapObjects)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving held object to open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, !openPose)");
        goal:!goalPred;
        act:putDown();

        //Move object to swap temporarily to an open location
        !openPose = act:getPoseWithSpace(?objectToSwapPose);
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapObjects)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving object to swap to an open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?objectToSwap, !openPose)");
        goal:!goalPred;
    }

    //Move held object to swap objects location
    op:log(info,"moving held object to desired location");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?heldObject, ?objectToSwapPose)");
    goal:!goalPred;

    //Pick object to swap up
    op:log(info,"pick swapped object up");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, ?objectToSwap)");
    goal:!goalPred;
}

//This currently applies to all goals and compound actions. putDown() and putDown(object) are allowed to just fail. Do we want to change this?
() = putDownAtFullLocationRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_success
        }
        failureReasons : {has(theLocationBelowMe,space)}
        excludedGoals : {putDown(X), putDown(X,Y)}
    }
    Symbol !response;
    Symbol !otherObject;
    String !otherObjectName;
    Symbol !object;
    Symbol !currentPose;
    Symbol !otherObjectPose;

    op:log("debug","[putDownAtFullLocationRecovery");
    op:log("debug","[putDownAtFullLocationRecovery] ?failedActionPredicates");
    op:log("debug","[putDownAtFullLocationRecovery] ?failureReasons");
    op:log("debug","[putDownAtFullLocationRecovery] ?failedGoal");

    !object = act:getHeldObject();
    !currentPose = act:getCurrentLocation();
    //TODO: populate question with held object and current location?
    //  report that we cannot put down the object because the location is full
    //  Ask if the user would like to swap the held object with a object in that location
    !response = act:reportFullLocationAndAskToSwap();
    if (op:equals(!response,yes)) {
        !otherObject = act:askForObjectToSwap();
        !otherObjectName = op:invokeMethod(!otherObject, "getName");
        if (op:equals(!otherObjectName,"none")) {
            act:reportNotSwappingObjects();
            exit(FAIL_RECOVERY);
        } else {
            !otherObjectPose = act:getObjectPose(!otherObject);
            if (~op:equals(!currentPose,!otherObjectPose)) {
                act:reportObjectToSwapNotAtCurrentPose();
                exit(FAIL_RECOVERY);
            }
            act:swapObjects(!object,!otherObject);
        }
    } else {
        act:reportNotSwappingObjects();
        exit(FAIL_RECOVERY);
    }
}

() = placeObjectDownAt(Symbol ?object:physobj, Symbol ?destination:pose) {
    Symbol !heldObject;

    act:assertIsPhysobj(?object);
    act:assertIsPose(?destination);

    !heldObject = act:getHeldObject();
    if (~op:equals(?object,!heldObject)) {
        exit(FAIL_PRECONDITIONS,not(holding(?actor,?object)));
    }

    act:goToPose(?destination);
    act:putDown();
}

() = placeObjectDownAt(Symbol ?destination:pose) {
    Symbol !heldObject;

    act:assertIsPose(?destination);

    !heldObject = act:getHeldObject();
    if (op:isNull(!heldObject)) {
        exit(FAIL_PRECONDITIONS,not(holding(?actor,anything)));
    }

    act:goToPose(?destination);
    act:putDown();
}

//TODO
//() = resetDomain() {
//    conditions : {
//        pre : free(?actor); //TODO can have recovery asking user where to put object down
//    }
//}

() = detectObject(edu.tufts.hrilab.fol.Symbol ?refID) {
    Symbol !objectDescriptor;

    act:perceiveEntity(?refID);
    !objectDescriptor = tsc:getDescriptorForID(?refID);
    act:reportSeenObject(!objectDescriptor);
}

() = detectObjects() {
    List !bindings;
    Symbol !ref;
    Symbol !armPose;
    Symbol !descriptor;

    !bindings = act:perceiveEntities();
    if (op:isEmpty(!bindings)) {
        !armPose = act:getCurrentLocation();
        !descriptor = tsc:getDescriptorForPoseID(!armPose);
        act:reportObjectsNotAtPose(!descriptor);
    } else {
        foreach (!ref : !bindings) {
            !descriptor = tsc:getDescriptorForID(!ref);
            act:reportSeenObject(!descriptor);
        }
    }
}

() = handleSeeObjectsITK(Symbol ?actor) {
    effects : {
        success : knows(?actor,whichObjectsAtCurrentPose(?actor));
    }
    List !activeGoals;
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    List !objects;
    Symbol !object;
    Symbol !currentPose;
    Symbol !objectDescriptor;
    Symbol !poseDescriptor;
    Predicate !ack = "ack(okay)";
    Symbol !response;
    java.lang.Boolean !knowObjectAt = false;
    Variable !queryVar = X;
    java.util.HashMap !binding;

    op:log(info,"handleSeeObjectsITK");

    !currentPose = act:getCurrentLocation();
    !poseDescriptor = tsc:getDescriptorForPoseID(!currentPose);
    !objects = act:getObjectsAtPose(!currentPose);

    foreach (!binding : !objects) {
        !object = op:get(!binding,!queryVar);
        !objectDescriptor = tsc:getDescriptorForID(!object);
        op:log(info,!object);
        op:log(info,!objectDescriptor);
        if (~op:equalsValue(!objectDescriptor, none)) {
            act:reportObjectBelievedToBeAtPose(!objectDescriptor,!poseDescriptor);
            !knowObjectAt = op:newObject("java.lang.Boolean", true);
        }
    }
    if (op:equals(!knowObjectAt,False)) {
        act:reportDontBelieveObjectsAtPose(!poseDescriptor);
    }

    !response = act:askToVerifyBelief();
    act:generateResponse(?actor,!ack,direct);
    if (op:equals(!response,yes)) {
        !activeGoals = act:getActiveGoals();
        if (op:isEmpty(!activeGoals)) {
            !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "detectObjects(?actor)");
            !goalID = act:submitGoal(!state);
            !goalStatus = act:joinOnGoal(!goalID);
        } else {
            act:reportCantVerifyBecauseActiveGoal();
            //not exiting so no more responses occur from handleITK
        }
    }
}

() = handleSeeITK(Symbol ?object) {
    effects : {
        success : knows(?actor,see(?object));
    }
    List !activeGoals;
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    Symbol !objectDescriptor;

    op:log(info,"handleSeeITK");

    !activeGoals = act:getActiveGoals();
    if (op:isEmpty(!activeGoals)) {
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "perceiveEntity(?actor,?object)");
        !goalID = act:submitGoal(!state);
        !goalStatus = act:joinOnGoal(!goalID);
        if (op:invokeMethod(!goalStatus, "isFailure")) {
           //TODO: get better output by asserting something?
          exit(FAIL);
        } else {
            !objectDescriptor = tsc:getDescriptorForID(?object);
            act:reportDetectedObject(!objectDescriptor);
        }
    } else {
        //!objectDescriptor = tsc:getDescriptorForID(?object);
        //act:reportCantVerifyBecauseActiveGoal(!objectDescriptor);
        act:reportCantVerifyBecauseActiveGoal();
        //not exiting so no more responses occur from handleITK
    }

    //act:perceiveEntity(?object);
}

() = handleObjectAtITK(Symbol ?object, Symbol ?pose) {
    effects : {
        success : knows(?actor,at(?object,?pose));
    }
    Symbol !objectPose;
    List !activeGoals;
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    Symbol !response;
    Predicate !ack = "ack(okay)";
    Symbol !objectDescriptor;
    Symbol !poseDescriptor;

    act:assertIsPose(?pose);
    act:assertIsPhysobj(?object);

    //Check belief first
    !objectPose = act:getObjectPose(?object);
    if (op:isNull(!objectPose)) {
        !objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    }

    !objectDescriptor = tsc:getDescriptorForID(?object);
    !poseDescriptor = tsc:getDescriptorForPoseID(?pose);

    //Report findings from belief
    if (op:equals(!objectPose,?pose)) {
        act:reportObjectBelievedToBeAtPose(!objectDescriptor,!poseDescriptor);
    } else {
        act:reportObjectNotBelievedToBeAtPose(!objectDescriptor,!poseDescriptor);
    }

    //Ask if user would like the system to go verify the belief (or lack thereof)
    !response = act:askToVerifyBelief();
    act:generateResponse(?actor,!ack,direct);
    if (op:equals(!response,yes)) {
        //Check if there is an active goal blocking verification
        !activeGoals = act:getActiveGoals();
        if (op:isEmpty(!activeGoals)) {
            //Verify through goToAndDetect action
            !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToAndDetect(?actor,?object,?pose)");
            !goalID = act:submitGoal(!state);
            !goalStatus = act:joinOnGoal(!goalID);
            if (op:invokeMethod(!goalStatus, "isFailure")) {
                //verification failed, report object is not at pose
                act:reportObjectNotAtPose(!objectDescriptor,!poseDescriptor);
            } else {
                //verification succeeded, report object is at pose
                act:reportDetectedObject(!objectDescriptor);
            }
        } else {
            //If there is already an active goal, can't go verify
            act:reportCantVerifyBecauseActiveGoal();
            //don't exit so no more responses
        }
    }
    //Otherwise do nothing, so no extra response
}

() = goToAndDetect(Symbol ?object, Symbol ?pose) {
    act:goToPose(?pose);
    act:perceiveEntity(?object);
}

() = handleObjectsAtITK(Symbol ?pose) {
    effects : {
        success : knows(?actor,objectsAt(?pose));
    }

    java.lang.Integer !num;
    Symbol !objectPose;
    List !activeGoals;
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    Symbol !response;
    Predicate !ack = "ack(okay)";
    Symbol !poseDescriptor;

    act:assertIsPose(?pose);

    !poseDescriptor = tsc:getDescriptorForPoseID(?pose);

    !num = act:getNumberOfObjectsAt(?pose);
    if (op:gt(!num,0)) {
        act:reportBelieveObjectsAtPose(!poseDescriptor);
    } else {
        act:reportDontBelieveObjectsAtPose(!poseDescriptor);
    }
    !response = act:askToVerifyBelief();
    act:generateResponse(?actor,!ack,direct);
    if (op:equals(!response,yes)) {
        //Check if there is an active goal blocking verification
        !activeGoals = act:getActiveGoals();
        if (op:isEmpty(!activeGoals)) {
            //Verify through goToAndDetect action
            !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToAndDetect(?actor,?pose)");
            !goalID = act:submitGoal(!state);
            !goalStatus = act:joinOnGoal(!goalID);
            if (op:invokeMethod(!goalStatus, "isFailure")) {
                //verification failed, report objects are not at pose
                act:reportObjectsNotAtPose(!poseDescriptor);
            } else {
                //verification succeeded, report objects are at pose
                act:reportDetectedObjects(!poseDescriptor);
                act:describeObjectsAt(?pose);
            }
        } else {
            //If there is already an active goal, can't go verify
            act:reportCantVerifyBecauseActiveGoal();
            //don't exit so no more responses
        }
    }
    //Otherwise do nothing, so no extra response
}

() = goToAndDetect(Symbol ?pose) {
    List !results;

    act:goToPose(?pose);
    !results = act:perceiveEntities();
    if (op:isEmpty(!results)) {
        exit(FAIL);
    }
}

/////// ITK ///////
() = describeObjectPose(Symbol ?object:physobj) {
    Symbol !objectPose;
    Symbol !poseDescriptor;
    Symbol !objectDescriptor;
    Symbol !holding;

    act:assertIsPhysobj(?object);

    !objectPose = act:getObjectPose(?object);
    !objectDescriptor = tsc:getDescriptorForID(?object);
    if (op:isNull(!objectPose)) {
        !holding = act:checkHolding(?object);
        if (op:equalsValue(!holding, true)) {
            act:reportHoldingObject(!objectDescriptor);
        } else {
            act:reportObjectLocationUnknown(!objectDescriptor);
        }
    } else {
        !poseDescriptor = tsc:getDescriptorForPoseID(!objectPose);
        act:reportObjectIsAtPose(!objectDescriptor,!poseDescriptor);
    }
}

//Describe known objects in the work area and their locations
() = describeObjectPoses() {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !pose;
    Symbol !object;
    Symbol !poseDescriptor;
    Symbol !objectDescriptor;
    String !poseDescriptorJapanese;
    String !objectDescriptorJapanese;
    java.lang.Boolean !knowObjectAt = false;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !objectVar;
    edu.tufts.hrilab.fol.Variable !poseVar;
    Symbol !holding;

    !objectVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !poseVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "Y");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,Y)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !object = op:get(!binding,!objectVar);
        !pose = op:get(!binding,!poseVar);
        !objectDescriptor = tsc:getDescriptorForID(!object);
        !poseDescriptor = tsc:getDescriptorForPoseID(!pose);
        if (~op:equals(!objectDescriptor,none)) {
            act:reportObjectIsAtPose(!objectDescriptor, !poseDescriptor);
            !knowObjectAt = op:newObject("java.lang.Boolean", true);
        }
    }
    !object = act:getHeldObject();
    !holding = act:checkHolding(!object);
    if (~op:isNull(!object) && op:equalsValue(!holding, true)) {
        !objectDescriptor = tsc:getDescriptorForID(!object);
        act:reportHoldingObject(!objectDescriptor);
        !knowObjectAt = op:newObject("java.lang.Boolean", true);
    }
    if (op:equals(!knowObjectAt,False)) {
        act:reportObjectLocationsAllUnknown();
    }
}

() = describeObjectsAt(Symbol ?pose:pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !object;
    Symbol !objectDescriptor;
    Symbol !poseDescriptor;
    java.lang.Boolean !knowObjectAt = false;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !objectVar;

    act:assertIsPose(?pose);

    !objectVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,?pose)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !object = op:get(!binding,!objectVar);
        !objectDescriptor = tsc:getDescriptorForID(!object);
        if (~op:equals(!objectDescriptor,none)) {
            act:reportObjectExists(!objectDescriptor);
            !knowObjectAt = op:newObject("java.lang.Boolean", true);
        }
    }
    if (op:equals(!knowObjectAt,False)) {
        !poseDescriptor = tsc:getDescriptorForPoseID(?pose);
        act:reportNoObjectsAtPose(!poseDescriptor);
    }
}

() = describeNumberOfObjectsAt(Symbol ?pose) {
    java.lang.Integer !numObjects;
    Symbol !poseDescriptor;

    act:assertIsPose(?pose);

    !numObjects = act:getNumberOfObjectsAt(?pose);
    !poseDescriptor = tsc:getDescriptorForPoseID(?pose);
    act:reportNumberOfObjectsAt(!numObjects,!poseDescriptor);
}

() = describeDetectableObjectNames() {
    List !objectList;

    !objectList = tsc:getEnglishObjectDescriptors();
    act:reportKnownObjects(!objectList);
}

() = describeKnownPoseNames() {
    List !poseList;

    !poseList = tsc:getEnglishPoseDescriptors();
    act:reportKnownPoses(!poseList);
}

() = describeCurrentGoal["describes ?actor's current active task(s)"](){

    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;
    java.lang.String !status;
    Predicate !goalPred;

    !goalPreds = act:getSystemGoalsPredicates();
    !resultsSize = op:size(!goalPreds);
    if(op:gt(!resultsSize,0)){
        foreach(!goalPred: !goalPreds){
            !status= op:getName(!goalPred);
            !goalPred= op:getArg(!goalPred,0);
            act:reportGoalStatus(!status);
            act:generateResponse(!goalPred);
        }
    } else {
        act:reportNoCurrentGoal();
    }
}

() = describePendingGoals["describes ?actor's pending goals task"](){
    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;

    !goalPreds = act:getPendingGoals();
    !resultsSize = op:size(!goalPreds);

    if(op:gt(!resultsSize,0)){
       foreach(!goalPred: !goalPreds){
           act:generateResponse(!goalPred);
       }
    } else{
        act:reportNoPendingGoal();
    }
}

() = describeCurrentPose() {
    Symbol !armPose;
    Symbol !descriptor;

    !armPose = act:getCurrentLocation();
    !descriptor = tsc:getDescriptorForPoseID(!armPose);

    act:reportCurrentPose(!descriptor);
}