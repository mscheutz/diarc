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
    conditions : {
        pre : free(?actor);
    }
    tsc:openGripper();
}

() = closeGripper() {
    tsc:closeGripper();
}

(List ?bindings) = perceiveEntities() {
    //runs a job that looks for all items known to the system and returns the set of (orderd) results. then binds the associated refrerences.

    Symbol !location;
    Symbol !objectLocation;
    Predicate !fact;

    java.lang.Double !objectHeightM = 0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED
    java.lang.Double !negObjectHeightM = -0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED

    tsc:moveZRelative(!objectHeightM);
    ?bindings = tsc:perceiveAllEntitiesInView("blocksDet");
    tsc:moveZRelative(!negObjectHeightM);
    !location = act:getCurrentLocation();
    foreach (!result : ?bindings) {
        !objectLocation = act:getObjectPose(!result);
        if (~op:equals(!objectLocation, !location)) {
            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(holding(?actor,!result))");
            act:retractBelief(!fact);
            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result,!objectLocation)");
            act:retractBelief(!fact);
            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result,!location)");
            act:assertBelief(!fact);
        }
    }
}

//todo: (pete) allows the LLM target for semantics to be more sane than "getOn". should be true everywhere.
() = getObjectTo(Symbol ?object, Symbol ?destination) {
    Symbol !objectPose;
    Predicate !failCond;
    Predicate !objectAtPred;
    Symbol !descriptor;
    Symbol !holding;

    //TODO: assert ?object is actually a physobj?
    !objectPose = act:getObjectPose(?object);
    !holding = act:checkHolding(?object);
    if (op:equalsValue(!holding, false)) {
        if (op:isNull(!objectPose)) {
            !descriptor = tsc:getDescriptorForID(?object);
            !objectPose = act:askForObjectPose(!descriptor);
            if (op:equals(!objectPose,none)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "know(?actor,theLocationOf,?object)");
                exit(FAIL, !failCond);
            } else {
                !objectAtPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,!objectPose)");
                act:assertBelief(!objectAtPred);
            }
        }
    }

    //TODO: get rid of this doubly nested action wrapping of goalspecs. Interacts badly with recovery - need to
    //      specifically exclude the parent from any goal recovery policies
    //  ^Is this actually just due to bad implementation for goal recovery w/ goalspecs in general?
    act:getOn(?object, ?destination);
}

() = pickUp() {
    conditions : {
        pre : free(?actor);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";
    List !objectList;
    Symbol !object;
    Symbol !location;
    Integer !resultsSize;
    Predicate !failCond;

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
        ////TODO: make sure response matches one of the detected objects
        if (~op:equals(!object, none)) {
            !location = act:getCurrentLocation();
            act:grabPerceivedEntity(!object,!location);
        } else {
            act:reportNotPickingUpObject();
        }
    }
}

() = pickUp(Symbol ?object) {
    conditions : {
        pre : free(?actor);
    }

    op:log(info, "[pickUp] ?object");

    Symbol !objectPose;
    Symbol !armPose;
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
    conditions : {
        pre : not(free(?actor));
    }

    act:putDown();
}

() = putDown() {
    conditions : {
        pre : not(free(?actor));
    }
    Symbol !location;
    Symbol !object;

    op:log(info, "[putDown]");

    !location = act:getCurrentLocation();
    !object = act:getHeldObject();
    act:putdown(!object,!location);
}

() = moveObjectTo(Symbol ?object:physobj, Symbol ?start:pose, Symbol ?destination:pose) {
    act:goToPose(?start);
    act:pickUp(?object);
    act:goToPose(?destination);
    act:putDown();
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
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "know(?actor,theLocationOf,?objectOne)");
        exit(FAIL, !failCond);
    } elseif (op:isNull(!objectTwoPose)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "know(?actor,theLocationOf,?objectTwo)");
        exit(FAIL, !failCond);
    }

    if (~op:isNull(!heldObject)) {
        if (op:equals(?objectOne,!heldObject)) {
            act:swapHeldObject(!heldObject,?objectTwo,!objectTwoPose);
            return;
        } elseif (op:equals(?objectTwo,!heldObject)) {
            act:swapHeldObject(!heldObject,?objectOne,!objectOnePose);
            return;
        } else {
            //TODO: ideally not to one of the swap poses
            //Move held object to open location
            !openPose = act:getPoseWithSpace();
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
        !openPose = act:getPoseWithSpace();
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

    !numObjects = act:getNumberOfObjectsAt(!objectToSwapPose);
    if (op:gt(!numObjects,1)) {
        //Move held object to open location
        !openPose = act:getPoseWithSpace();
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapObjects)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving held object to open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, !openPose)");
        goal:!goalPred;
        act:putDown();

        //Move object to swap temporarily to an open location
        !openPose = act:getPoseWithSpace();
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
        actionStatuses : {FAIL_PRECONDITIONS}
        failureReasons : {has(theLocationBelowMe,space)}
        excludedGoals : {putDown(X), putDown(X,Y)}
    }
    recovery : {
        actionStatuses : {FAIL_PRECONDITIONS}
        failureReasons : {has(theLocationBelowMe,space)}
        goals : {getObjectTo(X,Y,Z)}
    }
    Symbol !response;
    Symbol !otherObject;
    String !otherObjectName;
    Symbol !object;

    op:log("debug","[putDownAtFullLocationRecovery");
    op:log("debug","[putDownAtFullLocationRecovery] ?failedActionPredicates");
    op:log("debug","[putDownAtFullLocationRecovery] ?failureReasons");
    op:log("debug","[putDownAtFullLocationRecovery] ?failedGoal");

    !object = act:getHeldObject();
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
            act:swapObjects(!object,!otherObject);
        }
    } else {
        act:reportNotSwappingObjects();
        exit(FAIL_RECOVERY);
    }
}

() = moveObjectTo(Symbol ?object:physobj, Symbol ?destination:pose) {
    conditions : {
        pre : holding(?actor,?object);
    }
    act:goToPose(?destination);
    act:putDown();
}

() = moveObjectTo(Symbol ?destination:pose) {
    conditions : {
        pre : not(free(?actor));
    }
    act:goToPose(?destination);
    act:putDown();
}

() = detectObject(Symbol ?object) {
    effects : {
        success : knows(?actor,see(?actor,?object));
    }
    op:log(info,"detectObject");
    act:perceiveEntity(?object);
}

//TODO
() = resetDomain() {
    conditions : {
        pre : free(?actor); //TODO can have recovery asking user where to put object down
    }
}

/////// ITK ///////
() = describeObjectPose(Symbol ?object:physobj) {
    Symbol !objectPose;
    Symbol !poseDescriptor;
    Symbol !objectDescriptor;
    Symbol !holding;

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
() = describeKnownObjects() {
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

    !numObjects = act:getNumberOfObjectsAt(?pose);
    !poseDescriptor = tsc:getDescriptorForPoseID(?pose);
    act:reportNumberOfObjectsAt(!numObjects,!poseDescriptor);
}

////Could do same for locations if we want to
//() = describeObjectTypes() {
//    //TODO
//    //?objectは感知できます == (I know how to detect ?object)
//}

() = goToAndDetect(Symbol ?object, Symbol ?pose) {
    effects : {
        success : knows(?actor,at(?object,?pose));
    }

    act:goToPose(?pose);
    act:perceiveEntity(?object);
}

() = goToAndDetect(Symbol ?pose) {
    effects : {
        success : knows(?actor,objectsAt(?pose));
    }
    List !results;

    act:goToPose(?pose);
    !results = act:perceiveEntities();
    if (op:isEmpty(!results)) {
        exit(FAIL);
    }
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