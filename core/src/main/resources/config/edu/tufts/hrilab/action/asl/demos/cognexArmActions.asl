import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.HashMap;
import java.util.List;

/// utility actions used by configs with a cognex mounted on an arm
() = moveToCameraHeight["moves up by a camera z height as defined by an inline constant"](Symbol ?physobj,Symbol ?pose) {
    java.lang.Double !dist;
    Symbol !descriptor;
    Predicate !queryPred;
    List !bindings;
    java.util.HashMap !binding;
    Variable !bindingVar;
    Symbol !objectHeightSymbol;
    java.lang.Double !objectHeightM;
    java.lang.Double !cameraHeight;
    java.lang.Double !cameraHeightM;

    !descriptor = tsc:getDescriptorForID(?physobj);
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","objectDefinition(!descriptor,A,B)");
    !bindings = act:queryBelief(!queryPred);
    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","A");
    if (~op:isEmpty(!bindings)) {
        (!binding) = op:get(!bindings, 0);
        (!objectHeightSymbol) = op:get(!binding,!bindingVar);
    } else {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,objectDefinition(!descriptor)))");
        exit(FAIL_RETURNVALUE,!failCond);
    }
    !objectHeightM = op:/(!objectHeightSymbol,1);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,!bindingVar)");
    !bindings = act:queryBelief(!queryPred);
    !binding = op:get(!bindings, 0);
    !cameraHeight = op:get(!binding, !bindingVar);
    !cameraHeightM = op:/(!cameraHeight,1000);

    !dist = op:-(!cameraHeightM,!objectHeightM);
    tsc:moveZRelative(!dist);
}

() = moveToObjectHeight["moves down by a camera z height as defined by an inline constant"](Symbol ?physobj, Symbol ?pose) {
    java.lang.Double !dist;
    Symbol !descriptor;
    Predicate !queryPred;
    List !bindings;
    java.util.HashMap !binding;
    Variable !bindingVar;
    Symbol !objectHeightSymbol;
    java.lang.Double !objectHeightM;
    java.lang.Double !cameraHeight;
    java.lang.Double !cameraHeightM;
    java.lang.Double !graspDepth;
    java.lang.Double !graspDepthM;

    !descriptor = tsc:getDescriptorForID(?physobj);
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","objectDefinition(!descriptor,A,B)");
    !bindings = act:queryBelief(!queryPred);
    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","A");
    if (~op:isEmpty(!bindings)) {
        (!binding) = op:get(!bindings, 0);
        (!objectHeightSymbol) = op:get(!binding,!bindingVar);
    } else {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,objectDefinition(!descriptor)))");
        exit(FAIL_RETURNVALUE,!failCond);
    }
    !objectHeightM = op:/(!objectHeightSymbol,1);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,!bindingVar)");
    !bindings = act:queryBelief(!queryPred);
    !binding = op:get(!bindings, 0);
    !cameraHeight = op:get(!binding, !bindingVar);
    !cameraHeightM = op:/(!cameraHeight,1000);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "graspDepth(!descriptor,!bindingVar)");
    !bindings = act:queryBelief(!queryPred);
    !binding = op:get(!bindings, 0);
    !graspDepthM = op:get(!binding, !bindingVar);

    !dist = op:-(!cameraHeightM,!objectHeightM);
    !dist = op:+(!dist,!graspDepthM);
    !dist = op:*(!dist,-1);
    tsc:moveZRelative(!dist);
}

() = getOn["gets ?object on to the surface beneath ?destination"](Symbol ?object, Symbol ?destination) {
   Predicate !goalPred;

   (!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,?destination)");
   goal:!goalPred;
}

() = goToPose(Symbol ?destination:pose) {
   Predicate !goalPred;
   Symbol !currentPose;

   !currentPose = act:getCurrentLocation();
   act:gotocamerapose(?destination,!currentPose);
   //(!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,?destination)");
   //goal:!goalPred;
}

() = grabPerceivedEntity(Symbol ?physobj, Symbol ?pose) {
    onInterrupt :{
        cancel: act:grabOnCancel(!tracker,?physobj,?pose);
    }
    conditions : {
        pre : free(?actor);
        pre : at(?physobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?pose));
        //nonperf : not(at(?actor,?pose));
        //nonperf : unknownlocation(?actor);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";
    java.lang.Integer !tracker = 0;

    noInterrupt {
        act:rotateToEE(!gripper);
        !tracker = op:++(!tracker); //1
    }
    tsc:openGripper();
    tsc:moveAndOrientToCognexTarget(?physobj);
    noInterrupt {
        act:moveToObjectHeight(?physobj,?pose);
        !tracker = op:++(!tracker); //2
    }

    noInterrupt {
        tsc:closeGripper();
        act:moveToCameraHeight(?physobj,?pose);
        act:rotateToEE(!default);
        act:goToCameraPose(?pose);
        op:log(info, "[grabPerceivedEntity] ?physobj complete");
    }
}

() = grabOnCancel(java.lang.Integer ?tracker, Symbol ?physobj, Symbol ?pose) {
    //don't need any effects because we revert to the starting state here

    Symbol !default = "default";
    Symbol !gripper = "gripper";
    Predicate !responsePredicate;

    //If we entered this script, we are backtracking to a sane pose rather than going forward through the script to do so.
    op:log(info,"[grabOnCancel] in method");

    //Only performed perception, nothing needs to be done
    if (op:equals(?tracker,0)) {
        return;
    }

    //TODO: can replace with hardcoded message if having an actual translation
    !responsePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "returningToASafeStateBeforeCancelingMyCurrentTask()");
    act:generateResponse(!responsePredicate);

    //Have lowered gripper to the object - raise back to camera height before switching tcp and pose
    if (op:equals(?tracker,2)) {
        act:moveToCameraHeight(?physobj,?pose);
    }

    //Already messed with TCP and pose - revert TCP to default and return to a valid pose
    //if (op:lt(?tracker,3)) {
    act:rotateToEE(!default);
    act:goToCameraPose(?pose);
    //}

    //Else, we have progressed far enough to have grabbed the object. The action should have finished successfully
    //  and not been canceled due to a noninterrupt block around the remainder of steps
}

/// actions used in planning

() = grab["?actor grabs ?physobj"](Symbol ?physobj:physobj, Symbol ?pose:pose) {
    onInterrupt :{
        cancel: act:grabOnCancel(!tracker,?physobj,?pose);
    }
    conditions : {
        pre : free(?actor);
        pre : at(?physobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?pose));
        //nonperf : not(at(?actor,?pose));
        //nonperf : unknownlocation(?actor);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";
    java.lang.Integer !tracker = 0;

    op:log(info, "[grab] ?physobj");
    act:goToCameraPose(?pose);
    act:perceiveEntity(?physobj);

    noInterrupt {
        act:rotateToEE(!gripper);
        !tracker = op:++(!tracker); //1
    }
    tsc:openGripper();
    tsc:moveAndOrientToCognexTarget(?physobj);
    noInterrupt {
        act:moveToObjectHeight(?physobj,?pose);
        !tracker = op:++(!tracker); //2
    }

    noInterrupt {
        tsc:closeGripper();
        act:moveToCameraHeight(?physobj,?pose);
        act:rotateToEE(!default);
        act:goToCameraPose(?pose);
        op:log(info, "[grab] ?physobj complete");
    }
}

() = putdown["?actor releases ?physobj"](Symbol ?physobj:physobj, Symbol ?pose:pose) {
    conditions : {
        pre : holding(?actor, ?physobj);
        pre : at(?actor,?pose);
    }
    effects : {
        success : not(holding(?actor, ?physobj));
        success : free(?actor);
        success : at(?physobj,?pose);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";
    Predicate !failPred;
    java.lang.Boolean !result;
    java.util.List !results;
    java.lang.Integer !size;
    Predicate !failCond;
    edu.tufts.hrilab.action.justification.Justification !failJustification;
    Predicate !thisStep;

    Symbol !descriptor;
    Predicate !queryPred;
    List !bindings;
    java.util.HashMap !binding;
    Variable !bindingVar;
    Symbol !objectHeightSymbol;
    java.lang.Double !objectHeightM;
    Symbol !dimSymbol;
    java.lang.Double !dim;
    java.lang.Integer !tracker = 0;

    op:log(info, "[putdown] ?physobj + ?pose");

    !results = act:perceiveEntities();
    if (~op:isEmpty(!results)) {
        !size = op:invokeMethod(!results, "size");
        if (op:gt(!size,1)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "has(theLocationBelowMe,space)");
            !failJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
            !thisStep = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "putdown(?actor,?physobj,?pose)");
            op:invokeMethod(!failJustification, "setStep", !thisStep);
            exit(FAIL_PRECONDITIONS, !failJustification);
        }
    }

    !descriptor = tsc:getDescriptorForID(?physobj);
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","objectDefinition(!descriptor,A,B)");
    !bindings = act:queryBelief(!queryPred);
    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","B");
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !dimSymbol = op:get(!binding,!bindingVar);
    } else {
         !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,objectDefinition(!descriptor)))");
         exit(FAIL_RETURNVALUE,!failCond);
     }
    !dim = op:/(!dimSymbol,1);
    !dim = op:*(!dim,1000);

    noInterrupt {
        !result = tsc:moveToOpenLocation(!dim, "blocksDet"); //todo: THIS STRING SHOULDN'T BE HARDCODED
        act:rotateToEE(!gripper);
        !tracker = op:++(!tracker); //1
    }
    noInterrupt {
        act:moveToObjectHeight(?physobj,?pose);
        !tracker = op:++(!tracker); //2
    }
    noInterrupt {
        tsc:openGripper();
        act:moveToCameraHeight(?physobj,?pose);
        act:rotateToEE(!default);
        act:goToCameraPose(?pose);
        op:log(info, "[putdown] ?physobj + ?pose complete");
    }
}

() = putdownOnCancel(java.lang.Integer ?tracker, Symbol ?physobj, Symbol ?pose) {
    //Don't need effects because the action is either backtracked to starting state or allowed to complete successfully and not enter here

    //No movement occurred yet, do nothing
    if (op:equals(?tacker,0)) {
        return;
    }

    //TODO: can replace with hardcoded message if having an actual translation
    !responsePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "returningToASafeStateBeforeCancelingMyCurrentTask()");
    act:generateResponse(!responsePredicate);

    //Lowered to table, but not released -> raise back up
    if (op:equals(?tracker,2)) {
        act:moveToCameraHeight(?physobj,?pose);
    }

    //Go back to default tcp and camera pose
    //if (op:lt(?tracker,3)) {
    act:rotateToEE(!default);
    act:goToCameraPose(?pose);
    //}

    //Else, we have progressed far enough to have released the object. The action should have finished successfully
    //  and not been canceled due to a noninterrupt block around the remainder of steps
}


() = goToCameraPose["goes to pose at camera height"](Symbol ?pose:pose) {
    Predicate !queryPred;
    java.util.List !bindings;

    java.util.HashMap !elem;
    Symbol !currPose = "current";
    Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "[goToCameraPose] bindings: !bindings");
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (!currPose) =op:get(!elem,!bindingVar);
    }

    act:gotocamerapose(?pose, !currPose);
}

//TODO: need different nonperf effects depending on whether the movement primitive is actually interruptible or not
() = gotocamerapose["moves to ?pose1, from ?pose2"](Symbol ?pose1:pose, Symbol ?pose2:pose) {
    conditions : {
        pre : at(?actor, ?pose2);
    }
    effects : {
        success : not(at(?actor, ?pose2));
        success : at(?actor,?pose1);
        //nonperf : not(at(?actor,?pose1));
        //nonperf : not(at(?actor,?pose2));
        //nonperf : unknownlocation(?actor);
    }

    Predicate !queryPred;
    java.util.List !bindings;
    java.util.HashMap !elem;
    Symbol !cameraHeight;
    Variable !bindingVar = "X";

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,!bindingVar)");
    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "[gotocamerapose] bindings: !bindings");
    !elem = op:get(!bindings, 0);
    op:log(debug, "[gotocamerapose] cameraHeight found for query !queryPred: !bindings");
    !cameraHeight = op:get(!elem, !bindingVar);
    act:goToPosePrimitive(?pose1, !cameraHeight);
    //Until nonInterruptible blocks are implemented, having this as the last call without nonperf effects is safe if
    //  the primitive is not actually interruptible. The very last call this grounds out to is a goTo tsc. If canceled
    //  at any point before this action is complete, we are still at the original pose. (we do not handle exceptions
    //  during primitive execution as of now)
}

