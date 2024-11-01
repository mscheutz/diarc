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
    op:*(!dist,-1);
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
    conditions : {
        pre : free(?actor);
        pre : at(?physobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?pose));
        nonperf : not(at(?actor,?pose));
        nonperf : unknownlocation(?actor);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";

    act:rotateToEE(!gripper);
    tsc:openGripper();
    tsc:moveAndOrientToCognexTarget(?physobj);
    act:moveToObjectHeight(?physobj,?pose);
    tsc:closeGripper();
    act:moveToCameraHeight(?physobj,?pose);
    act:rotateToEE(!default);
    act:goToCameraPose(?pose);
    op:log(info, "[grabPerceivedEntity] ?physobj complete");
}

/// actions used in planning

() = grab["?actor grabs ?physobj"](Symbol ?physobj:physobj, Symbol ?pose:pose) {

    conditions : {
        pre : free(?actor);
        pre : at(?physobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?pose));
        nonperf : not(at(?actor,?pose));
        nonperf : unknownlocation(?actor);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";

    op:log(info, "[grab] ?physobj");
    act:goToCameraPose(?pose);
    act:perceiveEntity(?physobj);

    act:rotateToEE(!gripper);
    tsc:openGripper();
    tsc:moveAndOrientToCognexTarget(?physobj);
    act:moveToObjectHeight(?physobj,?pose);
    tsc:closeGripper();
    act:moveToCameraHeight(?physobj,?pose);
    act:rotateToEE(!default);
    act:goToCameraPose(?pose);
    op:log(info, "[grab] ?physobj complete");
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

    !result = tsc:moveToOpenLocation(!dim, "blocksDet"); //todo: THIS STRING SHOULDN'T BE HARDCODED
    act:rotateToEE(!gripper);
    act:moveToObjectHeight(?physobj,?pose);
    tsc:openGripper();
    act:moveToCameraHeight(?physobj,?pose);
    act:rotateToEE(!default);
    act:goToCameraPose(?pose);
    op:log(info, "[putdown] ?physobj + ?pose complete");
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

() = gotocamerapose["moves to ?pose1, from ?pose2"](Symbol ?pose1:pose, Symbol ?pose2:pose) {
    conditions : {
        pre : at(?actor, ?pose2);
    }
    effects : {
        success : not(at(?actor, ?pose2));
        success : at(?actor,?pose1);
        nonperf : not(at(?actor,?pose1));
        nonperf : not(at(?actor,?pose2));
        nonperf : unknownlocation(?actor);
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
    op:log(debug, "Finished going to pose ?pose1");
}

