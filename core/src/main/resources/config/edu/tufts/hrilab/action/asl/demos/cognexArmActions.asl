import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;

/// utility actions used by configs with a cognex mounted on an arm
() = moveToCameraHeight["moves up by a camera z height as defined by an inline constant"]() {
    //Todo: Will/Mar: change these from hardcoded values and/or see if these need to be inverses or not
    java.lang.Double !dist = 0.15;
    tsc:moveZRelative(!dist);
}

() = moveToObjectHeight["moves down by a camera z height as defined by an inline constant"]() {
    //Todo: Will/Mar: change these from hardcoded values and/or see if these need to be inverses or not
    java.lang.Double !dist = -0.15;
    tsc:moveZRelative(!dist);
}

() = getOn["gets ?object on to the surface beneath ?destination"](Symbol ?object, Symbol ?destination) {
   Predicate !goalPred;

   (!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,?destination)");
   goal:!goalPred;
}

() = goToPose(Symbol ?destination:pose) {
   Predicate !goalPred;

   (!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,?destination)");
   goal:!goalPred;
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
    act:moveToObjectHeight();
    tsc:closeGripper();
    act:moveToCameraHeight();
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
    act:moveToObjectHeight();
    tsc:closeGripper();
    act:moveToCameraHeight();
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
    java.lang.Double !objectHeightM = 0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED
    java.lang.Double !negObjectHeightM = -0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED
    java.lang.Double !dim = 50;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED
    Predicate !failPred;
    java.lang.Boolean !result;

//    !failPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "full(?pose)");

    op:log(info, "[putdown] ?physobj + ?pose");

    tsc:moveZRelative(!objectHeightM);
    !result = tsc:moveToOpenLocation(!dim, "blocksDet"); //todo: THIS STRING SHOULDN'T BE HARDCODED
    op:log(debug, "[putdown] result of moveToOpenLocation: !result");
//    if (~op:equals(!result, true)) {
//        exit(FAIL, !failPred);
//    }
    tsc:moveZRelative(!negObjectHeightM);
    act:rotateToEE(!gripper);
    act:moveToObjectHeight();
    tsc:openGripper();
    act:moveToCameraHeight();
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

