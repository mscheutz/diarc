//actions used for planning in circuit breaker screwing demos

() = grab["?actor grabs ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {

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

    edu.tufts.hrilab.fol.Symbol !default = "default";
    edu.tufts.hrilab.fol.Symbol !gripper = "gripper";

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
    op:log(info, "[grab] ?physobj complete");
}

() = putdown["?actor releases ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    conditions : {
        pre : holding(?actor, ?physobj);
        pre : at(?actor,?pose);
    }
    effects : {
        success : not(holding(?actor, ?physobj));
        success : free(?actor);
        success : at(?physobj,?pose);
    }

    edu.tufts.hrilab.fol.Symbol !default = "default";
    edu.tufts.hrilab.fol.Symbol !gripper = "gripper";

    op:log(info, "[putdown] ?physobj + ?pose");

    act:rotateToEE(!gripper);
    act:moveToObjectHeight();
    tsc:openGripper();
    act:moveToCameraHeight();
    act:rotateToEE(!default);

    op:log(info, "[putdown] ?physobj + ?pose complete");
}


() = goToCameraPose["goes to pose at camera height"](edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Symbol !currPose = "current";
    edu.tufts.hrilab.fol.Variable !bindingVar;
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

() = gotocamerapose["moves to ?pose1, from ?pose2"](edu.tufts.hrilab.fol.Symbol ?pose1:pose, edu.tufts.hrilab.fol.Symbol ?pose2:pose) {
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

    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Symbol !cameraHeight;
    edu.tufts.hrilab.fol.Variable !bindingVar = "X";

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,!bindingVar)");
    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "[gotocamerapose] bindings: !bindings");
    !elem = op:get(!bindings, 0);
    op:log(debug, "[gotocamerapose] cameraHeight found for query !queryPred: !bindings");
    !cameraHeight = op:get(!elem, !bindingVar);
    act:goToPose(?pose1, !cameraHeight);
    op:log(debug, "Finished going to pose ?pose1");
}

