//actions used for planning in circuit breaker screwing demos

() = grab["?actor grabs ?cognexobj"](edu.tufts.hrilab.fol.Symbol ?cognexobj:cognexobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {

    conditions : {
        pre : free(?actor);
        pre : at(?cognexobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?cognexobj);
        success : not(free(?actor));
        success : not(at(?cognexobj,?pose));
        nonperf : not(at(?actor,?pose));
        nonperf : unknownlocation(?actor);
    }

    edu.tufts.hrilab.fol.Symbol !default = "default";
    edu.tufts.hrilab.fol.Symbol !gripper = "gripper";

    op:log(info, "[grab] ?cognexobj");
    act:goToCameraPose(?pose);
    act:perceiveEntity(?cognexobj);

    act:rotateToEE(!gripper);
    tsc:openGripper();
    tsc:moveAndOrientToCognexTarget(?cognexobj);
    act:moveToObjectHeight();
    tsc:closeGripper();
    act:moveToCameraHeight();
    act:rotateToEE(!default);
    op:log(info, "[grab] ?cognexobj complete");
}

() = putdown["?actor releases ?cognexobj"](edu.tufts.hrilab.fol.Symbol ?cognexobj:cognexobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    conditions : {
        pre : holding(?actor, ?cognexobj);
        pre : at(?actor,?pose);
    }
    effects : {
        success : not(holding(?actor, ?cognexobj));
        success : free(?actor);
        success : at(?cognexobj,?pose);
    }

    edu.tufts.hrilab.fol.Symbol !default = "default";
    edu.tufts.hrilab.fol.Symbol !gripper = "gripper";

    op:log(info, "[putdown] ?cognexobj + ?pose");

    act:rotateToEE(!gripper);
    act:moveToObjectHeight();
    tsc:openGripper();
    act:moveToCameraHeight();
    act:rotateToEE(!default);

    op:log(info, "[putdown] ?cognexobj + ?pose complete");
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
    act:goToPosePrimitive(?pose1, !cameraHeight);
    op:log(debug, "Finished going to pose ?pose1");
}

