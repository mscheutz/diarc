import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.Map;

() = pickUp(Symbol ?item:physobj) {
    Predicate !queryPred;
    Symbol !pose;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");
    !pose = act:getBinding(!queryPred);

    act:pickup(?item, !pose);

}

() = putDown(Symbol ?item:physobj) {
    Predicate !queryPred;
    Symbol !pose;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");
    !pose = act:getBinding(!queryPred);

    act:putdown(?item, !pose);

}

() = pickup["?actor grabs ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {

    conditions : {
        pre : free(?actor);
        pre : at(?physobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?pose));
        success : not(occupied(?pose));
        nonperf : not(at(?actor,?pose));
        nonperf : unknownlocation(?actor);
    }

    edu.tufts.hrilab.fol.Symbol !default = "default";
    edu.tufts.hrilab.fol.Symbol !gripper = "gripper";

    op:log(info, "[grab] ?physobj");
    act:goTo(?pose);
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

() = openGripper() {
    tsc:openGripper();
}

() = closeGripper() {
    tsc:closeGripper();
}

() = putdown["?actor releases ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    conditions : {
        pre : holding(?actor, ?physobj);
        pre : at(?actor,?pose);
        pre : not(occupied(?pose));
    }
    effects : {
        success : not(holding(?actor, ?physobj));
        success : free(?actor);
        success : at(?physobj,?pose);
        success : occupied(?pose);
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

() = putingredient["stacks ingredient on another ingredient and un-binds that ingredient for all manipulation actions"](Symbol ?item:physobj, Symbol ?destination:physobj, Symbol ?pose:pose) {

    conditions : {
        pre : holding(?actor, ?item);
        pre : at(?actor, ?pose);
        pre : at(?destination, ?pose);
    }
    effects : {
        success : itemOn(?item, ?destination);
        success : not(holding(?actor, ?item));
        success : free(?actor);
        success : not(beenperceived(?item));
        success : not(occupied(?pose)); //todo: this is a hack
    }

    act:putdown(?item, ?pose);
}


() = goTo["goes to pose at camera height"](edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Symbol !currPose = "current";
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "[goTo] bindings: !bindings");
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (!currPose) =op:get(!elem,!bindingVar);
    }

    op:log(info, "[goTo] going to pose ?pose");
    act:gotocamerapose(?pose, !currPose);
}


() = gotocamerapose["moves to ?pose1, from ?pose2"](edu.tufts.hrilab.fol.Symbol ?pose1:pose, edu.tufts.hrilab.fol.Symbol ?pose2:pose) {
    conditions : {
        pre : at(?actor, ?pose2);
        pre : reachable(?actor, ?pose2);
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


() = getTo(Symbol ?item:physobj, Symbol ?destination:pose) {
    goal:at(?item, ?destination);
}

() = getOn(Symbol ?item:physobj, Symbol ?destination:physobj) {
    goal:itemOn(?item, ?destination);
}

() = perceiveitem["top level perception action which binds the refId based on the availability of that type of object at the ?actor's location"](Symbol ?refId:physobj, Symbol ?itemType:property, Symbol ?pose:pose) {

    conditions : {
        pre : at(?actor, ?pose);
        pre : property_of(?refId, ?itemType);
        pre : observableAt(?itemType,?pose);
        pre : not(beenperceived(?refId));
    }
    effects : {
        success : beenperceived(?refId);
        success : at(?refId,?pose);
    }

    act:perceiveEntity(?refId);
}

() = defineIngredient["defines a new ingredient and asks where it is located"](edu.tufts.hrilab.fol.Symbol ?descriptor){

    Map !bindings;
    Variable !x = "X";
    Symbol !pose;
    Symbol !job;

    !bindings = act:askQuestionFromString(?actor,"それはどこにありますか？", pose(X));
    //!bindings = act:askQuestionFromString(?actor,"Where is it located?", pose(X));
    !pose= op:get(!bindings, !x);

    !bindings = act:askQuestionFromString(?actor,"OK、検出するためにどのビジョンジョブを使用しますか？", job(X));
    //!bindings = act:askQuestionFromString(?actor,"Okay, what vision job is used to detect it?", job(X));
    !job = op:get(!bindings, !x);

    act:defineIngredientHelper(?descriptor,!pose,!job);

    act:generateResponseFromString("OK、鶏肉が何か分かりました。");
    //act:generateResponseFromString("Okay, I know what ?descriptor is");
}

() = defineIngredientHelper["helper method to standardize interaction based item definition with asl based definition"](Symbol ?descriptor, Symbol ?pose, Symbol ?job){

    Predicate !tmp;

    ?descriptor=tsc:addDetectionType(?descriptor,?job);

    !tmp= op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(?descriptor,0.0,0.0)");
    act:assertBelief(!tmp);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "observableAt(?descriptor,?pose)");
    act:assertBelief(!tmp);
}

//todo: consolidate this.
(Symbol ?result) = getBinding(Predicate ?queryPred) {
    Variable !queryKey = X;
    java.util.List !bindings;
    java.util.Map !binding;

    !bindings = act:queryBelief(?queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("error", "[getBinding] query ?queryPred returned no results");
    } else {
        !binding = op:get(!bindings, 0);
        ?result= op:get(!binding, !queryKey);
        op:log("debug", "[getBinding] query ?queryPred result: ?result");
    }
}

() = getingredient(Symbol ?actor:mobileManipulator, Symbol ?refId:physobj, Symbol ?itemType:property,
                   Symbol ?perceptionArea:pose, Symbol ?putdownArea:pose) {

    conditions : {
        pre : property_of(?refId, ?itemType);
        pre : observableAt(?itemType,?perceptionArea);
        pre : not(beenperceived(?refId));
        pre : free(?actor);
        pre : reachable(?actor, ?perceptionArea);
        pre : reachable(?actor, ?putdownArea);
    }
    effects : {
        success : beenperceived(?refId);
        success : at(?refId, ?putdownArea);
    }

    java.util.Map !bindings;
    Variable !x = "X";
    Predicate !response;

    java.lang.String !questionString;

    !questionString = act:getQuestionFromRefs(?itemType, ?perceptionArea, ?putdownArea);

    //!bindings = act:askQuestionFromString(?actor, !questionString, got(X));
    //TODO: hardcoded to pantry/corn/cooktop for this demo - don't know how to generally do ref replacement in Japanese but theoretically could
    //      in the method above
    !bindings = act:askQuestionFromString(?actor, "鶏肉を食料庫からクックトップまで持ってきてくれませんか？", got(X));

    !response = op:get(!bindings, !x);
}
