import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;

//one-off actions. simpler signatures than those used in planning.
() = openGripper() {
    effects : {
        success : gripperOpen(?actor);
    }
    act:openGripperRapid();
}

() = closeGripper() {
    effects : {
        success : not(gripperOpen(?actor));
    }
    act:closeGripperRapid();
}

() = pickUp(Symbol ?item:physobj) {
    Predicate !queryPred;
    Symbol !area;
    Symbol !location;
    Symbol !safeArea;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(?actor, X)");
    !area = act:getBinding(!queryPred);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(?actor, X)");
    !location = act:getBinding(!queryPred);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "safe(?actor, X)");
    !safeArea = act:getBinding(!queryPred);

    act:pickup(?item, !area, !location, !safeArea);

}

() = putDown(Symbol ?item:physobj) {
    Predicate !queryPred;
    Symbol !area;
    Symbol !location;
    Symbol !safeArea;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(?actor, X)");
    !area = act:getBinding(!queryPred);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(?actor, X)");
    !location = act:getBinding(!queryPred);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "safe(?actor, X)");
    !safeArea = act:getBinding(!queryPred);

    act:putdown(?item, !area, !location, !safeArea);

}

() = goTo(Symbol ?destination:area) {

    Predicate !queryPred;

    Symbol !current;
    Symbol !location;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(?actor, X)");
    op:log(debug, "[gotoarea] queryPred: !queryPred");
    !current = act:getBinding(!queryPred);
    op:log(debug, "[gotoarea] current: !current");

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(?actor, X)");
    op:log(debug, "[gotoarea] queryPred: !queryPred");
    !location = act:getBinding(!queryPred);
    op:log(debug, "[gotoarea] location: !location");

    act:gotoarea(!current, ?destination, !location);
}


//actions used by the planner
() = pickup["picks up an ingredient and moves to a safe area"](Symbol ?item:physobj, Symbol ?area:area, Symbol ?location:location, Symbol ?safeArea:area) {

    Predicate !queryPred;
    Symbol !pose;

    conditions : {
        pre : gripperOpen(?actor);
        pre : free(?actor);
        pre : agentAtArea(?actor, ?area);
        pre : itemAt(?item, ?area);
        pre : agentAtLocation(?actor, ?location);
        pre : beenperceived(?item);
        pre : safe(?actor, ?safeArea);
    }
    effects : {
        success : not(gripperOpen(?actor));
        success : not(free(?actor));
        success : holding(?actor, ?item);
        success : not(itemAt(?item, ?area));
        success : agentAtArea(?actor, ?safeArea);
        success : not(agentAtArea(?actor, ?area));
        success : not(occupied(?area));
    }

    op:log(info, "[pickup] perceiving ?item from area ?area.");
    act:perceiveEntityFromSymbol(?item);
    op:log(info, "[pickup] picking up ?item");
    try {
        act:pickupItem(?item);
    } catch(FAIL_POSTCONDITIONS, !e) {
    }

    op:log(debug, "[pickup] ?actor finishing up by going to final position at ?safeArea from original area ?area");
    act:gotosafepose();
}


() = putdown["puts an ingredient down at the given area and moves to a safe area"](Symbol ?item:physobj, Symbol ?area:area, Symbol ?location:location, Symbol ?safeArea:area) {
    Predicate !queryPred;
    Symbol !pose;

    conditions : {
        pre : not(gripperOpen(?actor));
        pre : holding(?actor, ?item);
        pre : agentAtArea(?actor, ?area);
        pre : agentAtLocation(?actor, ?location);
        pre : reachable(?actor, ?area, ?location);
        pre : safe(?actor, ?safeArea);
        pre : not(safe(?actor, ?area));
        pre : not(occupied(?area));
    }
    effects : {
        success : gripperOpen(?actor);
        success : itemAt(?item, ?area);
        success : not(holding(?actor, ?item));
        success : free(?actor);
        success : not(agentAtArea(?actor, ?area));
        success : agentAtArea(?actor, ?safeArea);
        success : occupied(?area);
    }

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "areaPose(?actor, X, ?area)");
    !pose = act:getBinding(!queryPred);

    try {
        act:putDownItem(?item, !pose);
    } catch(FAIL_POSTCONDITIONS, !e) {
        act:askSafeToProceed("putdown");
    }

    op:log(debug, "[putdown] ?actor finishing up by going to final position at ?safeArea from original area ?area");
    act:gotosafepose();
}


() = gotoarea["go to a specified area"](Symbol ?current:area, Symbol ?destination:area, Symbol ?location:location) {
    Predicate !queryPred;
    Symbol !pose;

    conditions : {
        pre : agentAtArea(?actor, ?current);
        pre : agentAtLocation(?actor, ?location);
        pre : reachable(?actor, ?destination, ?location);
    }
    effects : {
        success : not(agentAtArea(?actor, ?current));
        success : agentAtArea(?actor, ?destination);
    }

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "poseAbove(?actor, X, ?destination)");
    op:log(debug, "[gotoarea] pose above query !queryPred");
    !pose = act:getBinding(!queryPred);
    op:log(debug, "[gotoarea] pose above pose !pose");
    try {
        act:goToCameraPose(!pose);
    } catch(FAIL_POSTCONDITIONS, !e) {
        act:askSafeToProceed("go to pose");
    }
}

() = putingredient["stacks ingredient on another ingredient and un-binds that ingredient for all manipulation actions"](Symbol ?item:physobj, Symbol ?destination:physobj, Symbol ?location:location, Symbol ?area:area) {
    Predicate !queryPred;
    Symbol !pose;

    conditions : {
        pre : holding(?actor, ?item);
        pre : agentAtArea(?actor, ?area);
        pre : itemAt(?destination, ?area);
    }
    effects : {
        success : gripperOpen(?actor);
        success : itemOn(?item, ?destination);
        success : not(holding(?actor, ?item));
        success : free(?actor);
        success : not(beenperceived(?item));
    }

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "areaPose(?actor, X, ?area)");
    !pose = act:getBinding(!queryPred);

    try {
        act:putDownItem(?item, !pose);
    } catch(FAIL_POSTCONDITIONS, !e) {
        act:askSafeToProceed("put down");
    }
    act:gotosafepose();
}

//goal wrapper actions. submitted as a state-based goal, but we need actions for action learning
() = getTo(Symbol ?item:physobj, Symbol ?destination:area) {
    goal:itemAt(?item, ?destination);
}

() = getOn(Symbol ?item:physobj, Symbol ?destination:physobj) {
    goal:itemOn(?item, ?destination);
}

//utility actions
() =  askSafeToProceed(java.lang.String ?failedAction) {
    java.util.Map !bindings;
    Predicate !proceedPred;
    java.lang.String !proceedString;
    Variable !x = "X";

    !bindings = act:askQuestionFromString(?actor,"A motion error has occurred in ?failedAction, is it safe to proceed?", safeToProceed(X));
    !proceedPred = op:get(!bindings, !x);
    !proceedString = op:getName(!proceedPred);
    if (~op:equals(!proceedString,true)) {
        exit(FAIL, motionFailure(?actor));
    }
    act:generateResponseFromString("okay");
}

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

() = gotosafepose() {
    Predicate !queryPred;
    Symbol !area;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "safe(?actor, X)");
    !area = act:getBinding(!queryPred);

    try {
        act:goTo(!area);
    } catch(FAIL_POSTCONDITIONS, !e) {
        act:askSafeToProceed("go to pose");
    }
}
