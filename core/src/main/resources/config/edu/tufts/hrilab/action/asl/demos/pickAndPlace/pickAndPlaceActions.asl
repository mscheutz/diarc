import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

() = openGripper() {
    tsc:openGripper();
}

() = closeGripper() {
    tsc:openGripper();
}

//TODO: add failure case and recovery policy for block not being present in the work area
//todo: (pete) allows the LLM target for semantics to be more sane than "getOn". should be true everywhere.
() = getBlockTo(Symbol ?object, Symbol ?destination) {
    act:getOn(?object, ?destination);
}

(Symbol ?location) = getCurrentLocation() {
    Predicate !queryPred;
    java.util.List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (?location) =op:get(!elem,!bindingVar);
    }
}

(Symbol ?physobj) = getHeldObject() {
    Predicate !queryPred;
    java.util.List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (?physobj) =op:get(!elem,!bindingVar);
    }
}

() = pickUp() {
    conditions : {
        pre : free(?actor);
    }

    Symbol !default = "default";
    Symbol !gripper = "gripper";
    Symbol !object;
    Symbol !location;

    op:log(info, "[pickUp]");
    //TODO: Need implementation of this function (retval should be collection of objects)
    !object = act:perceiveEntities();

    //TODO: parse cognex results and split on case

    //No results
    //Predicate !failCond;
    //!failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "dont(?actor,see,anyBlocks,atMyLocation)");
    //exit(FAIL_RETURNVALUE,!failCond);

    ////Multiple results - ask which one to pick up
    //Map !bindings;
    //Variable !x = "X";
    ////TODO: add detected blocks in question
    //!bindings = act:askQuestionFromString("I see multiple blocks, which one do you want me to pick up?", blockType(X));
    //!object = op:get(!bindings, !x);
    ////TODO: make sure response matches one of the detected blocks

    ////Single result - pick up object
    !location = act:getCurrentLocation();
    act:grabPerceivedEntity(!object,!location);
}

//Placeholder
(java.util.List ?bindings) = perceiveEntities() {
//(Symbol ?bindings) = perceiveEntities() {
//    ?bindings = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "robotonephysobj_0:physobj");
//    act:perceiveEntity(?bindings);
}

() = pickUp(Symbol ?object) {
    conditions : {
        pre : free(?actor);
    }

    Symbol !location;

    op:log(info, "[pickUp] ?object");
    act:perceiveEntity(?object);
    !location = act:getCurrentLocation();

    act:grabPerceivedEntity(?object,!location);
}

() = putDown(Symbol ?block:physobj) {
    conditions : {
        pre : not(free(?actor));
    }
    op:log(info, "[putDown] ?block");

    act:putDown();
}

() = putDown() {
    conditions : {
        pre : not(free(?actor));
    }

    op:log(info, "[putDown]");

    java.util.List !results;
    java.lang.Integer !resultsSize;
    Predicate !failConds;
    Symbol !location;
    Symbol !object;

    //TODO: can just push this down into the putdown primitive and have recovery policy only apply to this script
    !results = act:perceiveEntities();
    if (~op:isNull(!results)) {
        !size = op:invokeMethod(!results, "size");
        if (op:gt(!size,3)) {
            //TODO: recovery policy
            !failConds = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "is(locationBelowMe,full)");
            exit(FAIL_PRECONDITIONS, !failConds);
        } else {
            !location = act:getCurrentLocation();
            !object = act:getHeldObject();
            act:putdown(!object,!location);
        }
    } else {
        !failConds = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(implemented(perceiveEntities))");
        exit(FAIL, !failConds);
    }
}

() = moveBlockTo(Symbol ?block:physobj, Symbol ?start:pose, Symbol ?destination:pose) {
    act:goToPose(?start);
    act:pickUp(?block);
    act:goToPose(?destination);
    act:putDown();
}

() = moveBlockTo(Symbol ?block:physobj, Symbol ?destination:pose) {
    conditions : {
        pre : holding(?actor,?block);
    }
    act:goToPose(?destination);
    act:putDown();
}

() = moveBlockTo(Symbol ?destination:pose) {
    conditions : {
        pre : not(free(?actor));
    }
    act:goToPose(?destination);
    act:putDown();
}

() = detectBlock(Symbol ?block:physobj) {
    effects : {
        success : knows(?actor,see(?actor,?block));
    }
    op:log(info,"detectBlock");
    act:perceiveEntity(?block);
}

//TODO
() = resetDomain() {
    conditions : {
        pre : free(?actor); //TODO can have recovery asking user where to put block down
    }
}

/////// ITK ///////
//Want this?
() = describeBlockLocation(Symbol ?block:physobj) {
    //TODO
}

//Want this?
//Describe known blocks in the work area and their locations
() = describeKnownBlocks() {
    //TODO
}

() = describeBlocksAt(Symbol ?pose:pose) {
    //TODO
}

() = describeNumberOfBlocksAt(Symbol ?pose:pose) {
    //TODO
}

//Could do same for locations if we want to
() = describeBlockTypes() {
    //TODO
}

() = goToAndDetect(Symbol ?block:physobj, Symbol ?pose:pose) {
    effects : {
        success : knows(?actor,at(?block,?pose));
    }

    act:goToPose(?pose);
    act:perceiveEntity(?block);
}

() = goToAndDetect(Symbol ?pose:pose) {
    effects : {
        success : knows(?actor,blocksAt(?pose));
    }

    act:goToPose(?pose);
    act:perceiveEntities(?block);
    //TODO: check if num results == 0 and fail if so
}