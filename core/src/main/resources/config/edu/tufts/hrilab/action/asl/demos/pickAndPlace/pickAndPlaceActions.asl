import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;
import edu.tufts.hrilab.action.justification.Justification;

(Symbol ?location) = getCurrentLocation() {
    Predicate !queryPred;
    List !bindings;

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
    List !bindings;

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

(Symbol ?pose) = getBlockPose(Symbol ?physobj) {
    Predicate !queryPred;
    List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?physobj,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (?pose) =op:get(!elem,!bindingVar);
    }
}

() = openGripper() {
    tsc:openGripper();
}

() = closeGripper() {
    tsc:openGripper();
}

//todo: (pete) allows the LLM target for semantics to be more sane than "getOn". should be true everywhere.
() = getBlockTo(Symbol ?object, Symbol ?destination) {
    Symbol !blockPose;
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Predicate !failCond;
    Predicate !blockAtPred;
    Symbol !descriptor;
    Symbol !heldBlock;

    //TODO: assert ?object is actually a block?
    !blockPose = act:getBlockPose(?object);
    if (op:isNull(!blockPose)) {
        !heldBlock = act:getHeldObject();
        if (~op:equals(?object,!heldBlock)) {
            !descriptor = tsc:getDescriptorForID(?object);
            (!bindings) = act:askQuestionFromString(?actor,"I dont know where the !descriptor is, at what pose can I expect to find it?", pose(X));
            !blockPose = op:get(!bindings,!x);
            if (op:equals(!blockPose,none)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,theLocationOf,?object))");
                exit(FAIL, !failCond);
            } else {
                !blockAtPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,!blockPose)");
                act:assertBelief(!blockAtPred);
            }
        }
    }

    act:getOn(?object, ?destination);
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
    Predicate !failCond;
    !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "dont(?actor,see,anyBlocks,atMyLocation)");
    exit(FAIL_RETURNVALUE,!failCond);

    ////Multiple results - ask which one to pick up
    //Map !bindings;
    //Variable !x = "X";
    ////TODO: get descriptors for all detected blocks and add into question specifically
    //!bindings = act:askQuestionFromString("I see multiple blocks, which one do you want me to pick up?", blockType(X));
    //!object = op:get(!bindings, !x);
    ////TODO: make sure response matches one of the detected blocks

    ////Single result - pick up object
    !location = act:getCurrentLocation();
    //act:grab(!object,!location);
    act:grabPerceivedEntity(!object,!location);
}

//Placeholder
(List ?bindings) = perceiveEntities() {
//(Symbol ?bindings) = perceiveEntities() {
//    ?bindings = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "robotonephysobj_0:physobj");
//    act:perceiveEntity(?bindings);
}

() = pickUp(Symbol ?object) {
    conditions : {
        pre : free(?actor);
    }

    op:log(info, "[pickUp] ?object");

    Symbol !blockPose;
    Symbol !armPose;
    !blockPose = act:getBlockPose(?object);
    !armPose = act:getCurrentLocation();
    if (~op:isNull(!blockPose) && ~op:equals(!armPose,!blockPose)) {
        op:log(debug, "going to !blockPose");
        act:goToPose(!blockPose);
        act:perceiveEntity(?object);
        act:grabPerceivedEntity(?object,!blockPose);
        //act:grab(?object,!blockPose);
    } else {
        op:log(debug, "looking at !armPose");
        act:perceiveEntity(?object);
        act:grabPerceivedEntity(?object,!armPose);
        //act:grab(?object,!armPose);
    }
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
    List !results;
    java.lang.Integer !resultsSize;
    Predicate !failCond;
    Justification !failJustification;
    Symbol !location;
    Symbol !object;
    Predicate !thisStep;

    op:log(info, "[putDown]");

    !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "has(theLocationBelowMe,space)");
    !failJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
    !thisStep = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "putDown(?actor)");
    op:invokeMethod(!failJustification, "setStep", !thisStep);
    exit(FAIL_PRECONDITIONS, !failJustification);

    ////TODO: Should just push this down into the putdown primitive and have recovery policy only apply to this script
    //!results = act:perceiveEntities();
    //if (~op:isNull(!results)) {
    //    !size = op:invokeMethod(!results, "size");
    //    if (op:gt(!size,3)) {
    //        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "has(theLocationBelowMe,space)");
    //        !failJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
    //        !thisStep = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "putDown(?actor)");
    //        op:invokeMethod(!failJustification, "setStep", !thisStep);
    //        exit(FAIL_PRECONDITIONS, !failJustification);
    //    } else {
    //        !location = act:getCurrentLocation();
    //        !object = act:getHeldObject();
    //        act:putdown(!object,!location);
    //    }
    //} else {
    //    !failConds = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(implemented(perceiveEntities))");
    //    exit(FAIL, !failConds);
    //}
}

() = moveBlockTo(Symbol ?block:physobj, Symbol ?start:pose, Symbol ?destination:pose) {
    act:goToPose(?start);
    act:pickUp(?block);
    act:goToPose(?destination);
    act:putDown();
}

//This currently applies to all goals and compound actions. putDown() and putDown(block) are allowed to just fail. Do we want to change this?
() = putDownAtFullLocationRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_success
        }
        actionStatuses : {FAIL_PRECONDITIONS}
        failureReasons : {has(theLocationBelowMe,space)}
        excludedGoals : {putDown(X), putDown(X,Y)}

    }

    op:log("info","[putDownAtFullLocationRecovery]");
    op:log("info","[putDownAtFullLocationRecovery] ?failedActionPredicates");
    op:log("info","[putDownAtFullLocationRecovery] ?failureReasons");
    op:log("info","[putDownAtFullLocationRecovery] ?failedGoal");

    //Get the held block and current location from belief in order to populate question asking?

    //TODO:
    //  report that we cannot put down the block because the location is full
    //  Ask if the user would like to swap the held block with a block in that location
    //    (!bindings) = act:askQuestionFromString(?actor,"I cannot put down the block because the location is full. Would you like me to swap out one of the blocks?", val(X));
    //      Y: act:askQuestionFromString(?actor,"Which block should be swapped?", blockType(X));
    //          BlockType: TODO: Need swapBlock() implementation - asks where to put the held block in the interim
    //      N: exit(FAIL_RECOVERY)
    exit(FAIL_RECOVERY);
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

//TODO: this breaks planning for goToPose? Figure out issue and comment back in
() = detectBlock(Symbol ?block) {
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
() = describeBlockPose(Symbol ?block:physobj) {
    Symbol !blockPose;
    Symbol !poseDescriptor;
    Symbol !blockDescriptor;

    !blockPose = act:getBlockPose(?block);
    !blockDescriptor = tsc:getDescriptorForID(?block);
    if (op:isNull(!blockPose)) {
        act:generateResponseFromString("I do not know where the !blockDescriptor is");
    } else {
        !poseDescriptor = tsc:getDescriptorForPoseID(!blockPose);
        act:generateResponseFromString("the !blockDescriptor is at !poseDescriptor");
    }
}

//Describe known blocks in the work area and their locations
() = describeKnownBlocks() {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !pose;
    Symbol !block;
    Symbol !poseDescriptor;
    Symbol !blockDescriptor;
    java.lang.Boolean !knowBlockAt = false;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !blockVar;
    edu.tufts.hrilab.fol.Variable !poseVar;

    !blockVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !poseVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "Y");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,Y)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !block = op:get(!binding,!blockVar);
        !pose = op:get(!binding,!poseVar);
        !blockDescriptor = tsc:getDescriptorForID(!block);
        !poseDescriptor = tsc:getDescriptorForPoseID(!pose);
        if (~op:equals(!blockDescriptor,none)) {
            act:generateResponseFromString("The !blockDescriptor is at !poseDescriptor");
            !knowBlockAt = op:newObject("java.lang.Boolean", true);
        }
    }
    if (op:equals(!knowBlockAt,False)) {
        act:generateResponseFromString("I don't know the location of any blocks");
    }
}

() = describeBlocksAt(Symbol ?pose:pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !block;
    Symbol !blockDescriptor;
    Symbol !poseDescriptor;
    java.lang.Boolean !knowBlockAt = false;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !blockVar;

    !blockVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,?pose)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !block = op:get(!binding,!blockVar);
        !blockDescriptor = tsc:getDescriptorForID(!block);
        if (~op:equals(!blockDescriptor,none)) {
            act:generateResponseFromString("There is a !blockDescriptor");
            !knowBlockAt = op:newObject("java.lang.Boolean", true);
        }
    }
    if (op:equals(!knowBlockAt,False)) {
        !poseDescriptor = tsc:getDescriptorForPoseID(?pose);
        act:generateResponseFromString("There are no blocks at !poseDescriptor");
    }
}

() = describeNumberOfBlocksAt(Symbol ?pose:pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !block;
    Symbol !blockDescriptor;
    Symbol !poseDescriptor;
    java.lang.Boolean !knowBlockAt = false;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !blockVar;
    java.lang.Integer !numBlocks = 0;

    !blockVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,?pose)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !block = op:get(!binding,!blockVar);
        !blockDescriptor = tsc:getDescriptorForID(!block);
        if (~op:equals(!blockDescriptor,none)) {
            !numBlocks = op:++(!numBlocks);
        }
    }

    !poseDescriptor = tsc:getDescriptorForPoseID(?pose);
    if (op:equals(!numBlocks,1)) {
        act:generateResponseFromString("There is !numBlocks block at !poseDescriptor");
    } else {
        act:generateResponseFromString("There are !numBlocks blocks at !poseDescriptor");
    }
}

//Could do same for locations if we want to
() = describeBlockTypes() {
    //TODO
}

() = goToAndDetect(Symbol ?block, Symbol ?pose) {
    effects : {
        success : knows(?actor,at(?block,?pose));
    }

    act:goToPose(?pose);
    act:perceiveEntity(?block);
}

() = goToAndDetect(Symbol ?pose) {
    effects : {
        success : knows(?actor,blocksAt(?pose));
    }

    act:goToPose(?pose);
    act:perceiveEntities();
    //TODO: check if num results == 0 and fail if so
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
            act:generateResponseFromString("!status goal: ");
            act:generateResponse(!goalPred);
        }
    } else {
        act:generateResponseFromString("no current goal");
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
       act:generateResponseFromString("no pending goals");
    }

}