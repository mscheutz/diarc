import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.util.Map;
import java.lang.String;
import edu.tufts.hrilab.action.justification.Justification;
import java.lang.Double;
import java.lang.Integer;

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

(Symbol ?holding) = checkHolding(Symbol ?physobj) {
    Predicate !queryPred;
    List !bindings;

    ?holding = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "false");

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor,?physobj)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        ?holding = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "true");
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
    String !japaneseDescriptor;
    Symbol !holding;

    //TODO: assert ?object is actually a block?
    !blockPose = act:getBlockPose(?object);
    !holding = act:checkHolding(?object);
    if (op:equalsValue(!holding, false)) {
        if (op:isNull(!blockPose)) {
            !descriptor = tsc:getDescriptorForID(?object);
            (!bindings) = act:askQuestionFromString(?actor,"I dont know where the !descriptor is, at what pose can I expect to find it?", pose(X));
//            !japaneseDescriptor = tsc:getJapaneseVersionOfDescriptor(!descriptor);
//            (!bindings) = act:askQuestionFromString(?actor,"!japaneseDescriptorがどこにあるかは分かりません。場所何番にありますか?", pose(X));
            !blockPose = op:get(!bindings,!x);
            if (op:equals(!blockPose,none)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "know(?actor,theLocationOf,?object)");
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
    List !objectList;
    Symbol !object;
    Symbol !location;
    Integer !resultsSize;
    Map !bindings;
    Variable !x = "X";
    Predicate !failCond;

    op:log(info, "[pickUp]");
    //TODO: Need implementation of this function (retval should be collection of objects)
    !objectList = act:perceiveEntities();

    !resultsSize = op:invokeMethod(!objectList, "size");
    op:log(debug, "[pickUp] results size: !resultsSize for object list !objectList");

    //No results

    if (op:equals(!resultsSize, 0)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "dont(?actor,see,anyBlocks,atMyLocation)");
        exit(FAIL_RETURNVALUE,!failCond);
    } elseif (op:equals(!resultsSize, 1)) {
        ////Single result - pick up object
        !object = op:get(!objectList, 0);
        !location = act:getCurrentLocation();
        //act:grab(!object,!location);
    } else {
        ////Multiple results - ask which one to pick up
        //TODO: get descriptors for all detected blocks and add into question specifically
        !bindings = act:askQuestionFromString(?actor, "I see multiple blocks, which one do you want me to pick up?", blockType(X));
//        !bindings = act:askQuestionFromString(?actor, "視野に複数のブロックが見えています。どれを拾ったらいいでしょうか？", blockType(X));
        !object = op:get(!bindings, !x);
        ////TODO: make sure response matches one of the detected blocks
    }
    if (~op:equals(none)) {
        act:grabPerceivedEntity(!object,!location);
    }
}

//Placeholder
(List ?bindings) = perceiveEntities() {
    //runs a job that looks for all items known to the system and returns the set of (orderd) results. then binds the associated refrerences.

    Symbol !location;
    Symbol !blockAt;
    Predicate !fact;

    ?bindings = tsc:perceiveAllEntitiesInView("blocksDet");
    //todo: rectify block locations in belief based on what we're currently seeing. need to do at(X,Y) and holding(X).
//    !location = act:getCurrentLocation();
//    foreach (!result : ?bindings) {
//        !blockLocation = act:getBlockPose(!result);
//        if (~op:equals(!blockLocation, !location)) {
//            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result, !blockLocation)");
//            act:retractBelief(!fact);
//            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result, !blockLocation)");
//        }
//    }

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
    java.lang.Integer !size;
    Predicate !failCond;
    Justification !failJustification;
    Symbol !location;
    Symbol !object;
    Predicate !thisStep;

    op:log(info, "[putDown]");

//    !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "has(theLocationBelowMe,space)");
//    !failJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
//    !thisStep = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "putDown(?actor)");
//    op:invokeMethod(!failJustification, "setStep", !thisStep);
//    exit(FAIL_PRECONDITIONS, !failJustification);

    ////TODO: Should just push this down into the putdown primitive and have recovery policy only apply to this script
    !results = act:perceiveEntities();
    if (~op:isNull(!results)) {
        !size = op:invokeMethod(!results, "size");
        if (op:gt(!size,1)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "has(theLocationBelowMe,space)");
            !failJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
            !thisStep = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "putDown(?actor)");
            op:invokeMethod(!failJustification, "setStep", !thisStep);
            exit(FAIL_PRECONDITIONS, !failJustification);
        } else {
        !location = act:getCurrentLocation();
        !object = act:getHeldObject();
        act:putdown(!object,!location);
        }
    }
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
    (!bindings) = act:askQuestionFromString(?actor,"I cannot put down the block because the location is full. Would you like me to swap out one of the blocks?", val(X));
    //    (!bindings) = act:askQuestionFromString(?actor,"場所がいっぱいになっているため、ブロックを置くことはできません。ブロックのどれかと交換しましょうか？", val(X));
    //      Y: act:askQuestionFromString(?actor,"Which block should be swapped?", blockType(X));
    //      Y: act:askQuestionFromString(?actor,"どのブロックと交換したらいいですか？", blockType(X));
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
    String !poseDescriptorJapanese;
    String !blockDescriptorJapanese;

    !blockPose = act:getBlockPose(?block);
    !blockDescriptor = tsc:getDescriptorForID(?block);
    if (op:isNull(!blockPose)) {
        act:generateResponseFromString("I do not know where the !blockDescriptor is");
//        !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
//        act:generateResponseFromString("!blockDescriptorJapaneseがどこにあるかが分かりません。");
    } else {
        !poseDescriptor = tsc:getDescriptorForPoseID(!blockPose);
        act:generateResponseFromString("the !blockDescriptor is at !poseDescriptor");
//        !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
//        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!poseDescriptor);
//        act:generateResponseFromString("!blockDescriptorJapaneseは!poseDescriptorJapaneseにあります");
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
    String !poseDescriptorJapanese;
    String !blockDescriptorJapanese;
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
//            !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
//            !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!poseDescriptor);
//            act:generateResponseFromString("!blockDescriptorJapaneseは!poseDescriptorJapaneseにあります");
            !knowBlockAt = op:newObject("java.lang.Boolean", true);
        }
    }
    if (op:equals(!knowBlockAt,False)) {
        //act:generateResponseFromString("I don't know the location of any blocks");
        act:generateResponseFromString("ブロックはどれも場所がわかりません");
    }
}

() = describeBlocksAt(Symbol ?pose:pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !block;
    Symbol !blockDescriptor;
    Symbol !poseDescriptor;
    String !blockDescriptorJapanese;
    String !poseDescriptorJapanese;
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
//            !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
//            act:generateResponseFromString("!blockDescriptorJapaneseがあります");
            !knowBlockAt = op:newObject("java.lang.Boolean", true);
        }
    }
    if (op:equals(!knowBlockAt,False)) {
        !poseDescriptor = tsc:getDescriptorForPoseID(?pose);
        act:generateResponseFromString("There are no blocks at !poseDescriptor");
//        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!poseDescriptor);
//        act:generateResponseFromString("!poseDescriptorJapaneseにブロックはありません");
    }
}

() = describeNumberOfBlocksAt(Symbol ?pose:pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !block;
    Symbol !blockDescriptor;
    Symbol !poseDescriptor;
    String !poseDescriptorJapanese;
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
//        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!poseDescriptor);
//        act:generateResponseFromString("!poseDescriptorJapaneseにブロックは一個あります");
    } else {
        act:generateResponseFromString("There are !numBlocks blocks at !poseDescriptor");
//        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!poseDescriptor);
//        act:generateResponseFromString("!poseDescriptorJapaneseにブロックが!numBlocks個あります");
    }
}

//Could do same for locations if we want to
() = describeBlockTypes() {
    //TODO
    //?blockは感知できます == (I know how to detect ?block)
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
            //act:generateResponseFromString("!status goal: ");
            act:generateResponseFromString("!status ゴール: ");
            act:generateResponse(!goalPred);
        }
    } else {
        //act:generateResponseFromString("no current goal");
        act:generateResponseFromString("現在ゴールはありません");
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
       //act:generateResponseFromString("no pending goals");
       act:generateResponseFromString("並んでいるゴールはありません");
    }

}

() = openGripper() {
    tsc:openGripper();
}

() = closeGripper() {
    tsc:closeGripper();
}
