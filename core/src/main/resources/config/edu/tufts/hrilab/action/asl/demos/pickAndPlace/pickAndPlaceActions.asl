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
    } else {
        ?physobj = op:setNull();
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

(Symbol ?pose) = getPoseWithSpace() {
    java.lang.Integer !numBlocks;
    java.lang.Integer !poseIndex = 0;
    java.lang.Boolean !doneChecking = false;

    op:log(debug,"getPoseWithSpace");

    while (op:equals(!doneChecking, false)) {
        ?pose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "pose_!poseIndex:pose");
        !numBlocks = act:getNumberOfBlocksAt(?pose);
        if (op:lt(!numBlocks,2)) {
            !doneChecking = op:newObject("java.lang.Boolean", true);
        }
        !poseIndex = op:++(!poseIndex);
        //TODO: Don't have number of locations hardcoded
        if (op:gt(!poseIndex,2)) {
            ?pose = op:setNull();
            !doneChecking = op:newObject("java.lang.Boolean", true);
        }
    }
}

() = openGripper() {
    tsc:openGripper();
}

() = closeGripper() {
    tsc:closeGripper();
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
            //!japaneseDescriptor = tsc:getJapaneseVersionOfDescriptor(!descriptor);
            //(!bindings) = act:askQuestionFromString(?actor,"!japaneseDescriptorがどこにあるかは分かりません。場所何番にありますか?", pose(X));
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

    //TODO: get rid of this doubly nested action wrapping of goalspecs. Interacts badly with recovery - need to
    //      specifically exclude the parent from any goal recovery policies
    //  ^Is this actually just due to bad implementation for goal recovery w/ goalspecs in general?
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
        act:grabPerceivedEntity(!object,!location);
    } else {
        ////Multiple results - ask which one to pick up
        //TODO: get descriptors for all detected blocks and add into question specifically
        !bindings = act:askQuestionFromString(?actor, "I see multiple blocks, which one do you want me to pick up?", blockType(X));
//        !bindings = act:askQuestionFromString(?actor, "視野に複数のブロックが見えています。どれを拾ったらいいでしょうか？", blockType(X));
        !object = op:get(!bindings, !x);
        ////TODO: make sure response matches one of the detected blocks
        if (~op:equals(!object, none)) {
            !location = act:getCurrentLocation();
            act:grabPerceivedEntity(!object,!location);
        } else {
            act:generateResponseFromString("okay, I am not picking up any block");
            //act:generateResponseFromString("わかった");
        }
    }

}

//Placeholder
(List ?bindings) = perceiveEntities() {
    //runs a job that looks for all items known to the system and returns the set of (orderd) results. then binds the associated refrerences.

    Symbol !location;
    Symbol !blockLocation;
    Predicate !fact;

    java.lang.Double !objectHeightM = 0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED
    java.lang.Double !negObjectHeightM = -0.050;//todo: URGENT @ PETE THIS SHOULDN'T BE HARDCODED

    tsc:moveZRelative(!objectHeightM);
    ?bindings = tsc:perceiveAllEntitiesInView("blocksDet");
    tsc:moveZRelative(!negObjectHeightM);
    !location = act:getCurrentLocation();
    foreach (!result : ?bindings) {
        !blockLocation = act:getBlockPose(!result);
        if (~op:equals(!blockLocation, !location)) {
            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(holding(?actor,!result))");
            act:retractBelief(!fact);
            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result,!blockLocation)");
            act:retractBelief(!fact);
            !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!result,!location)");
            act:assertBelief(!fact);
        }
    }

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

    act:putDown();
}

() = putDown() {
    conditions : {
        pre : not(free(?actor));
    }
    Symbol !location;
    Symbol !object;

    op:log(info, "[putDown]");

    !location = act:getCurrentLocation();
    !object = act:getHeldObject();
    act:putdown(!object,!location);
}

() = moveBlockTo(Symbol ?block:physobj, Symbol ?start:pose, Symbol ?destination:pose) {
    act:goToPose(?start);
    act:pickUp(?block);
    act:goToPose(?destination);
    act:putDown();
}

() = swapBlocks(Symbol ?blockOne, Symbol ?blockTwo) {
    Predicate !goalPred;
    Predicate !failCond;
    Symbol !openPose;
    Symbol !blockOnePose;
    Symbol !blockTwoPose;
    Symbol !heldBlock;
    java.lang.Integer !numBlocks1;
    java.lang.Integer !numBlocks2;
    java.lang.Boolean !true = True;
    java.lang.Boolean !false = False;

    //Make sure block locations are known (and check if one of them is currently being held)
    !heldBlock = act:getHeldObject();
    !blockOnePose = act:getBlockPose(?blockOne);
    !blockTwoPose = act:getBlockPose(?blockTwo);
    if (~op:isNull(!heldBlock)) {
        if (op:equals(!heldBlock,?blockOne)) {
            !blockOnePose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "holding");
        } elseif (op:equals(!heldBlock,?blockTwo)) {
            !blockTwoPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "holding");
        }
    }

    //Fail if either block is not present in the work area
    if (op:isNull(!blockOnePose)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "know(?actor,theLocationOf,?blockOne)");
        exit(FAIL, !failCond);
    } elseif (op:isNull(!blockTwoPose)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "know(?actor,theLocationOf,?blockTwo)");
        exit(FAIL, !failCond);
    }

    if (~op:isNull(!heldBlock)) {
        if (op:equals(?blockOne,!heldBlock)) {
            act:swapHeldBlock(!heldBlock,?blockTwo,!blockTwoPose);
            return;
        } elseif (op:equals(?blockTwo,!heldBlock)) {
            act:swapHeldBlock(!heldBlock,?blockOne,!blockOnePose);
            return;
        } else {
            //TODO: ideally not to one of the swap poses
            //Move held block to open location
            !openPose = act:getPoseWithSpace();
            if (op:isNull(!openPose)) {
                !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapBlocks)");
                exit(FAIL, !failCond);
            }
            op:log(info,"moving held block to open location");
            !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, !openPose)");
            goal:!goalPred;
            act:putDown();
        }
    }

    !numBlocks1 = act:getNumberOfBlocksAt(!blockOnePose);
    !numBlocks2 = act:getNumberOfBlocksAt(!blockTwoPose);
    if (op:gt(!numBlocks1,1) && op:gt(!numBlocks2,1)) {
        act:swapBlocksHelper(?blockOne,!blockOnePose,?blockTwo,!blockTwoPose, !true);
    } elseif (op:gt(!numBlocks1,1)) {
        act:swapBlocksHelper(?blockOne,!blockOnePose,?blockTwo,!blockTwoPose, !false);
    } else {
        act:swapBlocksHelper(?blockTwo,!blockTwoPose,?blockOne,!blockOnePose, !false);
    }

    //Pick original held block back up
    if (~op:isNull(!heldBlock)) {
        op:log(info,"picking original block back up");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, !heldBlock)");
        goal:!goalPred;
    }
}

//TODO: is there a bug with assigning args in calls between asl scripts where args have the same names but in a different order?
() = swapBlocksHelper(Symbol ?block1, Symbol ?block1Pose, Symbol ?block2, Symbol ?block2Pose, java.lang.Boolean ?poseFull) {
    Predicate !goalPred;
    Predicate !failCond;
    Symbol !openPose;

    if (op:equals(?poseFull,true)) {
        //Move block two temporarily to an open location
        !openPose = act:getPoseWithSpace();
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapBlocks)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving block two to open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?block2, !openPose)");
        goal:!goalPred;
    }

    //move block one to block two's pose
    op:log(info,"moving block one to desired location");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?block1, ?block2Pose)");
    goal:!goalPred;

    //Move block two to block one's location
    op:log(info,"moving block two to desired location");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?block2, ?block1Pose)");
    goal:!goalPred;
}

() = swapHeldBlock(Symbol ?heldBlock, Symbol ?blockToSwap, Symbol ?blockToSwapPose) {
    Predicate !goalPred;
    java.lang.Integer !numBlocks;
    Predicate !failCond;

    !numBlocks = act:getNumberOfBlocksAt(!blockToSwapPose);
    if (op:gt(!numBlocks,1)) {
        //Move held block to open location
        !openPose = act:getPoseWithSpace();
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapBlocks)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving held block to open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, !openPose)");
        goal:!goalPred;
        act:putDown();

        //Move block to swap temporarily to an open location
        !openPose = act:getPoseWithSpace();
        if (op:isNull(!openPose)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,openSpace,to,swapBlocks)");
            exit(FAIL, !failCond);
        }
        op:log(info,"moving block to swap to an open location");
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?blockToSwap, !openPose)");
        goal:!goalPred;
    }

    //Move held block to swap blocks location
    op:log(info,"moving held block to desired location");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?heldBlock, ?blockToSwapPose)");
    goal:!goalPred;

    //Pick block to swap up
    op:log(info,"pick swapped block up");
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, ?blockToSwap)");
    goal:!goalPred;
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
    recovery : {
        actionStatuses : {FAIL_PRECONDITIONS}
        failureReasons : {has(theLocationBelowMe,space)}
        goals : {getBlockTo(X,Y,Z)}
    }
    Symbol !response;
    Symbol !otherBlock;
    Map !bindings;
    Variable !x = "X";
    String !otherBlockName;
    Symbol !block;

    op:log("debug","[putDownAtFullLocationRecovery");
    op:log("debug","[putDownAtFullLocationRecovery] ?failedActionPredicates");
    op:log("debug","[putDownAtFullLocationRecovery] ?failureReasons");
    op:log("debug","[putDownAtFullLocationRecovery] ?failedGoal");

    //TODO: Get the held block and current location from belief in order to populate question asking?
    //  report that we cannot put down the block because the location is full
    //  Ask if the user would like to swap the held block with a block in that location
    (!bindings) = act:askQuestionFromString(?actor,"I cannot put down the block because the location is full. Would you like me to swap out one of the blocks?", val(X));
    !block = act:getHeldObject();
    //(!bindings) = act:askQuestionFromString(?actor,"場所がいっぱいになっているため、ブロックを置くことはできません。ブロックのどれかと交換しましょうか？", val(X));
    !response = op:get(!bindings, !x);
    if (op:equals(!response,yes)) {
        !bindings = act:askQuestionFromString(?actor,"Which block should be swapped?", blockType(X));
        //act:askQuestionFromString(?actor,"どのブロックと交換したらいいですか？", blockType(X));
        !otherBlock = op:get(!bindings, !x);
        !otherBlockName = op:invokeMethod(!otherBlock, "getName");
        if (op:equals(!otherBlockName,"none")) {
            act:generateResponseFromString("okay, I am not swapping any block");
            exit(FAIL_RECOVERY);
        } else {
            act:swapBlocks(!block,!otherBlock);
        }
    } else {
        act:generateResponseFromString("okay, I am not swapping any block");
        exit(FAIL_RECOVERY);
    }
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
        if (act:checkHolding(?block)) {
            act:generateResponseFromString("I am holding the !blockDescriptor");
            //!blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
            //act:generateResponseFromString("私は!blockDescriptorJapaneseを持っています");
        } else {
            act:generateResponseFromString("I do not know where the !blockDescriptor is");
            //!blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
            //act:generateResponseFromString("!blockDescriptorJapaneseがどこにあるかが分かりません。");
        }
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
    !block = act:getHeldObject();
    if (~op:isNull(!block) && act:checkHolding(!block)) {
        !blockDescriptor = tsc:getDescriptorForID(!block);
        act:generateResponseFromString("I am holding the !blockDescriptor");
        //!blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(!blockDescriptor);
        //act:generateResponseFromString("私は!blockDescriptorJapaneseを持っています");
        !knowBlockAt = op:newObject("java.lang.Boolean", true);
    }
    if (op:equals(!knowBlockAt,False)) {
        act:generateResponseFromString("I don't know the location of any blocks");
        //act:generateResponseFromString("ブロックはどれも場所がわかりません");
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

(java.lang.Integer ?numBlocks) = getNumberOfBlocksAt(Symbol ?pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !block;
    Symbol !blockDescriptor;
    edu.tufts.hrilab.fol.Variable !blockVar;

    ?numBlocks = op:newObject("java.lang.Integer", 0);

    !blockVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,?pose)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !block = op:get(!binding,!blockVar);
        !blockDescriptor = tsc:getDescriptorForID(!block);
        if (~op:equals(!blockDescriptor,none)) {
            ?numBlocks = op:++(?numBlocks);
        }
    }
}

() = describeNumberOfBlocksAt(Symbol ?pose) {
    java.lang.Integer !numBlocks;
    Symbol !poseDescriptor;
    String !poseDescriptorJapanese;

    !numBlocks = act:getNumberOfBlocksAt(?pose);

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
    List !results;

    act:goToPose(?pose);
    !results = act:perceiveEntities();
    if (op:isEmpty(!results)) {
        exit(FAIL);
    }
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
            //act:generateResponseFromString("!status ゴール: ");
            act:generateResponse(!goalPred);
        }
    } else {
        act:generateResponseFromString("no current goal");
        //act:generateResponseFromString("現在ゴールはありません");
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
       //act:generateResponseFromString("並んでいるゴールはありません");
    }

}