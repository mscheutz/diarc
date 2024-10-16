import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.util.Map;

(Symbol ?blockPose) = askForBlockPose(Symbol ?descriptor) {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I dont know where the ?descriptor is, at what pose can I expect to find it?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?blockPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?blockPose = op:get(!bindings,!x);
    }
}

(Symbol ?object) = askForBlockToPickUp() {
    Variable !x = "X";
    Map !bindings;

    //TODO: get descriptors for all detected blocks and add into question specifically
    !bindings = act:askQuestionFromString(?actor, "I see multiple blocks, which one do you want me to pick up?", blockType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings,!x);
    }
}

() = reportNotPickingUpBlock() {
    act:generateResponseFromString("okay, I am not picking up any block");
}

(Symbol ?response) = reportFullLocationAndAskToSwap() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I cannot put down the block because the location is full. Would you like me to swap out one of the blocks?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?block) = askForBlockToSwap() {
    Map !bindings;
    Variable !x = "X";

    !bindings = act:askQuestionFromString(?actor,"Which block should be swapped?", blockType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?block = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?block = op:get(!bindings, !x);
    }
}

() = reportNotSwappingBlocks() {
    act:generateResponseFromString("okay, I am not swapping any block");
}

() = reportHoldingBlock(Symbol ?descriptor) {
    act:generateResponseFromString("I am holding the ?descriptor");
}

() = reportBlockLocationUnknown(Symbol ?descriptor) {
    act:generateResponseFromString("I do not know where the ?descriptor is");
}

() = reportBlockIsAtPose(Symbol ?blockDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("the ?blockDescriptor is at ?poseDescriptor");
}

() = reportBlockLocationsAllUnknown() {
    act:generateResponseFromString("I don't know the location of any blocks");
}

() = reportBlockExists(Symbol ?descriptor) {
    act:generateResponseFromString("There is a ?descriptor");
}

() = reportNoBlocksAtPose(Symbol ?descriptor) {
    act:generateResponseFromString("There are no blocks at ?descriptor");
}

() = reportNumberOfBlocksAt(java.lang.Integer ?numBlocks, Symbol ?descriptor) {
    if (op:equals(?numBlocks,1)) {
        act:generateResponseFromString("There is ?numBlocks block at ?descriptor");
    } else {
        act:generateResponseFromString("There are ?numBlocks blocks at ?descriptor");
    }
}

() = reportGoalStatus(java.lang.String ?status) {
    act:generateResponseFromString("?status goal: ");
}

() = reportNoCurrentGoal() {
    act:generateResponseFromString("no current goal");
}

() = reportNoPendingGoal() {
   act:generateResponseFromString("no pending goals");
}