import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.util.Map;

(Symbol ?blockPose) = askForBlockPose(Symbol ?descriptor) {
    Map !bindings;
    Variable !x = "X";
    String !japaneseDescriptor;

    !japaneseDescriptor = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    (!bindings) = act:askQuestionFromString(?actor,"!japaneseDescriptorがどこにあるかは分かりません。場所何番にありますか?", pose(X));
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
    !bindings = act:askQuestionFromString(?actor, "視野に複数のブロックが見えています。どれを拾ったらいいでしょうか？", blockType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings,!x);
    }
}

() = reportNotPickingUpBlock() {
    act:generateResponseFromString("わかった");
}

(Symbol ?response) = reportFullLocationAndAskToSwap() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"場所がいっぱいになっているため、ブロックを置くことはできません。ブロックのどれかと交換しましょうか？", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?block) = askForBlockToSwap() {
    Map !bindings;
    Variable !x = "X";

    act:askQuestionFromString(?actor,"どのブロックと交換したらいいですか？", blockType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?block = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?block = op:get(!bindings, !x);
    }
}

() = reportNotSwappingBlocks() {
    //TODO: this is google translate result, get actual translation
    act:generateResponseFromString("わかりました。ブロックを交換しません");
}

() = reportHoldingBlock(Symbol ?descriptor) {
    String !blockDescriptorJapanese;

    !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("私は!blockDescriptorJapaneseを持っています");
}

() = reportBlockLocationUnknown(Symbol ?descriptor) {
    String !blockDescriptorJapanese;

    !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!blockDescriptorJapaneseがどこにあるかが分かりません。");
}

() = reportBlockIsAtPose(Symbol ?blockDescriptor, Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    String !blockDescriptorJapanese;

    !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?blockDescriptor);
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);
    act:generateResponseFromString("!blockDescriptorJapaneseは!poseDescriptorJapaneseにあります");
}

() = reportBlockLocationsAllUnknown() {
    act:generateResponseFromString("ブロックはどれも場所がわかりません");
}

() = reportBlockExists(Symbol ?descriptor) {
    String !blockDescriptorJapanese;

    !blockDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?escriptor);
    act:generateResponseFromString("!blockDescriptorJapaneseがあります");
}

() = reportNoBlocksAtPose(Symbol ?descriptor) {
    String !poseDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!poseDescriptorJapaneseにブロックはありません");
}

() = reportNumberOfBlocksAt(java.lang.Integer ?numBlocks, Symbol ?descriptor) {
    String !poseDescriptorJapanese;

    if (op:equals(?numBlocks,1)) {
        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
        act:generateResponseFromString("!poseDescriptorJapaneseにブロックは一個あります");
    } else {
        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
        act:generateResponseFromString("!poseDescriptorJapaneseにブロックが?numBlocks個あります");
    }
}

() = reportGoalStatus(java.lang.String ?status) {
    act:generateResponseFromString("?status ゴール: ");
}

() = reportNoCurrentGoal() {
    act:generateResponseFromString("現在ゴールはありません");
}

() = reportNoPendingGoal() {
   act:generateResponseFromString("並んでいるゴールはありません");
}