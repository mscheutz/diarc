import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.util.Map;

(Symbol ?objectPose) = askForObjectPose(Symbol ?descriptor) {
    Map !bindings;
    Variable !x = "X";
    String !japaneseDescriptor;

    !japaneseDescriptor = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    (!bindings) = act:askQuestionFromString(?actor,"!japaneseDescriptorがどこにあるかは分かりません。場所何番にありますか?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?objectPose = op:get(!bindings,!x);
        act:assertIsPose(?objectPose);
    }
}

(Symbol ?object) = askForObjectToPickUp() {
    Variable !x = "X";
    Map !bindings;

    //TODO: get descriptors for all detected objects and add into question specifically
    !bindings = act:askQuestionFromString(?actor, "視野に複数のブロックが見えています。どれを拾ったらいいでしょうか？", objectType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings,!x);
        act:assertIsPhysobj(?object);
    }
}

() = reportNotPickingUpObject() {
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

(Symbol ?object) = askForObjectToSwap() {
    Map !bindings;
    Variable !x = "X";

    act:askQuestionFromString(?actor,"どのブロックと交換したらいいですか？", objectType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings, !x);
        act:assertIsPhysobj(?object);
    }
}

() = reportNotSwappingObjects() {
    //TODO: this is google translate result, get actual translation
    act:generateResponseFromString("わかりました。ブロックを交換しません");
}

() = reportHoldingObject(Symbol ?descriptor) {
    String !objectDescriptorJapanese;

    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("私は!objectDescriptorJapaneseを持っています");
}

() = reportObjectLocationUnknown(Symbol ?descriptor) {
    String !objectDescriptorJapanese;

    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!objectDescriptorJapaneseがどこにあるかが分かりません。");
}

() = reportObjectIsAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    String !objectDescriptorJapanese;

    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?objectDescriptor);
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);
    act:generateResponseFromString("!objectDescriptorJapaneseは!poseDescriptorJapaneseにあります");
}

() = reportObjectLocationsAllUnknown() {
    act:generateResponseFromString("ブロックはどれも場所がわかりません");
}

() = reportObjectExists(Symbol ?descriptor) {
    String !objectDescriptorJapanese;

    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?escriptor);
    act:generateResponseFromString("!objectDescriptorJapaneseがあります");
}

() = reportNoObjectsAtPose(Symbol ?descriptor) {
    String !poseDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!poseDescriptorJapaneseにブロックはありません");
}

() = reportNumberOfObjectsAt(java.lang.Integer ?numObjects, Symbol ?descriptor) {
    String !poseDescriptorJapanese;

    if (op:equals(?numObjects,1)) {
        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
        act:generateResponseFromString("!poseDescriptorJapaneseにブロックは一個あります");
    } else {
        !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
        act:generateResponseFromString("!poseDescriptorJapaneseにブロックが?numObjects個あります");
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

() = reportCurrentPose(Symbol ?armPose) {
    String !poseDescriptorJapanese;
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);

   //TODO: get translation
   //act:generateResponseFromString("I am currently at ?armPose");
}

() = reportObjectToSwapNotAtCurrentPose() {
   //TODO: get translation
   act:generateResponseFromString("The object you indicated is not at my current location");
}

() = reportObjectsToSwapAreTheSame() {
   //TODO: get translation
   act:generateResponseFromString("You asked me to swap an object with itself. Doing nothing");
}

() = reportDetectedObject(Symbol ?objectDescriptor) {
    act:generateResponseFromString("yes, I see ?objectDescriptor");
}

() = reportCantVerifyBecauseActiveGoal() {
    act:generateResponseFromString("I cannot verify that because I am currently doing something");
}

() = reportObjectNotBelievedToBeAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not believe ?objectDescriptor is at ?poseDescriptor");
}

() = reportObjectBelievedToBeAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I believe ?objectDescriptor is at ?poseDescriptor");
}

() = reportObjectNotAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not see ?objectDescriptor at ?poseDescriptor");
}

() = reportDetectedObject(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I see ?objectDescriptor at ?poseDescriptor");
}

(Symbol ?response) = askToVerifyBelief() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"Would you like me to verify that?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

() = reportDontBelieveObjectsAtPose(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not believe any objects are at ?poseDescriptor");
}

() = reportBelieveObjectsAtPose(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I believe objects are at ?poseDescriptor");
}

() = reportObjectsNotAtPose(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not see any objects at ?poseDescriptor");
}

() = reportDetectedObjects(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I see objects at ?poseDescriptor");
}

(Symbol ?response) = reportHoldingObjectAndAskToPutDown() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I cannot pick up the object because I am holding something. Would you like me to put it down somewhere?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?objectPose) = askForPose() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"Where should I put it down?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?objectPose = op:get(!bindings,!x);
        act:assertIsPose(?objectPose);
    }
}

() = reportKnownObjects(java.util.List ?objects) {
    String !descriptor;
    String !japaneseDescriptor;

    foreach (!descriptor : ?objects) {
        !japaneseDescriptor = tsc:getJapaneseVersionOfDescriptor(!descriptor);
        act:generateResponseFromString(!japaneseDescriptor);
    }
}

() = reportKnownPoses(java.util.List ?poses) {
    String !descriptor;
    String !japaneseDescriptor;

    foreach (!descriptor : ?poses) {
        !japaneseDescriptor = tsc:getJapaneseVersionOfDescriptor(!descriptor);
        act:generateResponseFromString(!japaneseDescriptor);
    }
}