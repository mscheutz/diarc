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
    !bindings = act:askQuestionFromString(?actor, "視野に複数の物体が入っています。どれを拾ったらいいでしょうか？", objectType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings,!x);
        act:assertIsPhysobj(?object);
    }
}

() = reportNotPickingUpObject() {
    act:generateResponseFromString("承知しました。物体は拾いません");
}

//TODO: Either make parallel pipeline or update existing one to handle a list of response formats (referred pipeline being askQuestion variations)
//      Don't want to do now for fear of breaking things. Writing just this method strangely for now.
(Symbol ?response) = reportFullLocationAndAskToSwapV2() {
    Map !bindings;
    Variable !x = X;
    Variable !y = Y;
    Predicate !yesNoResponseForm = val(X);
    Predicate !objectTypeResponseForm = objectType(Y);
    java.util.List !responseFormList;

    edu.tufts.hrilab.slug.common.Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?actor);
    edu.tufts.hrilab.slug.common.UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setWords", "場所がいっぱいになっているため、物体を置くことができません。物体のどれかを交換しましょうか？");

    op:log(debug,"[reportFullLocationAndAskToSwapV2] asking question");
    async {
      act:communicateUtterance(!utterance);
    }

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    op:add(!responseFormList,!yesNoResponseForm);
    op:add(!responseFormList,!objectTypeResponseForm);

    op:log(debug,"[reportFullLocationAndAskToSwapV2] waiting for one of two response forms");
    !bindings = act:waitForOneOfResponses(!responseFormList, ?actor, -1);
    if (op:invokeMethod(!bindings, "isEmpty")) {
        op:log(debug,"[reportFullLocationAndAskToSwapV2] got empty bindings, answering 'no'");
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        op:log(debug,"[reportFullLocationAndAskToSwapV2] have bindings !bindings");
        ?response = op:invokeMethod(!bindings,"get",!x);
        op:log(debug,"[reportFullLocationAndAskToSwapV2] X binding value ?response");
        if (op:isNull(?response)) {
            ?response = op:get(!bindings,!y);
            op:log(debug,"[reportFullLocationAndAskToSwapV2] Y binding value ?response");
            if (op:isNull(?response)) {
                op:log(warn,"[reportFullLocationAndAskToSwapV2] No binding found for X or Y, this shouldn't be the case");
                ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
            } else {
                if (~op:equalsValue(?response,none)) {
                    act:assertIsPhysobj(?response);
                }
            }
        }
    }
}

(Symbol ?response) = reportFullLocationAndAskToSwap() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"場所がいっぱいになっているため、物体を置くことができません。物体のどれかを交換しましょうか？", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?object) = askForObjectToSwap() {
    Map !bindings;
    Variable !x = "X";

    !bindings = act:askQuestionFromString(?actor,"どの物体を交換したいいでしょうか？", objectType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings, !x);
        act:assertIsPhysobj(?object);
    }
}

() = reportNotSwappingObjects() {
    act:generateResponseFromString("承知しました。物体を交換しません");
}

() = reportHoldingObject(Symbol ?descriptor) {
    String !objectDescriptorJapanese;

    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!objectDescriptorJapaneseを持っています。");
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
    act:generateResponseFromString("物体はどれも場所がわかりません");
}

() = reportObjectExists(Symbol ?descriptor) {
    String !objectDescriptorJapanese;

    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!objectDescriptorJapaneseがあります");
}

() = reportNoObjectsAtPose(Symbol ?descriptor) {
    String !poseDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);
    act:generateResponseFromString("!poseDescriptorJapaneseに物体はありません");
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
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?armPose);

   act:generateResponseFromString("現在、!poseDescriptorJapaneseの状態です");
}

() = reportObjectToSwapNotAtCurrentPose() {
   act:generateResponseFromString("ご指定の物体は現在の位置にありません");
}

() = reportObjectsToSwapAreTheSame() {
   act:generateResponseFromString("物体をその物体自体と交換するように依頼されたため、何もしません");
}

() = reportDetectedObject(Symbol ?objectDescriptor) {
    String !objectDescriptorJapanese;
    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?objectDescriptor);

    act:generateResponseFromString("はい、!objectDescriptorJapaneseが見えます");
}

() = reportCantVerifyBecauseActiveGoal() {
    act:generateResponseFromString("現在作業中ですので、それを確認することができません");
}

() = reportObjectNotBelievedToBeAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    String !objectDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);
    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?objectDescriptor);

    act:generateResponseFromString("!objectDescriptorJapaneseは!poseDescriptorJapaneseにあると思いません");
}

() = reportObjectBelievedToBeAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    String !objectDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);
    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?objectDescriptor);

    act:generateResponseFromString("!objectDescriptorJapaneseは!poseDescriptorJapaneseにあると思います");
}

() = reportObjectNotAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    String !objectDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);
    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?objectDescriptor);

    act:generateResponseFromString("!poseDescriptorJapaneseには!objectDescriptorJapaneseは見えません");
}

() = reportDetectedObject(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    String !objectDescriptorJapanese;

    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);
    !objectDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?objectDescriptor);

    act:generateResponseFromString("!poseDescriptorJapaneseに!objectDescriptorJapaneseが見えます");
}

(Symbol ?response) = askToVerifyBelief() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"確認しましょうか?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

() = reportDontBelieveObjectsAtPose(Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);

    act:generateResponseFromString("!poseDescriptorJapaneseに物体はないと思います");
}

() = reportBelieveObjectsAtPose(Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);

    act:generateResponseFromString("!poseDescriptorJapaneseに物体はあると思います");
}

() = reportObjectsNotAtPose(Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);

    act:generateResponseFromString("!poseDescriptorJapaneseに物体はひとつも見えません");
}

() = reportDetectedObjects(Symbol ?poseDescriptor) {
    String !poseDescriptorJapanese;
    !poseDescriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?poseDescriptor);

    act:generateResponseFromString("!poseDescriptorJapaneseに物体が見えます");
}

//TODO: Either make parallel pipeline or update existing one to handle a list of response formats (referred pipeline being askQuestion variations)
//      Don't want to do now for fear of breaking things. Writing just this method strangely for now.
(Symbol ?response) = reportHoldingObjectAndAskToPutDownV2() {
    Map !bindings;
    Variable !x = X;
    Variable !y = Y;
    Predicate !yesNoResponseForm = val(X);
    Predicate !objectTypeResponseForm = pose(Y);
    java.util.List !responseFormList;

    edu.tufts.hrilab.slug.common.Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?actor);
    edu.tufts.hrilab.slug.common.UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setWords", "すでに物体を持っているため、拾うことはできません。どちらかに置きましょうか?");

    op:log(debug,"[reportHoldingObjectAndAskToPutDownV2] asking question");
    async {
      act:communicateUtterance(!utterance);
    }

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    op:add(!responseFormList,!yesNoResponseForm);
    op:add(!responseFormList,!objectTypeResponseForm);

    op:log(debug,"[reportHoldingObjectAndAskToPutDownV2] waiting for one of two response forms");
    !bindings = act:waitForOneOfResponses(!responseFormList, ?actor, -1);
    if (op:invokeMethod(!bindings, "isEmpty")) {
        op:log(debug,"[reportHoldingObjectAndAskToPutDownV2] got empty bindings, answering 'no'");
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        op:log(debug,"[reportHoldingObjectAndAskToPutDownV2] have bindings !bindings");
        ?response = op:invokeMethod(!bindings,"get",!x);
        op:log(debug,"[reportHoldingObjectAndAskToPutDownV2] X binding value ?response");
        if (op:isNull(?response)) {
            ?response = op:get(!bindings,!y);
            op:log(debug,"[reportHoldingObjectAndAskToPutDownV2] Y binding value ?response");
            if (op:isNull(?response)) {
                op:log(warn,"[reportHoldingObjectAndAskToPutDownV2] No binding found for X or Y, this shouldn't be the case");
                ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
            } else {
                if (~op:equalsValue(?response,none)) {
                    act:assertIsPose(?response);
                }
            }
        }
    }
}

(Symbol ?response) = reportHoldingObjectAndAskToPutDown() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"すでに物体を持っているため、拾うことはできません。どちらかに置きましょうか?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?objectPose) = askForPose() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"どちらに置きましょうか?", pose(X));
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

() = reportSeenObject(Symbol ?descriptor) {
    String !descriptorJapanese;
    !descriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);

    act:generateResponseFromString("!descriptorJapaneseが見えます");
}

() = reportObjectsAtSameLocation() {
    act:generateResponseFromString("物体は同じポーズをとっている");
}

(Symbol ?objectPose) = reportFailedDetectionAndAskForObjectPose(Symbol ?descriptor) {
    Map !bindings;
    Variable !x = "X";
    String !descriptorJapanese;
    !descriptorJapanese = tsc:getJapaneseVersionOfDescriptor(?descriptor);

    (!bindings) = act:askQuestionFromString(?actor,"!descriptorJapaneseが見えません, 場所何番にありますか?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?objectPose = op:get(!bindings,!x);
        act:assertIsPose(?objectPose);
    }
}

(String ?name) = askForTaskName() {
    Symbol !response;
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"課題の呼び方は何と言いますか?", taskName(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?name = op:setNull();
    } else {
        !response = op:get(!bindings,!x);
        ?name = op:getName(!response);
    }
}

() = reportActionLearningStart(String ?name) {
    //TODO: can turn this into a question (just for utteranceType, not expecting an answer through the normal means)
    Predicate !pred;
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "how(?actor,perform(?name))");
    op:log(info,"!pred");
    act:generateResponse(!pred);

    //act:generateResponseFromString("?nameの実行方法を教えてください");
}

() = reportActionLearned(String ?name) {
    Predicate !pred;
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knowHowTo(?actor,perform(?name))");
    op:log(info,"!pred");
    act:generateResponse(!pred);

    //act:generateResponseFromString("?nameは実行できる");
}

() = reportNoLearningState() {
    act:generateResponseFromString("まだ私に教え始めていない");
}