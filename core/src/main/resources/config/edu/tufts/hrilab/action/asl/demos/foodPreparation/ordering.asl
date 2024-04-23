import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;
import java.util.Map;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

//set up environment

() = setDeliveryPose(Symbol ?poseRefId) {
    Predicate !tmp;

    op:log(debug, "[setDeliveryPose] setting delivery pose to ?poseRefId");
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "deliveryPose(?poseRefId)");
    act:assertBelief(!tmp);

}

() = followMeBlocking() {
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Predicate !response;

    !bindings = act:askQuestionFromString(?actor,"Following", val(X));
    !response = op:get(!bindings, !x);
    act:generateResponseFromString("okay");
}

//TODO:brad: save pose , delete pose, save location, delete location

() = savePose(Symbol ?descriptor) {
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Symbol !location;
    edu.tufts.hrilab.fol.Symbol !equivalentPose;

    !bindings = act:askQuestionFromString(?actor,"What location is this pose reachable from?", location(X));
    !location = op:get(!bindings, !x);

    !bindings = act:askQuestionFromString(?actor,"What other pose does it overlap with?", pose(X));
    !equivalentPose = op:get(!bindings, !x);

    act:savePoseHelper(?actor, ?descriptor,!location,!equivalentPose);
}

() = saveLocation(Symbol ?descriptor) {
    op:log(debug, "[saveLocation] saving location ?descriptor");
    act:saveLocationHelper(?descriptor);
}


(Symbol ?locationRef) =saveLocationHelper["helper method to standardize interaction based item definition with asl based definition"](Symbol ?descriptor){
    ?locationRef= act:saveCurrentLocation(?descriptor);
}

(Symbol ?poseRef) =savePoseHelper["helper method to standardize interaction based item definition with asl based definition"](Symbol ?descriptor, String ?poseString, Symbol ?locationRef, Symbol ?equivalentPose=none){

    op:log(debug, "[savePoseHelper] saving pose ?descriptor accessible from ?locationRef with equivalent pose ?equivalentPose");

    Predicate !tmp;
    ?poseRef=act:addCameraPose(?descriptor, ?poseString);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(?actor, ?poseRef,?locationRef)");
    act:assertBelief(!tmp);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "overlapping(?poseRef,?poseRef)");
    act:assertBelief(!tmp);
    if (~op:equals(?equivalentPose, none)) {
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "overlapping(?poseRef,?equivalentPose)");
        act:assertBelief(!tmp);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "overlapping(?equivalentPose,?poseRef)");
        act:assertBelief(!tmp);
    }

}

//define ingredient
() = defineIngredient["defines a new ingredient and asks where it is located"](edu.tufts.hrilab.fol.Symbol ?descriptor){

    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Symbol !location;
    edu.tufts.hrilab.fol.Symbol !parentType;
    edu.tufts.hrilab.fol.Symbol !job;

    edu.tufts.hrilab.fol.Symbol !right= "right:yumi";
    edu.tufts.hrilab.fol.Symbol !graspPose= "rightpose_3:rightpose"; //TODO:which pose is this?

    //TODO:brad: right now you have to say "location" table 2
    !bindings = act:askQuestionFromString(?actor,"Where is it located?", pose(X));
    !location = op:get(!bindings, !x);

    !bindings = act:askQuestionFromString(?actor,"Okay, What type of ingredient is it?", type(X));
    !parentType = op:get(!bindings, !x);

    !bindings = act:askQuestionFromString(?actor,"Okay, what vision job is used to detect it?", job(X));
    !job = op:get(!bindings, !x);

    act:defineIngredientHelper(?descriptor,!location,!parentType,!job);

    //TODO:brad: teach grasp pose here
    !right.act:teachGraspPoint(?descriptor,!graspPose);

    act:generateResponseFromString("okay");

    op:sleep(2000);
    !right.act:goToCameraPose(!graspPose);
}

() =defineIngredientHelper["helper method to standardize interaction based item definition with asl based definition"](Symbol ?descriptor, Symbol ?location, Symbol ?parentType, Symbol ?job){

    Predicate !tmp;

    ?descriptor=act:addDetectionType(?descriptor,?job);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "observableAt(?descriptor,?location)");
    act:assertBelief(!tmp);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "typeof(?descriptor,?parentType)");
    //TODO:brad: is it valid to use ?job?
    act:addDetectionType(?parentType,?job);
    act:assertBelief(!tmp);
}


() = teachGraspPoint(Symbol ?type, Symbol ?pose) {
    Symbol !refId;
    !refId = act:positItem(?type);
    op:log(info, "posited item !refId");
    act:goToCameraPose(?pose);
    op:log(info, "perceiving entity for refId !refId of type ?type");
    act:teachGraspPointHelper(!refId, ?type);
    act:invalidateReference(!refId);
}

(Symbol ?refId) = positItem(Symbol ?itemType) {
    Term !props;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(?itemType(X:physobj))");
    op:log(info, "positing reference for type ?itemType with props !props");
    ?refId = act:positReference(!props);
}

() = teachGraspPointHelper(Symbol ?refId, Symbol ?itemType) {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    Symbol !equivalentPose;

    act:perceiveEntityFromSymbol(?refId);

    !bindings = act:askQuestionFromString(?actor,"I see a ?itemType, can you show me how to grasp it?", grasp(X));
    !response = op:get(!bindings, !x);
    if (~op:equalsValue(!response, none())) {
        act:defineGraspPointForDescriptor(?refId, ?itemType);
    } else {
        //todo: this is a bad failure justification
        exit(FAIL, not(found(graspPoint)));
    }

}
//() = removeIngredient["defines a new ingredient and asks where it is located"](edu.tufts.hrilab.fol.Symbol ?descriptor){
//
//    java.util.Map !bindings;
//    edu.tufts.hrilab.fol.Symbol !location;
//
//     //TODO:brad: implement this
//
//     //    //TODO:brad: right now you have to say "location" table 2
//     //    !bindings = act:askQuestionFromString(?actor,"Okay, where is it located?", location(X));
//     //    !location = op:get(!bindings, !x);
//     //
//     //    !bindings = act:askQuestionFromString(?actor,"Okay, what vision job is ued to detect it?", job(X));
//     //    !location = op:get(!bindings, !x);
//
//}

() = suspendDefineItem[""](edu.tufts.hrilab.fol.Predicate ?scriptID) {
    edu.tufts.hrilab.action.learning.ActionLearningStatus !learningStatus;
    edu.tufts.hrilab.action.learning.ActionLearningStatus !activeStatus = "ACTIVE";
    edu.tufts.hrilab.action.learning.ActionLearningStatus !pauseStatus = "PAUSE";

    !learningStatus = act:getLearningStatus();
    if (op:equals(!learningStatus,!activeStatus)) {
        act:updateActionLearning(?scriptID,!pauseStatus);
    }
}

() = resumeDefineItem[""](edu.tufts.hrilab.fol.Predicate ?scriptID) {
    edu.tufts.hrilab.action.learning.ActionLearningStatus !learningStatus;
    edu.tufts.hrilab.action.learning.ActionLearningStatus !resumeStatus = "RESUME";
    edu.tufts.hrilab.action.learning.ActionLearningStatus !pauseStatus = "NONE"; //Why is it set to NONE and not kept as PAUSE? Will cause an error log if we actually never started action learning before suspending

    !learningStatus = act:getLearningStatus();
    if (op:equals(!learningStatus,!pauseStatus)) {
        act:updateActionLearning(?scriptID,!resumeStatus);
    }
}

() = cancelDefineItem[""](edu.tufts.hrilab.fol.Symbol ?itemRefId, edu.tufts.hrilab.fol.Symbol ?trayRefId, edu.tufts.hrilab.fol.Predicate ?scriptID) {
    edu.tufts.hrilab.action.learning.ActionLearningStatus !learningStatus;
    edu.tufts.hrilab.action.learning.ActionLearningStatus !activeStatus = "ACTIVE";
    edu.tufts.hrilab.action.learning.ActionLearningStatus !cancelStatus = "CANCEL";

    if (~op:isNull(?itemRefId)) {
       act:invalidateReference(?itemRefId);
    }
    if (~op:isNull(?trayRefId)) {
       act:invalidateReference(?trayRefId);
    }

    //Technically don't need to pass in scriptID
    !learningStatus = act:getLearningStatus();
    if (op:equals(!learningStatus,!activeStatus)) {
        act:updateActionLearning(?scriptID,!cancelStatus);
    }
}

//Item
() = defineItem[""](edu.tufts.hrilab.fol.Symbol ?itemName){

    onInterrupt: {
      suspend: tsc:suspendDefineItem(!scriptID);
      cancel: tsc:cancelDefineItem(!itemRefId, !trayRefId, !scriptID);
      resume: tsc:resumeDefineItem(!scriptID);
    }

    java.lang.Integer !i =0;

    java.util.Map !bindings;
    java.util.List !bindingArgs;
    edu.tufts.hrilab.fol.Variable !x = "X";

    edu.tufts.hrilab.fol.Term !optionsCountQuery= "val(X)";
    edu.tufts.hrilab.fol.Term !options;
    java.lang.Integer !optionsCount= 0;
    edu.tufts.hrilab.fol.Term !optionsQuery= "option(X)";
    edu.tufts.hrilab.fol.Symbol !optionSymbol;
    java.util.List !optionsArgs;
    edu.tufts.hrilab.fol.Term !endQuery= "want(A,endItemDefinition(Y:agent,X:property))";
    edu.tufts.hrilab.fol.Term !props;
    edu.tufts.hrilab.fol.Symbol !trayRefId;
    edu.tufts.hrilab.fol.Symbol !itemRefId;
    edu.tufts.hrilab.fol.Predicate !tmpPredicate;

    edu.tufts.hrilab.fol.Predicate !scriptID;
    edu.tufts.hrilab.fol.Symbol !startSymbol ="start";
    edu.tufts.hrilab.fol.Symbol !endSymbol= "end";

    java.util.List !signatureArgs;
    !signatureArgs = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    op:add(!signatureArgs,?actor);

    ?itemName=act:addItem(?itemName);
    java.lang.String !signatureName= "prepare?itemName";

    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(?itemName(X:item))");
    !itemRefId = act:positReference(!props);
    op:add(!signatureArgs,!itemRefId);

//    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(servingBox(X:physobj))");
//    !trayRefId = act:positReference(!props);
//    op:add(!signatureArgs,!trayRefId);

    !bindings = act:askQuestionFromString(?actor,"How many options does a ?itemName have ?", !optionsCountQuery);
    !optionsCount = op:get(!bindings, !x);
    op:log(debug,"[defineItem] number options: !optionsCount");
//    !optionsCount = op:getArg(!options,0);
//    op:log(debug,"[defineItem] number options count: !optionsCount");

    //if(op:gt(!optionsCount,0)){
    while(op:gt(!optionsCount,!i)){
    //TODO:brad: put this back
        !bindings = act:askQuestionFromString(?actor,"What is option !i", !optionsQuery);
//        !bindings = act:askQuestionFromString(?actor,"What is option 1", !optionsQuery);
        !optionSymbol = op:get(!bindings, !x);
        op:log(debug,"[defineItem] option symbol: !optionSymbol ");
        op:add(!signatureArgs,!optionSymbol);
        op:log(debug,"[defineItem] updated signature args: !signatureArgs ");
        !i =op:++(!i);
    }
    //TODO:check that the number options matches what was said

    (!scriptID) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !signatureName,!signatureArgs);

    act:updateActionLearning(!scriptID,!startSymbol);
    !bindings = act:askQuestionFromString(?actor,"How do I prepare a ?itemName ?",!endQuery);

    //generate new signature that is unique to model name
    op:log(warn, "[defineItem] ending learning for: !scriptID");
    act:updateActionLearning(!scriptID,!endSymbol);

//    !tmpPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "insert(propertyof(!itemRefId, ?itemName:property))");
//    act:modifyAction(!scriptID, !tmpPredicate, condition(pre));

//    !tmpPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "insert(propertyof!trayRefId, servingBox:property))");
//    act:modifyAction(!scriptID, !tmpPredicate, condition(pre));

    !tmpPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "insert(prepared(!itemRefId))");
    act:modifyAction(!scriptID, !tmpPredicate, effect(success));
//    !tmpPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "insert(itemOn(!itemRefId, !trayRefId))");
//    act:modifyAction(!scriptID, !tmpPredicate, effect(success));

    //invalidating the references for the items we had to posit earlier (so that we actually used the args to the action in the body)
//    act:invalidateReference(!trayRefId);
    act:invalidateReference(!itemRefId);


    act:generateResponseFromString("okay");
}

(Predicate ?signature) = getPrepareSignatureForItem(Symbol ?itemType) {
    Symbol !actionName;
    java.util.List !signatures;
    Predicate !query;
    java.util.List !bindings;
    Map !binding;
    Symbol !superType;
    String !superTypeName;
    String !itemTypeName;

    !itemTypeName = op:getName(?itemType);
    ?itemType = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !itemTypeName);

    op:log(debug, "[getPrepareSignatureForItem] checking for an action signature with item type ?itemType");

    !actionName = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "prepare?itemType");

    ?signature = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "none()");

    !signatures = act:getActionSignaturesForName(!actionName);
    if (~op:isEmpty(!signatures)) {
        ?signature = op:get(!signatures,0);
        op:log(debug, "[getPrepareSignatureForItem] at least one action signature with name !actionName found: ?signature");
    } else {
        op:log(debug, "[getPrepareSignatureForItem] no actions matching name !actionName found, searching for type of item instead");
        !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "typeof(!itemTypeName, !x)");
        !bindings = act:queryBelief(!query);
        if (~op:isEmpty(!bindings)) {
            !binding = op:get(!bindings, 0);
            !superType = op:get(!binding, !x);
            !superTypeName = op:invokeMethod(!superType, "getName");
            !superType = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !superTypeName);
            op:log(debug, "[getPrepareSignatureForItem] item type !itemTypeName has a parent type !superType");
            !actionName = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "prepare!superType");
            !signatures = act:getActionSignaturesForName(!actionName);
            if (~op:isEmpty(!signatures)) {
                ?signature = op:get(!signatures,0);
                op:log(debug, "[getPrepareSignatureForItem] at least one action signature with name !actionName found: ?signature");
            }
        }
    }
}

(List ?optionsList) = getListFromOptionsPredicate(Predicate ?options) {

    List !args;
    Symbol !firstArg;
    String !firstArgName;
    java.util.List !firstArgArgs;

    !args = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    ?optionsList = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    op:log(debug, "[getListFromOptionsPredicate] getting list from predicate: ?options");

    //sloppy var reuse so we don't need to declare a new one
    if (~op:equalsValue(?options, none())) {
        !args = op:getArgs(?options);
        !firstArg= op:get(!args,0);
        !firstArgName= op:getName(!firstArg);
        if(op:equals(!firstArgName,"and")){
            !firstArgArgs= op:getArgs(!firstArg);
            op:addAll(?optionsList,!firstArgArgs);
            op:log(debug, "[getListFromOptionsPredicate] added !firstArgArgs to optionsList ?optionsList because firstArgName: !firstArgName is 'and' -> multiple option");
        }
        else {
            op:add(?optionsList,!firstArg);
            op:log(debug, "[getListFromOptionsPredicate] added !firstArg to optionsList ?optionsList because firstArgName: !firstArgName is not 'and' -> single option");
        }
    }
}

//?options of the form options(X,Y,....)
(Symbol ?itemRefId) = prepare["prepares Item for ?itemType"](Symbol ?itemType, Predicate ?options="none()", Symbol ?trayRefId="none"){

    String !itemTypeName;
    Symbol !itemTypeNameSymbol;
    Predicate !goalState;
    Term !props;
    java.lang.Long !gid;
    java.util.List !signatures;
    Predicate !signature;
    java.util.List !args;
    java.lang.Integer !length;
    Symbol !possibleActionName;
    Variable !x = "X";
    Predicate !query;
    java.util.List !bindings;
    Map !binding;
    Symbol !superType;
    String !superTypeName;
    Predicate !tmp;
    java.lang.Integer !expectedLength;
    java.lang.String !possibleActionNameString;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.justification.Justification !justification;

    java.util.List !signatureArgs;

    java.util.List !optionsArgs;
    java.lang.Integer !optionsLength;
    java.lang.Integer !i = 1;
    Symbol !option;
    String !signatureName;
    String !signatureNameSymbol;

    java.util.Map !questionBindings;

    ?itemRefId = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");

    !signatureArgs = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    op:log(debug,"[prepare] item type: ?itemType with options: ?options, trayRefId: ?trayRefId");

//    //Posit tray ref id if not passed in.
//    if (op:equalsValue(?trayRefId, none)) {
//       op:log(debug, "[prepareSingleItem] positing reference for tray");
//       !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(servingBox(X:physobj))");
//       ?trayRefId = act:positReference(!props);
//    }

    !itemTypeName = op:getName(?itemType);
    op:log(debug, "itemTypeName: !itemTypeName");
    !itemTypeNameSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !itemTypeName);

    op:add(!signatureArgs,?actor);

    !possibleActionName = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "prepare!itemTypeName");
    !signature = act:getPrepareSignatureForItem(!itemTypeName);
    !signatureName = op:getName(!signature);
    !signatureNameSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !signatureName);
    if (op:equalsValue(!signatureNameSymbol, !possibleActionName)) {
        !expectedLength = op:newObject("java.lang.Integer", 2);
        op:log(debug, "positing reference for !itemTypeName");
        !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(!itemTypeName(X:item))");
        ?itemRefId = act:positReference(!props);
        op:add(!signatureArgs,?itemRefId);
//        op:add(!signatureArgs,?trayRefId);
    } else {
        !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "typeof(!itemTypeName, !x)");
        !bindings = act:queryBelief(!query);
        if (~op:isEmpty(!bindings)) {
            !binding = op:get(!bindings, 0);
            !superType = op:get(!binding, !x);
            !superTypeName = op:invokeMethod(!superType, "getName");
            !superType = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !superTypeName);
            op:log(debug, "[prepare] item type !itemTypeName has a parent type !superType");
            op:log(debug, "positing reference for !superType");
            !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(!superType(X:item))");
            ?itemRefId = act:positReference(!props);
            op:add(!signatureArgs,?itemRefId);
//            op:add(!signatureArgs,?trayRefId);
        }
        !possibleActionName = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "prepare!superType");
        if (~op:equalsValue(!signature, none())) {
            !expectedLength = op:newObject("java.lang.Integer", 3);
            op:add(!signatureArgs, !itemTypeNameSymbol);
        } else {
            op:log(error, "[prepare] unable to find prepare action matching item super type !superType");
        }
    }

    if (op:equalsValue(!signature, none())) {
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "prepare!itemTypeName");
        exit(FAIL, not(found(!tmp)));
    } else {
        //assemble the goal submission
        !args = op:invokeMethod(!signature, "getArgs");
        !length = op:invokeMethod(!args, "size");

        op:log(debug, "[prepare] length of signature args: !signature, args: !args is !length and expected length is !expectedLength");
        if (op:gt(!length, !expectedLength)) {
            op:log(debug, "[prepare] options provided are ?options");
            if (~op:equalsValue(?options, none())) {
            //sloppy var reuse so we don't need to declare a new one
                !args = act:getListFromOptionsPredicate(?options);
                op:addAll(!signatureArgs,!args);
                !expectedLength = op:invokeMethod(!signatureArgs, "size");
                if(~op:equals(!length, !expectedLength)) {
                    op:log(error, "[prepare] additional options required for prepare ?itemType. Only ?options provided, but signature is !signature");
                    exit(FAIL);
                }
            } else {
                //how many options do we need?
                !optionsLength = op:-(!length, !expectedLength);
                while(op:le(!i, !optionsLength)) {
                    //todo: better question asking language
                    !questionBindings = act:askQuestionFromString(?actor,"What is option !i of !optionsLength?", option(X));
                    //TODO:brad: undo this
//                    !questionBindings = act:askQuestion(?actor,"What type of sauce", option(X));
                    !option = op:get(!questionBindings, !x);
                    op:add(!signatureArgs, !option);
                    !i = op:++(!i);
                }
            }
        }

        !possibleActionNameString = op:invokeMethod(!possibleActionName, "getName");
        op:log(warn, "[prepare] possibleActionName: !possibleActionName, possibleActionNameString: !possibleActionNameString, signatureArgs: !signatureArgs");
        (!goalState) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !possibleActionNameString, !signatureArgs);
        op:log(debug,"[prepare ] goal state: !goalState");
        goal:!goalState;
//        !gid = act:submitGoalDirectly(!goalState);
//        !goalStatus = act:joinOnGoal(!gid);
        //What about cancel case?
//        if (op:invokeMethod(!goalStatus, "isFailure")) {
//            !justification = act:getGoalFailConditions(!gid);
//            exit(FAIL, !justification);
//        }


    act:generateResponseFromString("!itemTypeName is complete");
    }
}

//define meal

() = defineMeal["defines a new meal and gathers information about the types of items which compose the meal"]() {

    java.util.Map !bindings;
    Predicate !itemType;
    Predicate !mealPred;
    Predicate !componentsPred;
    java.lang.String !itemTypeString;
    Symbol !itemTypeSymbol;
    Variable !x = "X";
    java.util.List !itemTypes;
    !itemTypes = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    Symbol !mealID;
    Predicate !query;
    java.util.List !queryBindings;
    java.lang.Integer !size = 0;

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "meal(X,Y)");
    op:log(debug, "[definteMeal] query: !query");
    !queryBindings = act:queryBelief(!query);
    op:log(debug, "[definteMeal] queryBindings: !queryBindings");
    !size = op:invokeMethod(!queryBindings, "size");
    op:log(debug, "[definteMeal] size: !size");
    !mealID = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "meal!size");


    !bindings = act:askQuestionFromString(?actor,"It will be called !mealID . What type of items are in it?", item(X));
    !itemType = op:get(!bindings, !x);

    while(~op:equalsValue(!itemType, none())) {
        !itemTypeString = op:invokeMethod(!itemType, "getName");
        !itemTypeSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !itemTypeString);
        op:add(!itemTypes, !itemTypeSymbol);
        op:log(debug, "[defineMeal] got item type !itemType");
        !bindings = act:askQuestionFromString(?actor,"Okay. What else is included?", item(X));
        !itemType = op:get(!bindings, !x);
    }
    op:log(debug, "[defineMeal] finished getting item types !itemTypes");

    !componentsPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "components", !itemTypes);
    !mealPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "meal(!mealID, !componentsPred)");
    act:assertBelief(!mealPred);
    op:log(debug,"[defineMeal] meal definition predicate !mealPred");
}

() = orderMeal[""](Symbol ?mealType, Symbol ?options="none()") {
    Variable !x= "X";
    Predicate !queryPred;
    java.util.List !bindings;
    java.util.Map !answerBindings;
    Symbol !drinkRef;
    Map !binding;
    Predicate !containsPred;
    Symbol !itemType;
    java.util.List !itemTypes;
    java.util.List !items;
    Symbol !item;
    Symbol !mealID;
    Predicate !none= "none()";
    Symbol !itemRefId;

    !mealID = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "meal?mealType");

    Symbol !trayRefId;
    Term !props;


    List !itemOptionList;
    List !mealOptionList;
    List !signatureArgsList;
    Predicate !itemOptions;
    Integer !optionsListSize = 0;

    Predicate !actionSignature;
    Integer !numOfExpectedOptions;
    List !mealItemAndOptionsPredicateList;
    Predicate !tmp;
    Predicate !mealOrderPredicate;
    Predicate !prepareMealPredicate;

    java.util.Map !questionBindings;
    Symbol !option;

    java.lang.Integer !i = 1;

    !mealItemAndOptionsPredicateList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");

    !items = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    !itemOptionList = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    !mealOptionList = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    !signatureArgsList = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    (!queryPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "meal(!mealID, X)");
    !bindings = act:queryBelief(!queryPred);
    if(~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !containsPred = op:get(!binding, !x);
        op:log(debug, "[orderMeal] meal !mealID defined with contains: !containsPred");
        !itemTypes = op:getArgs(!containsPred);
    } else {
        op:log(error, "[orderMeal] empty bindings for meal query !queryPred");
        exit(FAIL, not(know(meal(!mealID))));
    }

    op:addAll(!items,!itemTypes);

    //TODO:sandwich?
//    foreach (!itemType : !itemTypes) {
//        !answerBindings = act:askQuestionFromString(?actor,"What !itemType do you want?", val(X));
//        !item = op:get(!answerBindings, !x);
//        op:log(debug, "[orderMeal] got response !item for item type !itemType");
//        //todo: type checking
//        if (op:isNull(!item)) {
//            op:log(error, "[orderMeal] no bindings retrieved when asking for specific item corresponding to !itemType");
//            exit(FAIL); //todo: should this have a failure justification?
//        }
//        op:add(!items, !item);
//    }

    //posit the tray reference
    op:log(debug, "[orderMeal] positing reference for tray");
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tray(X:physobj))");
    !trayRefId = act:positReference(!props);

    !mealOptionList = act:getListFromOptionsPredicate(?options);
    //submit the item prepare actions
    foreach (!item : !items) {
        !actionSignature = act:getPrepareSignatureForItem(!item);
        if (op:equalsValue(!actionSignature, none())) {
            op:log(error, "[orderMeal] can't match action signature expected for item !item");
            exit(FAIL);
        }

        !itemOptions = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "none");

        !signatureArgsList = op:getArgs(!actionSignature);
        !numOfExpectedOptions = op:invokeMethod(!signatureArgsList, "size");
        !numOfExpectedOptions = op:-(!numOfExpectedOptions, 3); //?actor, ?itemRef, ?trayRef should always be there
        if (op:gt(!numOfExpectedOptions, 0)) {
        //not an obvious way to handle partially specified options. we're handling either fully specified or completely unspecified
        //doing this so we don't have a partially-implemented version of partial-specification where the under-specified item parameterizations have to be at the end of the list of items.
            if (~op:equalsValue(?options, none())) {
                !optionsListSize = op:invokeMethod(!mealOptionList, "size");
                //todo: make sure there's enough options provided!
                if (op:lt(!optionsListSize, !numOfExpectedOptions)) {
                    op:log(error, "[orderMeal] number of expected options for item !item exceeds the remaining options provided for meal: !mealOptionList");
                    exit(FAIL);
                }
                op:log(debug, "[orderMeal] remaining meal options are !mealOptionList");
                !itemOptionList = op:subList(!mealOptionList, 0, !numOfExpectedOptions);
                op:log(debug, "[orderMeal] item options for item !item are !itemOptionList");
                !mealOptionList = op:subList(!mealOptionList, !numOfExpectedOptions, !optionsListSize);

                !itemOptions = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and", !itemOptionList);
                !itemOptions = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "options", !itemOptions);
            } else {
            //we have to ask for options!
                //how many options do we need?
                while(op:le(!i, !numOfExpectedOptions)) {
                    //todo: better question asking language
                    !questionBindings = act:askQuestionFromString(?actor,"What is option !i of !numOfExpectedOptions?", option(X));
                    !option = op:get(!questionBindings, !x);
                    op:add(!itemOptionList, !option);
                    !i = op:++(!i);
                }
                !itemOptions = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and", !itemOptionList);
                !itemOptions = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "options", !itemOptions);
            }
        }
        op:log(warn, "[orderMeal] preparing item !item");
        //!none is the default value of options, might want to change imp
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "item(!item, !itemOptions)");
        op:add(!mealItemAndOptionsPredicateList, !tmp);
    }

//    !answerBindings = act:askQuestionFromString(?actor,"What drink would you like", val(X));
//    !drinkRef = op:get(!answerBindings, !x);

    act:generateResponseFromString("okay");

    !mealOrderPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "meal", !mealItemAndOptionsPredicateList);

    !prepareMealPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "prepareMeal", ?actor, !mealOrderPredicate, !trayRefId, !drinkRef, !mealID);
    act:submitGoal(!prepareMealPredicate);
}

() = prepareMeal["prepare a whole meal"](Predicate ?mealOrderPredicate, Symbol ?trayRefId, Symbol ?drinkRefId, Symbol ?mealID) {
    Predicate !readyPred;
    List !refIds;
    List !args;
    List !itemList;
    Predicate !itemInfo;
    Symbol !item;
    Predicate !options;
    Symbol !itemRefId;
    Symbol !deliveryPose= "mobileyumipose_2:pose";
    Variable !x = "X";

    Predicate !query;
    java.util.List !bindings;
    Map !binding;

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "deliveryPose(!x)");
    !bindings = act:queryBelief(!query);
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !deliveryPose = op:get(!binding, !x);
    }

    op:log(debug, "[prepareMeal] preparing meal ?mealOrderPredicate on tray ?trayRefId with drink ?drinkRefId");

    !itemList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !args = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    !refIds = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    !itemList = op:getArgs(?mealOrderPredicate);
    foreach (!itemInfo : !itemList) {
        !args = op:getArgs(!itemInfo);
        !item = op:get(!args, 0);
        !options = op:get(!args, 1);
        !itemRefId = act:prepare(!item,!options);
        op:add(!refIds, !itemRefId);
    }

//    if(~op:equalsValue(?drinkRefId, none)) {
//        act:getOn(?drinkRefId,?trayRefId);
//        op:add(!refIds, ?drinkRefId);
//    }

    //TODO:brad:specify this pose somehow? perhaps in meal definition
//    goal:itemAt(?trayRefId,!deliveryPose);

    //mealReady(!trayRefId,items(!items));
    !readyPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "items", !refIds);
    !readyPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "mealReady",?trayRefId,!readyPred);
    op:log(warn, "[orderMeal] prepared meal: ?mealID: !readyPred");
    act:assertBelief(!readyPred);
}


///////////////////////////////////////Reference scripts from assembly demo///////////////////////////////

//interactive version where it asks questions about the differences
() = defineItemByAnalogy["creates a new prepare script based on an existing one"](edu.tufts.hrilab.fol.Symbol ?newItem){

  Symbol !oldItem;
  edu.tufts.hrilab.fol.Predicate !oldScriptGoal;
  java.util.List !oldGoalArgs;
  edu.tufts.hrilab.fol.Predicate !newScriptGoal;
  edu.tufts.hrilab.fol.Predicate !likeGoal;
  edu.tufts.hrilab.fol.Predicate !modLocation;
  edu.tufts.hrilab.fol.Predicate !objectPred;
  edu.tufts.hrilab.fol.Predicate !location;

  java.util.Map !bindings;
  java.util.List !bindingArgs;
  edu.tufts.hrilab.fol.Variable !x = "X";
  edu.tufts.hrilab.fol.Variable !y = "Y";
  edu.tufts.hrilab.fol.Predicate !modification;

  java.lang.String !modName;

  edu.tufts.hrilab.fol.Symbol !speaker;

   //TODO:brad: do we still need to do this?
  (!speaker) = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

  ?newItem=act:addItem(?newItem);

  !bindings = act:askQuestionFromString(?actor,"What item is it like?", item(X));
  !oldItem = op:get(!bindings, !x);
  !oldScriptGoal = act:getPrepareSignatureForItem(!oldItem);
  op:log(debug, "osg !oldScriptGoal");
  !oldGoalArgs = op:getArgs(!oldScriptGoal);
  op:set(!oldGoalArgs,0,!speaker);
  op:log(debug, "old goal args !oldGoalArgs");

  !newScriptGoal =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "prepare?newItem", !oldGoalArgs);
  op:log(debug, "nsg !newScriptGoal");

  !likeGoal =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "like", !newScriptGoal,!oldScriptGoal);
  op:log(debug, "lg !likeGoal");

  !bindings = act:askQuestionFromString(!speaker,"okay. what are the differences", mod(X,Y));
  !modification = op:get(!bindings, !x);
  !location = op:get(!bindings, !y);
  op:log(debug, "modification !modification location !location");
  act:modifyAction(!likeGoal,!modification,!location);

  !bindings = act:askQuestionFromString(!speaker,"okay. are there any more differences?", mod(X,Y));
  !modification = op:get(!bindings, !x);
    !location = op:get(!bindings, !y);
  !modName = op:getName(!modification);
  while(op:!=(!modName,"none")){
    op:log(debug, "modification !modification location !location");
    act:modifyAction(!newScriptGoal,!modification,!location);
    !bindings = act:askQuestionFromString(!speaker,"okay. are there any more differences?", mod(X,Y));
    !modification = op:get(!bindings, !x);
    !location = op:get(!bindings, !y);
    !modName = op:getName(!modification);
  }

  act:generateResponseFromString("okay");
}

() = modifyPrepare["modifies an existing prepare script"](edu.tufts.hrilab.fol.Symbol ?item){

  edu.tufts.hrilab.fol.Predicate !scriptGoal;
  java.util.List !goalArgs;
  edu.tufts.hrilab.fol.Predicate !modLocation;
  edu.tufts.hrilab.fol.Predicate !objectPred;

  java.util.Map !bindings;
  java.util.List !bindingArgs;
  edu.tufts.hrilab.fol.Variable !x = "X";
  edu.tufts.hrilab.fol.Variable !y = "Y";

  edu.tufts.hrilab.fol.Predicate !modification;
  java.lang.String !modName;
  edu.tufts.hrilab.fol.Predicate !location;

  edu.tufts.hrilab.fol.Symbol !speaker;

  String !scriptName;

   //TODO:brad: do we still need to do this?
  !speaker = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

  !scriptGoal = act:getPrepareSignatureForItem(?item);
  !goalArgs = op:getArgs(!scriptGoal);
  !scriptName = op:getName(!scriptGoal);
  op:set(!goalArgs,0,!speaker);
  op:log(debug, "goal args !goalArgs");

  !scriptGoal =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", !scriptName, !goalArgs);
  op:log(debug, "goal !scriptGoal");

  !bindings = act:askQuestionFromString(!speaker,"okay what are the changes", mod(X,Y));
  !modification = op:get(!bindings, !x);
  !location = op:get(!bindings, !y);
  !modName = op:getName(!modification);
  while(op:!=(!modName,"none")){
    op:log(debug, "[modifyPrepare] modification !modification location !location");
    act:modifyAction(!scriptGoal,!modification,!location);
    !bindings = act:askQuestionFromString(!speaker,"okay are there any more changes", mod(X,Y));
    !modification = op:get(!bindings, !x);
    !location = op:get(!bindings, !y);
    !modName = op:getName(!modification);
  }

  act:generateResponseFromString("okay");
}
