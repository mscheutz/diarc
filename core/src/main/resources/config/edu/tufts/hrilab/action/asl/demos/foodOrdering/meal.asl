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