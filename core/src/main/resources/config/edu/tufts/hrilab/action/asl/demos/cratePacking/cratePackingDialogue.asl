import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.util.Map;
import java.util.List;
import java.lang.Integer;
import java.lang.Boolean;

() = packItemsIntoCrates() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    Boolean !instructedDirectly = false;

    Integer !crateL;
    Integer !crateW;
    Integer !crateH;
    Integer !itemL;
    Integer !itemW;
    Integer !itemH;
    Integer !numItems;
    Integer !updatedNumItems;

    !crateL = tsc:getCrateLength();
    !crateW = tsc:getCrateWidth();
    !crateH = tsc:getCrateHeight();
    !itemL = tsc:getPackedItemLength();
    !itemW = tsc:getPackedItemWidth();
    !itemH = tsc:getPackedItemHeight();

    if (op:equals(!crateL,-1) || op:equals(!crateW,-1) || op:equals(!crateH,-1) || op:equals(!itemL,-1) || op:equals(!itemW,-1) || op:equals(!itemH,-1)) {
        //TODO: use semantics for all questions, move text into prompt examples
        !bindings = act:askQuestionFromString(?actor,"Before commencing, there are unset packing parameters which must be configured. May I ask for them now?", val(X));
        if (~op:invokeMethod(!bindings,"isEmpty")) {
            !response = op:get(!bindings, !x);
            if (op:equalsValue(!response,yes)) {
                act:configurePackingParameters(!instructedDirectly);
                !numItems = act:getNumberOfItemsToPackPerCrate();
                //TODO: had to write same loop twice because of recovery interaction with BLOCK contexts - revisit
                while (true) {
                    !updatedNumItems = act:getNumberOfItemsToPackPerCrate();
                    if (~op:equals(!numItems,!updatedNumItems)) {
                        //If numItems updated between loops, report packing a different total number of items
                        act:generateResponseFromString("Putting update to number of items to pack into effect. I will now pack !updatedNumItems items into each crate.");
                        !numItems = op:newObject("java.lang.Integer",!updatedNumItems);
                    }
                    tsc:resetPackedItemIndex();
                    act:reportPackingNewCrate();
                    act:packCrate(!numItems);
                    act:waitForNewCrate();
                    //act:reportPackingComplete(); //TODO: revert if using sensing to detect an empty crate
                }
            } else {
                exit(FAIL,missing(configurationParameters));
            }
        } else {
            exit(FAIL,missing(configurationParameters));
        }
    }

    //else {
    act:checkPackingResultUpToDate(); //TODO: Better way to do this? Need to check if solver has been run with up to date information
    act:reportMaxNumberOfPackedItems();
    !numItems = act:getNumberOfItemsToPackPerCrate();
    if (op:equals(!numItems,-1)) {
        !numItems = act:askForNumberOfItemsToPack();
        act:setNumberOfItemsToPackPerCrate(!numItems, !instructedDirectly);
    } else {
        !bindings = act:askQuestionFromString(?actor,"I am currently configured to pack !numItems items into each crate. Do you want to change that amount?",val(X));
        if (~op:invokeMethod(!bindings,"isEmpty")) {
            !response = op:get(!bindings, !x);
            if (op:equalsValue(!response,yes)) {
                !numItems = act:askForNumberOfItemsToPack();
                act:setNumberOfItemsToPackPerCrate(!numItems, !instructedDirectly);
            }
        }
    }
    //}

    while (true) {
        !updatedNumItems = act:getNumberOfItemsToPackPerCrate();
        if (~op:equals(!numItems,!updatedNumItems)) {
            //If numItems updated between loops, report packing a different total number of items
            act:generateResponseFromString("Putting update to number of items to pack into effect. I will now pack !updatedNumItems items into each crate.");
            !numItems = op:newObject("java.lang.Integer",!updatedNumItems);
        }
        tsc:resetPackedItemIndex();
        act:reportPackingNewCrate();
        act:packCrate(!numItems);
        act:waitForNewCrate();
        //act:reportPackingComplete(); //TODO: revert if using sensing to detect an empty crate
    }
}

() = configurePackingParameters(Boolean ?direct=true) {
    Integer !crateL;
    Integer !crateW;
    Integer !crateH;
    Integer !itemL;
    Integer !itemW;
    Integer !itemH;
    Integer !numItemsToPack;
    Boolean !instructedDirectly = false;

    if (op:equals(?direct,true)) {
        act:generateResponse("askForNow(relevantInformation)");
        !crateL = act:askForCrateLength();
        !crateW = act:askForCrateWidth();
        !crateH = act:askForCrateHeight();
        act:setCrateDimensions(!crateL,!crateW,!crateH);
        !itemL = act:askForItemLength();
        !itemW = act:askForItemWidth();
        !itemH = act:askForItemHeight();
        act:setPackedItemDimensions(!itemL,!itemW,!itemH);
    } else {
        //Only ask for uninitialized parameters
        //TODO: is there ever a case where this^ is actually correct to do though? Should just ask for all each time, or at least have a more in depth recovery dialogue
        !crateL = tsc:getCrateLength();
        if (op:equals(!crateL,-1)) {
            !crateL = act:askForCrateLength();
            act:setCrateLength(!crateL);
        }

        !crateW = tsc:getCrateWidth();
        if (op:equals(!crateW,-1)) {
            !crateW = act:askForCrateWidth();
            act:setCrateWidth(!crateW);
        }

        !crateH = tsc:getCrateHeight();
        if (op:equals(!crateH,-1)) {
            !crateH = act:askForCrateHeight();
            act:setCrateHeight(!crateH);
        }

        !itemL = tsc:getPackedItemLength();
        if (op:equals(!itemL,-1)) {
            !itemL = act:askForItemLength();
            act:setPackedItemLength(!itemL);
        }

        !itemW = tsc:getPackedItemWidth();
        if (op:equals(!itemW,-1)) {
            !itemW = act:askForItemWidth();
            act:setPackedItemWidth(!itemW);
        }

        !itemH = tsc:getPackedItemHeight();
        if (op:equals(!itemH,-1)) {
            !itemH = act:askForItemHeight();
            act:setPackedItemHeight(!itemH);
        }
    }
    act:generateResponseFromString("Okay, give me a moment to compute the optimal packing arrangement");
    act:solveCratePacking3D();
    act:reportFullyPackedCrateConfiguration();
    !numItemsToPack = act:askForNumberOfItemsToPack();
    act:setNumberOfItemsToPackPerCrate(!numItemsToPack, !instructedDirectly);
    if (op:equals(?direct,true)) {
        act:generateResponseFromString("packing parameters have been fully updated.");
    }
}

(Integer ?length) = askForCrateLength() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How long is the crate?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        //TODO EW: note to self - think about refactoring to have askQuestionFromString throw this error and then force
                // scripts to handle differently through recovery policy if desired.
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?length = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?width) = askForCrateWidth() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How wide is the crate?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?width = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?height) = askForCrateHeight() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How tall is the crate?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?height = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?length) = askForItemLength() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How long is the item being packed?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?length = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?width) = askForItemWidth() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How wide is the item being packed?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?width = op:newObject("java.lang.Integer", !sizeString);
    }
}

(Integer ?height) = askForItemHeight() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    (!bindings) = act:askQuestionFromString(?actor,"How tall is the item being packed?", sizeInMM(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    } else {
        !response = op:get(!bindings,!x);
        !sizeString = op:getName(!response);
        ?height = op:newObject("java.lang.Integer", !sizeString);
    }
}

() = reportFullyPackedCrateConfiguration() {
    Integer !numLayers;
    Integer !numItemsPerLayer;
    Integer !totalNumItems;
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }

    !numLayers = tsc:getNumLayers();
    !numItemsPerLayer = tsc:getNumberOfItemsPerLayer();
    !totalNumItems = tsc:getTotalNumberOfItems();

    if (op:equals(!totalNumItems,-1)) {
        //This case shouldn't be possible anymore with precondition
        exit(FAIL,not(ranPackingAlgorithmWithUpdatedSettings));
    } elseif (op:equals(!totalNumItems,0)) {
        act:generateResponse("cannot(fitAnyItemsInto(crate),given(currentKnownItemAndCrateDimensions))");
    } else {
        act:generateResponse("fullyPackedCrateConfiguration(maxItems(!totalNumItems),maxLayers(!numLayers),maxPerLayer(!numItemsPerLayer))");
    }
}

(Integer ?numItems) = askForNumberOfItemsToPack() {
    Map !bindings;
    Variable !x = "X";
    Integer !totalNumItems = 0;
    String !numItemsString;
    String !response;
    java.util.List !responseFormList;
    Predicate !responseForm;
    Predicate !fact;

    !totalNumItems = tsc:getTotalNumberOfItems();

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("val(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("amount(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString(?actor, "do you want me to fill each crate completely?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !response = op:get(!bindings, !x);
    if (op:equalsValue(!response,yes)) {
        ?numItems = op:newObject("java.lang.Integer", !totalNumItems);
        return;
    } elseif (op:equalsValue(!response,no)) {
        !bindings = act:askQuestionFromString(?actor, "How many items would you like me to pack?", amount(X));
        if (op:invokeMethod(!bindings,"isEmpty")) {
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
    }

    !numItemsString = op:get(!bindings, !x);
    if (op:equalsValue(!numItemsString,"max")) {
        ?numItems = op:newObject("java.lang.Integer", !totalNumItems);
    } else {
        ?numItems = op:newObject("java.lang.Integer", !numItemsString);
    }

    if (op:gt(?numItems,!totalNumItems)) {
        //act:generateResponseFromString("I cannot fit ?numItems into a single crate, the most I can do is !totalNumItems. I will proceed with that maximum amount.");
        act:generateResponse("therefore(and(cannot(fitNumberOfItemsInCrate(?numItems)),maxPackableItemsPerCrate(!totalNumItems)),proceedWith(maxAmount))");
        ?numItems = op:newObject("java.lang.Integer", !totalNumItems);
    }
    if (op:lt(?numItems,0)) {
        exit(FAIL,cannot(packNegativeAmountOfObjects));
    }

    //TODO: attach as success effects?
    !fact = op:createPredicate("numItemsPerCrate(X)");
    act:retractBelief(!fact);
    !fact = op:createPredicate("numItemsPerCrate(?numItems)");
    act:assertBelief(!fact);
}

//() = reportStartingPacking() {
//    //act:generateResponseFromString("Starting packing procedure now");
//    act:generateResponse("starting(packingProcedure)");
//}

() = reportPackingNewCrate() {
    //act:generateResponseFromString("Beginning to pack new crate");
    act:generateResponse("packing(newCrate)");
}

//() = reportPackingComplete() {
() = waitForNewCrate() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    Boolean !answered = false;

    !bindings = act:askQuestionFromString(?actor, "I have finished packing this crate, please let me know when you have replaced it with an empty one", replaced(packingCrate));
    act:generateResponseFromString("thank you!");
    //TODO: cannot check for abort(question) in this way because won't be any bindings in either case
    //if (~op:invokeMethod(!bindings,"isEmpty")) {
    //    !answered = op:newObject("java.lang.Boolean", true);
    //}
    //while (op:equals(!answered,false)) {
    //    !bindings = act:askQuestionFromString(?actor, "Please let me know when you have replaced the crate or cancel the current task", replaced(packingCrate));
    //    if (~op:invokeMethod(!bindings,"isEmpty")) {
    //        !answered = op:newObject("java.lang.Boolean", true);
    //    }
    //}

    //TODO: Replace if using vision to detect new crate
    //act:generateResponse("and(finishedPacking(crate),askFor(replace(crate)))");
    ////Input:REPLY(robotone,%addressee,and(finishedPacking(crate),askFor(replace(crate))),{})
    ////Output:{"translation":"I have finished packing this crate, please replace it with an empty one"}
}

() = reportAllPackingParameters() {
    act:reportPackedItemDimensions();
    act:reportCrateDimensions();
    act:reportCurrentPackedCrateConfiguration();
    act:reportFullyPackedCrateConfiguration();
}

() = reportPackedItemDimensions() {
    Integer !itemL;
    Integer !itemW;
    Integer !itemH;
    Predicate !dimsPred;

    !itemL = tsc:getPackedItemLength();
    !itemW = tsc:getPackedItemWidth();
    !itemH = tsc:getPackedItemHeight();
    !dimsPred = op:createPredicate("packedItemDimensions(!itemL,!itemW,!itemH)");
    act:generateResponse(!dimsPred);
}

() = reportPackedItemLength() {
    Integer !itemL;

    //  ^and ask if the user wants to set? Same goes for all of these methods
    !itemL = tsc:getPackedItemLength();
    act:generateResponse("packedItemLength(!itemL)");
}

() = reportPackedItemWidth() {
    Integer !itemW;

    !itemW = tsc:getPackedItemWidth();
    act:generateResponse("packedItemWidth(!itemW)");
}

() = reportPackedItemHeight() {
    Integer !itemH;

    !itemH = tsc:getPackedItemHeight();
    act:generateResponse("packedItemHeight(!itemH)");
}

() = reportCrateDimensions() {
    Integer !crateL;
    Integer !crateW;
    Integer !crateH;
    Predicate !dimsPred;

    !crateL = tsc:getCrateLength();
    !crateW = tsc:getCrateWidth();
    !crateH = tsc:getCrateHeight();
    !dimsPred = op:createPredicate("crateDimensions(!crateL,!crateW,!crateH)");
    act:generateResponse(!dimsPred);
}

() = reportCrateLength() {
    Integer !crateL;

    !crateL = tsc:getCrateLength();
    act:generateResponse("crateLength(!crateL)");
}

() = reportCrateWidth() {
    Integer !crateW;

    !crateW = tsc:getCrateWidth();
    act:generateResponse("crateWidth(!crateW)");
}

() = reportCrateHeight() {
    Integer !crateH;

    !crateH = tsc:getCrateHeight();
    act:generateResponse("crateHeight(!crateH)");
}

() = reportMaxNumberOfPackedItems() {
    Integer !numItems;
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }

    !numItems = tsc:getTotalNumberOfItems();
    if (op:equals(!numItems,-1)) {
        exit(FAIL,not(ranPackingAlgorithmWithUpdatedSettings));
        //act:generateResponse("cannot(report(maxNumberOfPackedItems),because(not(ranPackingAlgorithm)))");
    } else {
        act:generateResponse("maxPackableItemsPerCrate(!numItems)");
    }
}

() = reportMaxNumberOfPackedLayers() {
    Integer !numLayers;
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }

    !numLayers = tsc:getNumLayers();
    if (op:equals(!numLayers,-1)) {
        exit(FAIL,not(ranPackingAlgorithmWithUpdatedSettings));
        //act:generateResponse("cannot(report(maxNumberOfPackableLayers),because(not(ranPackingAlgorithm)))");
    } else {
        act:generateResponse("maxPackableLayersPerCrate(!numLayers)");
    }
}

() = reportMaxNumberOfPackedItemsPerLayer() {
    Integer !numItemsPerLayer;
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }

    !numItemsPerLayer = tsc:getNumberOfItemsPerLayer();
    if (op:equals(!numItemsPerLayer,-1)) {
        exit(FAIL,not(ranPackingAlgorithmWithUpdatedSettings));
        //act:generateResponse("cannot(report(maxNumberOfPackableItemsPerLayer),because(not(ranPackingAlgorithm)))");
    } else {
        act:generateResponse("maxPackableItemsPerLayer(!numItemsPerLayer)");
    }
}

() = reportCurrentNumberOfItemsToPack() {
    Integer !numItems;

    !numItems = act:getNumberOfItemsToPackPerCrate();
    if (op:equals(!numItems,-1)) {
        act:generateResponse("and(not(told(numberOfItemsPerCrate)),assuming(maxPossibleNumber))");
        //act:reportMaxNumberOfPackedItems();
    } else {
        act:generateResponse("currentPackedItemsPerCrate(!numItems)");
    }
}

() = reportCurrentNumberOfLayersToPack() {
    Integer !numItems;
    Integer !maxItemsPerLayer;
    Integer !numLayers;
    Integer !temp;
    Boolean !lastLayerFull = true;
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }

    !numItemsPerLayer = tsc:getNumberOfItemsPerLayer();
    !numItems = act:getNumberOfItemsToPackPerCrate();

    if (op:equals(!numItemsPerLayer,-1)) {
        exit(FAIL,not(ranPackingAlgorithmWithUpdatedSettings));
    }

    if (op:equals(!numItems,-1)) {
        act:generateResponse("and(not(told(numberOfItemsPerCrate)),assuming(maxPossibleNumber))");
        act:reportMaxNumberOfPackedLayers();
    } else {
        !numLayers = op:/(!numItems,!maxItemsPerLayer);
        !temp = op:*(!maxItemsPerLayer,!numLayers);
        if (~op:equals(!numItems,!temp)) {
            !numLayers = op:++(!numLayers);
            !lastLayerFull = op:newObject("java.lang.Boolean", false);
        }
        act:generateResponse("currentPackedLayersPerCrate(numLayers(!numLayers),lastLayerFull(!lastLayerFull))");
    }
}

() = reportCurrentPackedCrateConfiguration() {
    Integer !numItems;
    Integer !maxItemsPerLayer;
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }

    !numItems = act:getNumberOfItemsToPackPerCrate();
    !maxItemsPerLayer = tsc:getNumberOfItemsPerLayer();

    if (op:equals(!maxItemsPerLayer,-1)) {
        exit(FAIL,not(ranPackingAlgorithmWithUpdatedSettings));
    }
    
    if (op:equals(!numItems,-1)) {
        act:generateResponse("and(not(told(numberOfItemsPerCrate)),assuming(maxPossibleNumber))");
        act:reportFullyPackedCrateConfiguration();
    } else {
        act:generateResponse("currentPackedCrateConfiguration(totalItems(!numItems),maxItemsPerLayer(!maxItemsPerLayer))");
    }
}

() = solverNotRanRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_retry
        }
        failureReasons : {ranPackingAlgorithmWithUpdatedSettings(X)}
    }
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    edu.tufts.hrilab.packing.PackingResult3D !res;

    !bindings = act:askQuestionFromString(?actor,"Relevant packing parameters have been updated since the last time an optimal crate arrangement has been calculated, may I rerun the algorithm to be up-to-date?",val(X));
    if (~op:invokeMethod(!bindings,"isEmpty")) {
        !response = op:get(!bindings, !x);
        if (op:equalsValue(!response,yes)) {
            act:generateResponseFromString("Okay, please give me a moment to compute the optimal packing arrangement");
            !res = act:solveCratePacking3D();
            if (op:isNull(!res)) {
                exit(FAIL);
            }
            act:generateResponseFromString("Done! Thank you");
            return;
        }
    }
    //TODO: why does exit(FAIL_RECOVERY) block?
    exit(FAIL);
}

//---------------------

//TODO: move below to demo script file

() = setNumberOfItemsToPackPerCrate(Integer ?numItems, Boolean ?direct=true) {
    Predicate !fact;
    conditions : {
        pre: ranPackingAlgorithmWithUpdatedSettings(?actor);
    }
    //TODO: make sure a valid amount has been supplied based on max amount

    !fact = op:createPredicate("numItemsPerCrate(X)");
    act:retractBelief(!fact);
    !fact = op:createPredicate("numItemsPerCrate(?numItems)");
    act:assertBelief(!fact);

    //TODO: if called in the middle of packing a crate, will defer until next crate. But what if less than that number of items is currently in the crate?
    if (op:equals(?direct,true)) {
        act:generateResponseFromString("Okay, this change will take effect for the next packed crate");
    }
}

(Integer ?numItemsPerCrate) = getNumberOfItemsToPackPerCrate() {
    Predicate !queryPred;
    List !bindings;
    java.util.HashMap !binding;
    Variable !bindingVar = X;
    Symbol !numSymbol;
    String !numString;

    !queryPred = op:createPredicate("numItemsPerCrate(X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !numSymbol = op:get(!binding,!bindingVar);
        !numString = op:getName(!numSymbol);
        ?numItemsPerCrate = op:newObject("java.lang.Integer", !numString);
    } else {
        //TODO: fail here, or let any caller handle it?
        //TODO: see if default value for return arg works?
        ?numItemsPerCrate = op:newObject("java.lang.Integer", -1);
    }
}

//TODO: remove?
//Wrappers which don't actually need to exist?
() = setPackedItemDimensions(Integer ?itemLength, Integer ?itemWidth, Integer ?itemHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setPackedItemDimensions(?itemLength, ?itemWidth, ?itemHeight);
}

//TODO: add these to LLM prompts? Or if don't like the idea of being able to set one at a time, should at least report to user the correct way to update this rather than just say "dont know what you mean"
() = setPackedItemLength(Integer ?itemLength) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setPackedItemLength(?itemLength);
}

() = setPackedItemWidth(Integer ?itemWidth) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setPackedItemWidth(?itemWidth);
}

() = setPackedItemHeight(Integer ?itemHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setPackedItemHeight(?itemHeight);
}

() = setCrateDimensions(Integer ?crateLength, Integer ?crateWidth, Integer ?crateHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setCrateDimensions(?crateLength, ?crateWidth, ?crateHeight);
}

() = setCrateLength(Integer ?crateLength) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setCrateLength(?crateLength);
}

() = setCrateWidth(Integer ?crateWidth) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setCrateWidth(?crateWidth);
}

() = setCrateHeight(Integer ?crateHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setCrateHeight(?crateHeight);
}

(edu.tufts.hrilab.packing.PackingResult3D ?result) = solveCratePacking3D() {
    Variable !x;
    effects : {
        success : ranPackingAlgorithmWithUpdatedSettings(?actor);
        success : not(numItemsPerCrate(!x));
        success : numItemsPerCrate(-1);
    }
    ?result = tsc:solveCratePacking3D();
    if (op:isNull(?result)) {
        act:generateResponseFromString("I could not run the packing algorithm because the crate and item dimensions have not been fully defined");
        //TODO: fix user facing experience when interacting with recovery policy and use the below
        //exit(FAIL,not(fullyDefined(and(crateDimensions,itemDimensions))));
    }
}

//TODO: figure out where this lives on main branch
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
            act:reportGoalStatus(!status);
            act:generateResponse(!goalPred);
        }
    } else {
        act:reportNoCurrentGoal();
    }
}

() = checkPackingResultUpToDate() {
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }
    op:log("debug","[checkPackingResultUpToDate] dummy action");
}