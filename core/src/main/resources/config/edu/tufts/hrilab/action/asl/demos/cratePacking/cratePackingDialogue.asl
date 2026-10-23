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
    Integer !currentCrateCapacity;

    !crateL = tsc:getCrateLength();
    !crateW = tsc:getCrateWidth();
    !crateH = tsc:getCrateHeight();
    !itemL = tsc:getPackedItemLength();
    !itemW = tsc:getPackedItemWidth();
    !itemH = tsc:getPackedItemHeight();

    if (op:equals(!crateL,-1) || op:equals(!crateW,-1) || op:equals(!crateH,-1) || op:equals(!itemL,-1) || op:equals(!itemW,-1) || op:equals(!itemH,-1)) {
        //!bindings = act:askQuestionFromString(?actor,"Before commencing, there are unset packing parameters which must be configured. May I ask for them now?", val(X));
        !bindings = act:askQuestion(?actor,then(report(prerequisite(packing,configure(unsetParameters))),ask(can(askFor(them)))), val(X));
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
                        //If current crate capacity is already set to the updated number of items, means change was applied mid-loop. Don't report.
                        !currentCrateCapacity = tsc:getCurrentCrateCapacity();
                        if (~op:equals(!currentCrateCapacity,!updatedNumItems)) {
                            //act:generateResponseFromString("Putting update to number of items to pack into effect. I will now pack !updatedNumItems items into each crate.");
                            act:generateResponse("then(applyingParameterUpdate(numItemsToPack),packingPerCrate(!updatedNumItems))");
                        }
                        !numItems = op:newObject("java.lang.Integer",!updatedNumItems);
                    }
                    tsc:setCurrentCrateCapacity(!updatedNumItems);
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
        //!bindings = act:askQuestionFromString(?actor,"I am currently configured to pack !numItems items into each crate. Do you want to change that amount?",val(X));
        !bindings = act:askQuestion(?actor,then(report(configuredAmountToPackPerCrate(?actor,!numItems)),ask(want(?actor,changeAmount))),val(X));
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
            //If current crate capacity is already set to the updated number of items, means change was applied mid-loop. Don't report.
            !currentCrateCapacity = tsc:getCurrentCrateCapacity();
            if (~op:equals(!currentCrateCapacity,!updatedNumItems)) {
                //act:generateResponseFromString("Putting update to number of items to pack into effect. I will now pack !updatedNumItems items into each crate.");
                act:generateResponse("then(applyingParameterUpdate(numItemsToPack),packingPerCrate(!updatedNumItems))");
            }
            !numItems = op:newObject("java.lang.Integer",!updatedNumItems);
        }
        tsc:setCurrentCrateCapacity(!updatedNumItems);
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
    //act:generateResponseFromString("Okay, give me a moment to compute the optimal packing arrangement");
    act:generateResponse("report(needMomentToCompute(optimalPackingArrangement))");
    act:solveCratePacking3D();
    act:reportFullyPackedCrateConfiguration();
    !numItemsToPack = act:askForNumberOfItemsToPack();
    act:setNumberOfItemsToPackPerCrate(!numItemsToPack, !instructedDirectly);
    if (op:equals(?direct,true)) {
        //act:generateResponseFromString("packing parameters have been fully updated.");
        act:generateResponse("fullyUpdated(packingParameters)");
    }
}

(Integer ?length) = askForCrateLength() {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    String !sizeString;

    //(!bindings) = act:askQuestionFromString(?actor,"How long is the crate?", sizeInMM(X));
    (!bindings) = act:askQuestion(?actor,length(crate), sizeInMM(X));
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

    //(!bindings) = act:askQuestionFromString(?actor,"How wide is the crate?", sizeInMM(X));
    (!bindings) = act:askQuestion(?actor,width(crate), sizeInMM(X));
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

    //(!bindings) = act:askQuestionFromString(?actor,"How tall is the crate?", sizeInMM(X));
    (!bindings) = act:askQuestion(?actor,height(crate), sizeInMM(X));
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

    //(!bindings) = act:askQuestionFromString(?actor,"How long is the item being packed?", sizeInMM(X));
    (!bindings) = act:askQuestion(?actor,length(packedItem), sizeInMM(X));
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

    //(!bindings) = act:askQuestionFromString(?actor,"How wide is the item being packed?", sizeInMM(X));
    (!bindings) = act:askQuestion(?actor,width(packedItem), sizeInMM(X));
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

    //(!bindings) = act:askQuestionFromString(?actor,"How tall is the item being packed?", sizeInMM(X));
    (!bindings) = act:askQuestion(?actor,height(packedItem), sizeInMM(X));
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
    //!bindings = act:askQuestionFromString(?actor, "do you want me to fill each crate completely?", !responseFormList);
    !bindings = act:askQuestion(?actor, fill(?actor,eachCrate,completely), !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !response = op:get(!bindings, !x);
    if (op:equalsValue(!response,yes)) {
        ?numItems = op:newObject("java.lang.Integer", !totalNumItems);
        return;
    } elseif (op:equalsValue(!response,no)) {
        //!bindings = act:askQuestionFromString(?actor, "How many items would you like me to pack?", amount(X));
        !bindings = act:askQuestion(?actor, number(items,pack(?actor)), amount(X));
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
    List !responseFormList;
    Predicate !responseForm;

    //!bindings = act:askQuestionFromString(?actor, "I have finished packing this crate, please let me know when you have replaced it with an empty one", replaced(packingCrate));

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("replaced(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("didIt(X)");
    op:add(!responseFormList,!responseForm);

    !bindings = act:askQuestion(?actor, then(report(complete(pack(?actor,crate))),ask(ackWhen(replaced(packedCrate,emptyCrate)))), !responseFormList);
    //Asked not to have this
    //act:generateResponseFromString("thank you!");

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

() = reportConfiguredNumberOfItemsToPack() {
    Integer !configuredNumItems;
    Integer !currentNumItems;
    String !currentPackingStatus;
    Boolean !ranUpdatedAlgo;

    !configuredNumItems = act:getNumberOfItemsToPackPerCrate();
    !currentNumItems = tsc:getCurrentCrateCapacity();
    !currentPackingStatus = tsc:getCratePackingStatus();

    //cases: upcoming | current/prev
    //  -1 -1  - report not told, assuming maximum possible number
    //  -1 set - say currently packing N, but parameters affecting the number of possible items have been updated (algo has been run)
    //  set -1 - Think actually this one isn't possible. can never start packing without setting numToPack to positive num
    //  set set == just say N
    //  set set =/= say currently packing N, next crate will pack M - only possible if changed while packing

    if (op:equals(!configuredNumItems,-1)) {
        if (op:equals(!currentNumItems,-1)) {
            // Neither configured
            act:generateResponse("not(configured(numberOfItemsPerCrate))");
        } else {
            // Current/last crate packing !numItems into crate, next crate unknown/max
            if (op:equals(!currentPackingStatus,"packing")) {
                //Don't believe this case is possible
                act:generateResponse("but(currentPackedItemsPerCrate(!currentNumItems),and(nextPackedItemsPerCrate(unknown),because(packingParametersChanged)))");
            } elseif (op:equals(!currentPackingStatus,"complete")) {
                //Don't believe this case is possible
                act:generateResponse("but(lastPackedItemsPerCrate(!currentNumItems),and(nextPackedItemsPerCrate(unknown),because(packingParametersChanged)))");
            } else {
                act:generateResponse("isNot(configured(numberOfItemsPerCrate),because(packingParametersChanged))");
            }
        }
    } else {
        !ranUpdatedAlgo = act:getHaveRanUpdatedPackingAlgorithm();
        if (op:equals(!ranUpdatedAlgo,false)) {
            act:generateResponse("isNot(configured(numberOfItemsPerCrate),because(packingParametersChanged))");
        } elseif (op:equals(!currentNumItems,!configuredNumItems)) {
            // Configured amount to pack unchanged
            act:generateResponse("currentPackedItemsPerCrate(!currentNumItems)");
        } else {
            // Configured amount has changed. Next crate packing less
            if (op:equals(!currentPackingStatus,"canceled")) {
                act:generateResponse("upcomingPackedItemsPerCrate(!configuredNumItems)");
            } else {
                act:generateResponse("but(currentPackedItemsPerCrate(!currentNumItems),upcomingPackedItemsPerCrate(!configuredNumItems))");
            }
        }
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

() = reportNumberOfItemsInCrate() {
    Integer !numItems;
    String !crateStatus;

    !crateStatus = tsc:getCratePackingStatus();
    !numItems = tsc:getCurrentCrateNumItems();
    if (op:equals(!crateStatus,"")) {
        act:generateResponse("not(startedPacking(?actor,crates))");
    } elseif (op:equals(!crateStatus,"canceled")) {
        act:generateResponse("beforePackingWasCanceled(held(lastCrate,numberOfItems(!numItems)))");
    } else {
        act:generateResponse("holding(currentCrate,numberOfItems(!numItems))");
    }
}

() = reportWhatPackingParametersExist() {
    act:generateResponse("packingParametersWhichExistAndCanBeConfigured(crate(length,width,height),packedItem(length,width,height),numberOfItemsToPackPerCrate)");
}

() = reportUnsetPackingParameters() {
    Integer !crateL;
    Integer !crateW;
    Integer !crateH;
    Integer !itemL;
    Integer !itemW;
    Integer !itemH;
    Integer !numItemsToPack;
    List !unsetParamList;
    String !unsetParamString;

    !unsetParamList = op:newObject("java.util.ArrayList");

    !crateL = tsc:getCrateLength();
    if (op:equals(!crateL,-1)) {
        op:add(!unsetParamList,"crateLength");
    }
    !crateW = tsc:getCrateWidth();
    if (op:equals(!crateW,-1)) {
        op:add(!unsetParamList,"crateWidth");
    }
    !crateH = tsc:getCrateHeight();
    if (op:equals(!crateH,-1)) {
        op:add(!unsetParamList,"crateHeight");
    }
    !itemL = tsc:getPackedItemLength();
    if (op:equals(!itemL,-1)) {
        op:add(!unsetParamList,"itemLength");
    }
    !itemW = tsc:getPackedItemWidth();
    if (op:equals(!itemW,-1)) {
        op:add(!unsetParamList,"itemWidth");
    }
    !itemH = tsc:getPackedItemHeight();
    if (op:equals(!itemH,-1)) {
        op:add(!unsetParamList,"itemHeight");
    }
    !numItemsToPack = act:getNumberOfItemsToPackPerCrate();
    if (op:equals(!numItemsToPack,-1)) {
        op:add(!unsetParamList,"numberOfItemsToPack");
    }

    if (op:isEmpty(!unsetParamList)) {
        act:generateResponse("allSet(packingParameters)");
    } else {
        !unsetParamString = op:invokeStaticMethod("java.lang.String","join",",",!unsetParamList);
        act:generateResponse("unsetParamsAre(!unsetParamString)");
    }
}

() = solverNotRanRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : retry
        }
        failureReasons : {ranPackingAlgorithmWithUpdatedSettings(X)}
    }
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    ai.thinkingrobots.mtracs.packing.PackingResult3D !res;

    //!bindings = act:askQuestionFromString(?actor,"Relevant packing parameters have been updated since the last time an optimal crate arrangement has been calculated, may I rerun the algorithm to be up-to-date?",val(X));
    !bindings = act:askQuestion(?actor,then(report(updatedSince(relevantPackingParameters,calculated(optimalCrateArrangement))),ask(may(?actor,rerunAlgorithm),desire(upToDate))),val(X));
    if (~op:invokeMethod(!bindings,"isEmpty")) {
        !response = op:get(!bindings, !x);
        if (op:equalsValue(!response,yes)) {
            //act:generateResponseFromString("Okay, please give me a moment to compute the optimal packing arrangement");
            act:generateResponse("report(needMomentToCompute(optimalPackingArrangement))");
            !res = act:solveCratePacking3D();
            if (op:isNull(!res)) {
                exit(FAIL);
            }
            //act:generateResponseFromString("Done! Thank you");
            act:generateResponse("and(report(done),ack(thankYou))");
            return;
        }
    }
    //TODO: why does exit(FAIL_RECOVERY) block?
    exit(FAIL);
}

//---------------------

//TODO: move below to demo script file

() = setNumberOfItemsToPackPerCrate(Integer ?numItems, Boolean ?direct=true) {
    conditions : {
        pre: ranPackingAlgorithmWithUpdatedSettings(?actor);
    }
    Predicate !fact;
    Integer !currentNumItems;
    Integer !maxNumItems;
    String !packingStatus;

    !maxNumItems = tsc:getTotalNumberOfItems();
    // make sure a valid amount has been supplied based on max amount
    if (op:le(?numItems,0)) {
        exit(FAIL,not(supplied(numberGreaterThanZero)));
    } elseif (op:gt(?numItems,!maxNumItems)) {
        exit(FAIL,greaterThan(?numItems,maxCapacity));
    }

    !fact = op:createPredicate("numItemsPerCrate(X)");
    act:retractBelief(!fact);
    !fact = op:createPredicate("numItemsPerCrate(?numItems)");
    act:assertBelief(!fact);

    if (op:equals(?direct,true)) {
        !packingStatus = tsc:getCratePackingStatus();
        if (op:equals(!packingStatus,"packing")) {
            //!currentNumItems = tsc:getCurrentCrateNumItems();
            !currentNumItems = tsc:getPackingItemNum();
            //if (op:gt(?numItems,!currentNumItems)) {
            if (op:ge(?numItems,!currentNumItems)) {
                tsc:setCurrentCrateCapacity(?numItems);
                act:generateResponse("report(changeWillTakeEffectForTheCurrentPackedCrate)");
                return;
            }
        }

        //act:generateResponseFromString("Okay, this change will take effect for the next packed crate");
        act:generateResponse("report(changeWillTakeEffectForTheNextPackedCrate)");
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

() = setPackedItemDimensions(Integer ?itemLength, Integer ?itemWidth, Integer ?itemHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?itemLength,1)) {
        act:generateResponse("mustBe(itemLength,greaterThan(0))");
        return;
    }
    if (op:lt(?itemWidth,1)) {
        act:generateResponse("mustBe(itemWidth,greaterThan(0))");
        return;
    }
    if (op:lt(?itemHeight,1)) {
        act:generateResponse("mustBe(itemHeight,greaterThan(0))");
        return;
    }
    tsc:setPackedItemDimensions(?itemLength, ?itemWidth, ?itemHeight);
}

() = setPackedItemLength(Integer ?itemLength) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?itemLength,1)) {
        act:generateResponse("mustBe(itemLength,greaterThan(0))");
        return;
    }
    tsc:setPackedItemLength(?itemLength);
}

() = setPackedItemWidth(Integer ?itemWidth) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?itemWidth,1)) {
        act:generateResponse("mustBe(itemWidth,greaterThan(0))");
        return;
    }
    tsc:setPackedItemWidth(?itemWidth);
}

() = setPackedItemHeight(Integer ?itemHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?itemHeight,1)) {
        act:generateResponse("mustBe(itemHeight,greaterThan(0))");
        return;
    }
    tsc:setPackedItemHeight(?itemHeight);
}

() = reconfigurePackedItemDimensions() {
    effects : {
        always : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    Integer !itemL;
    Integer !itemW;
    Integer !itemH;

    act:generateResponse("report(clearing(?actor,currentNotionOfPackedItemDimensions))");
    tsc:setPackedItemLength(-1);
    tsc:setPackedItemWidth(-1);
    tsc:setPackedItemHeight(-1);

    !itemL = act:askForItemLength();
    act:setPackedItemLength(!itemL);
    !itemW = act:askForItemWidth();
    act:setPackedItemWidth(!itemW);
    !itemH = act:askForItemHeight();
    act:setPackedItemHeight(!itemH);

    act:generateResponse("report(?actor,haveUpdatedItemDimensions)");
}

() = setCrateDimensions(Integer ?crateLength, Integer ?crateWidth, Integer ?crateHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?crateLength,1)) {
        act:generateResponse("mustBe(crateLength,greaterThan(0))");
        return;
    }
    if (op:lt(?crateWidth,1)) {
        act:generateResponse("mustBe(crateWidth,greaterThan(0))");
        return;
    }
    if (op:lt(?crateHeight,1)) {
        act:generateResponse("mustBe(crateHeight,greaterThan(0))");
        return;
    }
    tsc:setCrateDimensions(?crateLength, ?crateWidth, ?crateHeight);
}

() = setCrateLength(Integer ?crateLength) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?crateLength,1)) {
        act:generateResponse("mustBe(crateLength,greaterThan(0))");
        return;
    }
    tsc:setCrateLength(?crateLength);
}

() = setCrateWidth(Integer ?crateWidth) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?crateWidth,1)) {
        act:generateResponse("mustBe(crateWidth,greaterThan(0))");
        return;
    }
    tsc:setCrateWidth(?crateWidth);
}

() = setCrateHeight(Integer ?crateHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    if (op:lt(?crateHeight,1)) {
        act:generateResponse("mustBe(crateHeight,greaterThan(0))");
        return;
    }
    tsc:setCrateHeight(?crateHeight);
}

() = reconfigureCrateDimensions() {
    effects : {
        always : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    Integer !crateL;
    Integer !crateW;
    Integer !crateH;

    act:generateResponse("report(clearing(?actor,currentNotionOfCrateDimensions))");
    tsc:setCrateLength(-1);
    tsc:setCrateWidth(-1);
    tsc:setCrateHeight(-1);

    !crateL = act:askForCrateLength();
    act:setCrateLength(!crateL);
    !crateW = act:askForCrateWidth();
    act:setCrateWidth(!crateW);
    !crateH = act:askForCrateHeight();
    act:setCrateHeight(!crateH);

    act:generateResponse("report(?actor,haveUpdatedCrateDimensions)");
}

(ai.thinkingrobots.mtracs.packing.PackingResult3D ?result) = solveCratePacking3D() {
    Variable !x;
    effects : {
        success : ranPackingAlgorithmWithUpdatedSettings(?actor);
        success : not(numItemsPerCrate(!x));
        success : numItemsPerCrate(-1);
    }
    ?result = tsc:solveCratePacking3D();
    if (op:isNull(?result)) {
        //act:generateResponseFromString("I could not run the packing algorithm because the crate and item dimensions have not been fully defined");
        act:generateResponse("and(couldNot(run(?actor,packingAlgorithm)),because(not(fullyDefined(and(crateDimensions,itemDimensions)))))");
        //TODO: fix user facing experience when interacting with recovery policy and use the below
        //exit(FAIL,not(fullyDefined(and(crateDimensions,itemDimensions))));
    }
}

() = checkPackingResultUpToDate() {
    conditions : {
        pre : ranPackingAlgorithmWithUpdatedSettings(?actor);
    }
    op:log("debug","[checkPackingResultUpToDate] dummy action");
}

(Boolean ?ran) = getHaveRanUpdatedPackingAlgorithm() {
    Predicate !queryPred;
    List !bindings;
    Variable !bindingVar = X;
    Map !elem;
    Symbol !a;
    String !actorString;

    !queryPred = op:createPredicate("ranPackingAlgorithmWithUpdatedSettings(X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        foreach(!elem : !bindings) {
            !a = op:get(!elem,!bindingVar);
            !actorString = op:getName(!a);
            if (op:equalsValue(!actorString,?actor)) {
                ?ran = op:newObject("java.lang.Boolean", true);
                return;
            }
        }
    }
    ?ran = op:newObject("java.lang.Boolean", false);
}

() = updateAllDimensions(Integer ?crateLength, Integer ?crateWidth, Integer ?crateHeight, Integer ?itemLength, Integer ?itemWidth, Integer ?itemHeight) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setCrateDimensions(?crateLength, ?crateWidth, ?crateHeight);
    tsc:setPackedItemDimensions(?itemLength, ?itemWidth, ?itemHeight);
}

() = updateItemTolerance(Integer ?tolerance) {
    effects : {
        success : not(ranPackingAlgorithmWithUpdatedSettings(?actor));
    }
    tsc:setTolerance(?tolerance);
}