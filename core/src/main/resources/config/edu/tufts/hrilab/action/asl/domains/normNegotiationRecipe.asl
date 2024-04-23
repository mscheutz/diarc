import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.Long;
import java.lang.Integer;
import java.lang.String;
import edu.tufts.hrilab.action.goal.GoalStatus;

/**
 * recipe format:
 *  recipe(lunchbox_one,
 *      container(tray:area),
 *      contains(
 *         counter(pillBottle,2),
 *         counter(screwBox,1),
 *         counter(ioCardcounter,1)
 *      )
 *  )
 */
() = defineRecipe["defines new recipe which is essentially a goal state to be achieved a planner"](edu.tufts.hrilab.fol.Symbol ?descriptor){

    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Symbol !containerID;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Symbol !containerPred;
    edu.tufts.hrilab.fol.Predicate !contentsPred;
    edu.tufts.hrilab.fol.Predicate !recipePred;
    edu.tufts.hrilab.fol.Predicate !contentType;
    edu.tufts.hrilab.fol.Symbol !contentLocation;
    edu.tufts.hrilab.fol.Symbol !containerRef;
    java.util.List !newRefsList;
    Integer !refsListLength;
    String !areaString;
    Symbol !areaSymbol;
    Predicate !tmp;
    Predicate !question;

    !bindings = act:askQuestionFromString(?actor,"What container does it use?", val(X));
    !containerPred = op:get(!bindings, !x);

    !containerPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "container",!containerPred);

    java.util.List !tmpDescriptors;
    !tmpDescriptors = op:newObject("java.util.ArrayList");
    Symbol !number;
    Symbol !contained;

    !bindings = act:askQuestionFromString(?actor,"Okay. What does a ?descriptor contain?", val(X));
    !contentType = op:get(!bindings, !x);
    op:log("debug","contentType: !contentType");

    while(~op:equalsValue(!contentType,none())){
        op:add(!tmpDescriptors,!contentType);
        (!bindings) = act:askQuestionFromString(?actor,"does it contain anything else?", val(X));
        (!contentType) = op:get(!bindings, !x);
        op:log("debug","contentType: !contentType");
    }

    (!contentsPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "contents",!tmpDescriptors);
    op:log("debug","contentsPred: !contentsPred");

    (!recipePred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "recipe",?descriptor,!containerPred,!contentsPred);
    act:assertBelief(!recipePred);
    op:log("info","!recipePred");
    act:injectDictionaryEntry(?descriptor,"KITTYPE",?descriptor,"");
    act:submitTTSRequest(?actor,?actor,"okay");
}

//todo: This got out of hand and should be its own algorithm somewhere. No longer functional, but here for reference
() = lookfor[""](Symbol ?actor:fetch, java.util.List ?searchList) {

    java.util.List !poses;
    !poses = act:getAllPoses();
    java.util.List !searchContents;
    java.util.List !refs;
    java.util.List !descriptors;
    java.util.List !descriptorContents;
    edu.tufts.hrilab.fol.Term !descriptorTerm;
    edu.tufts.hrilab.fol.Term !property;
    edu.tufts.hrilab.fol.Term !searchItem;
    Symbol !descriptorName;
    java.lang.Long !typeId;
    java.lang.String !symbolString;

    java.lang.Integer !index = 0;
    java.lang.Integer !searchSize;
    !searchSize = op:invokeMethod(?searchList, "size");
    java.lang.Integer !test;

    Symbol !previousPose = movebaselocation_0;

    //The tally is used to keep track of how many objects of each type we've seen
    java.util.List !tally;
    !tally = act:initializeTally(?searchList);
    java.lang.Integer !currentAmount;
    java.lang.Integer !refsSize;
    java.lang.Integer !updatedAmount;

    //For each location in the location consultant (sorted by closeness, ideally)
    foreach(!pose : !poses) {
        op:log("debug", "looking at pose !pose");
        act:gotomovebase(!previousPose, !pose);

        //todo: change back to a for(...) statement for simplicity
        !index = op:newObject("java.lang.Integer", 0);
        while (op:lt(!index, !searchSize)) {
            !searchItem = op:get(?searchList, !index);
            !searchContents = op:getArgs(!searchItem);
            !descriptorTerm = op:get(!searchContents, 1);
            !descriptors = op:getArgs(!descriptorTerm);
            op:log("debug", "querying !searchItem for !searchContents to get !descriptorTerm with !descriptors)");

            //Get the descriptor of the item being looked for
            !descriptorContents = op:getArgs(!descriptorTerm);
            !property = op:get(!descriptorContents, 0);
            !descriptorName = op:invokeMethod(!property, "getName");
            op:log("debug", "querying !descriptorContents for !property with !descriptorName)");

            //Perform a search for a specific set of descriptors

            !typeId = act:getTypeId(!descriptors);
            op:log("debug", "looking for item !searchItem via !descriptors");
            act:startType(!typeId);
            !refs = act:getCurrentReferences();
            act:stopAndRemoveType(!typeId);

            // Assert relevant properties for all seen objects
            foreach(!ref : !refs) {
                act:detectObjectAtLocation(!ref, !descriptorName, !pose);
            }

            //update tally by removing the objects we've seen
            !currentAmount = op:invokeMethod(!tally,"get",!index);
            !refsSize = op:invokeMethod(!refs, "size");
            !updatedAmount = op:-(!currentAmount, !refsSize);
            op:invokeMethod(!tally,"set",!index,!updatedAmount);
            !index = op:++(!index);
        }

        //todo: fix this with something like !previousPose = !pose
        !symbolString = op:invokeMethod(!pose,"toString");
        !previousPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !symbolString);

        // Check the tally to see if we're done
        !test = op:newObject("java.lang.Integer", 0);
        foreach(!count : !tally) {
            if(op:gt(!count, 0)) {
                !test = op:++(!test);
            }
        }
        if(op:==(!test, 0)) {
            op:log("info", "Found all objects!");
            return;
        }
    }

    exit(FAIL);
}

() = detectObjectAtLocation[""](Symbol ?refId, Symbol ?property, Symbol ?location) {
    effects : {
        success infer : object(?refId, ?property); //assert that it exists
        success infer : at(?refId, ?location); //assert that it's at the current location
        success infer : propertyof(?refId, ?property); //assert that it has the given property (for fluents)
        success infer : fluent_increase(amount(?location, ?property), 1); //increase fluent
    }

    op:log("debug", "Detected ?refId w property ?property at ?location");

    if(op:equalsValue(?property,medicalcaddy)) {
        act:initializeContainer(?refId);
    }
    if(op:equalsValue(?property,box)) {
        act:initializeContainer(?refId);
    }
    if(op:equalsValue(?property,bowl)) {
        act:initializeContainer(?refId);
    }
}

//todo: this needs to be auto generated somehow. Maybe via recipe?
() = initializeContainer[""](Symbol ?refId) {
    effects : {
        success infer : fluent_equals(amount(?refId, painkiller:property), 0); //increase fluent
        success infer : fluent_equals(amount(?refId, bandagebox:property), 0); //increase fluent
        success infer : fluent_equals(amount(?refId, antiseptic:property), 0); //increase fluent
        success infer : fluent_equals(amount(?refId, apple:property), 0);
        success infer : fluent_equals(amount(?refId, baseball:property), 0);
        success infer : fluent_equals(amount(?refId, glassbottle:property), 0);
        success infer : fluent_equals(amount(?refId, carrot:property), 0);
        success infer : fluent_equals(amount(?refId, donut:property), 0);
        success infer : fluent_equals(amount(?refId, flowerpot:property), 0);
        success infer : fluent_equals(amount(?refId, computermouse:property), 0);
        success infer : fluent_equals(amount(?refId, car:property), 0);
        success infer : fluent_equals(amount(?refId, sportsbottle:property), 0);
        success infer : fluent_equals(amount(?refId, teddybear:property), 0);
        success infer : fluent_equals(amount(?refId, tennisball:property), 0);
        success infer : fluent_equals(amount(?refId, waterbottle:property), 0);
    }
    op:log("debug", "initialized ?refId as container");
}

(edu.tufts.hrilab.fol.Symbol ?recipeInstance) = createRecipeGoal["creates goal state to submit to planner based on ?recipeID"](edu.tufts.hrilab.fol.Symbol ?recipeID){

    edu.tufts.hrilab.fol.Symbol !tmp;
    //decode lunchbox recipe
    edu.tufts.hrilab.fol.Variable !containerVar = "X"; //container name
    edu.tufts.hrilab.fol.Variable !contentsVar = "Y"; //Contents
    edu.tufts.hrilab.fol.Predicate !queryPred;
    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","recipe",?recipeID,!containerVar,!contentsVar);
    op:log(debug,"queryPred: !queryPred");

    java.util.List !bindings;
    !bindings = act:queryBelief(!queryPred);
    java.util.Map !binding;
    !binding = op:get(!bindings, 0);

    //Get the contents predicate from the bindings
    edu.tufts.hrilab.fol.Predicate !contentsPred;
    !contentsPred = op:get(!binding,!contentsVar);
    op:log("debug","contentsPred: !contentsPred");

    java.util.List !contents;
    !contents = op:getArgs(!contentsPred);
    op:log("debug","contents: !contents");

    //Get the name of the container from the bindings
    edu.tufts.hrilab.fol.Symbol !container;
    !container = op:get(!binding,!containerVar);
    !container = op:getArg(!container,0);
    !container = op:getArg(!container,0);
    op:log("debug","container: !container");

    java.util.List !searchTerms;
    !searchTerms = op:newObject("java.util.ArrayList");
    op:invokeMethod(!searchTerms,"addAll",!contents);
    edu.tufts.hrilab.fol.Predicate !containerSearchTerm;
    !containerSearchTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "content(1,descriptors(!container))");
    op:add(!searchTerms,!containerSearchTerm);
    op:log("debug", !searchTerms);

    act:lookfor(!searchTerms);

    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","propertyof",!containerVar,!container);
    !bindings = act:queryBelief(!queryPred);
    op:log("debug","bindings: !bindings");

    !binding = op:get(!bindings, 0);
    !container = op:get(!binding,!containerVar);

    //create goal predicate
    Predicate !goalPred;
    java.util.List !goalArgs;
    !goalArgs = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    edu.tufts.hrilab.fol.Symbol !property;
    java.util.List !contentArgs;
    edu.tufts.hrilab.fol.Symbol !contentAmount;
    java.lang.String !functionName = "amount";
    edu.tufts.hrilab.fol.Symbol !functionPred;
    edu.tufts.hrilab.fol.Predicate !arg;

    foreach(!content : !contents) {
       op:log("debug","content: !content");
       !contentArgs = op:getArgs(!content);
       (!contentAmount) = op:get(!contentArgs,0);
       op:log("debug","contentAmount: !contentAmount");
       !property = op:get(!contentArgs,1);
       !property = op:getArg(!property,0);
       op:log("debug","property: !property");
       !functionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate",!functionName,!container,!property);
       op:log("debug","functionPred: !functionPred");
       !arg = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","fluent_equals",!functionPred,!contentAmount);
       op:log("debug","arg: !arg");
       op:add(!goalArgs,!arg);
    }

    op:log("debug", !goalArgs);
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and",!goalArgs);
    op:log("info","goal generated for recipe: ?recipeID : !goalPred ");

    goal:!goalPred;

    //reach out and identify the container bound to the container area involved in the goal predicate. this is the packed kit refid that we will return
    Variable !areaVar = "Z"; //container name
    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","areaBoundToContainer",!areaVar,!containerVar);

    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "queryPred: !queryPred");
    op:log(debug,"bindings: !bindings");
    !binding = op:get(!bindings, 0);
    ?recipeInstance = op:get(!binding,!containerVar);

    //todo: assert that the kit is packed at this point?
}


() = deliver["?actor delivers a lunch box of ?recipe"](edu.tufts.hrilab.fol.Symbol ?actor, edu.tufts.hrilab.fol.Symbol ?recipe, edu.tufts.hrilab.fol.Symbol ?location) {

   edu.tufts.hrilab.fol.Term !tempMod;
   (!tempMod) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "none(none)");

   act:deliver(?recipe,!tempMod,?location);
   op:log("info", "Delivered");

}


//Wrapper script that is used to assemble a "package"
//TODO:rename this to something more general? maybe assemblePackage
() = deliver["?actor delivers a lunch box of ?recipe"](edu.tufts.hrilab.fol.Symbol ?actor, edu.tufts.hrilab.fol.Symbol ?recipe, edu.tufts.hrilab.fol.Term ?mod, edu.tufts.hrilab.fol.Symbol ?location) {

    Symbol !recipeInstance;
    Predicate !goalPred;
    long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.fol.Predicate !g;

    op:log("debug","Delivering ?recipe");
    !recipeInstance = act:createRecipeGoal(?recipe);
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "delivered", !recipeInstance, ?location);
    goal:!goalPred;

}
