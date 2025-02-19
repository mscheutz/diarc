import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.Long;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
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
() = defineRecipe["defines new recipe which is essentially a goal state to be achieved a planner"](Symbol ?descriptor){

    java.util.Map !bindings;
    Symbol !containerID;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Symbol !containerPred;
    Predicate !contentsPred;
    Predicate !recipePred;
    Predicate !contentType;
    Symbol !contentLocation;
    Symbol !containerRef;
    List !newRefsList;
    Integer !refsListLength;
    String !areaString;
    Symbol !areaSymbol;
    Predicate !tmp;
    Predicate !question;

    !bindings = act:askQuestionFromString(?actor,"What container does it use?", val(X));
    !containerPred = op:get(!bindings, !x);

    //check if we know about any container of the specified type, if not ask
    if(act:knowsContainerLocation(!containerPred)){
        op:log(debug,"don't know the location of any !containerPred , need to ask");
        (!question) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","where(is(there,num(1,!containerPred)))");
        (!bindings) = act:askQuestion(?actor,!question, val(X));
        (!contentLocation) = op:get(!bindings, !x);
        (!newRefsList) = act:positContents(!contentLocation);

        //todo: this is hacky. we're asserting isContainer based on the newly generated predicate so that we can check on it in the domain actions
        !refsListLength = op:invokeMethod(!newRefsList, "size");
        if(~op:equals(!refsListLength, 1)) {
            op:log(error, "[defineRecipe] recipes are only allowed to have a singular possible container provided to them. !refsListLength hypotheticals were posited during execution");
        }
        !containerRef = op:get(!newRefsList, 0);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "isContainer(!containerRef)");
        act:assertBelief(!tmp);

        //create the area object associated with the container
        !areaString = op:invokeMethod(!containerRef, "getName");
        !areaString = op:invokeMethod(!areaString, "concat", "Area");
        !areaSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaString);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "object(!areaSymbol,area)");
        act:assertBelief(!tmp);
        !areaSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaString, "area");
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "areaBoundToContainer(!areaSymbol, !containerRef)");
        act:assertBelief(!tmp);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "containerArea(!areaSymbol)");
        act:assertBelief(!tmp);
    }

    !containerPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "container",!containerPred);

    List !tmpDescriptors;
    !tmpDescriptors = op:newObject("java.util.ArrayList");
    Symbol !number;
    Symbol !contained;

    !bindings = act:askQuestionFromString(?actor,"Okay. What does a ?descriptor contain?", val(X));
    !contentType = op:get(!bindings, !x);
    op:log("debug","contentType: !contentType");

    while(~op:equalsValue(!contentType,none())){
        op:add(!tmpDescriptors,!contentType);
        if(act:knowsContentLocation(!contentType)){
            op:log(debug,"don't know the location of any !contentType , need to ask");
            (!number) = op:invokeMethod(!contentType, "get", 0);
            (!contained) = op:invokeMethod(!contentType, "get", 1);
            (!question) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","where(are(there,num(!number,!contained)))");
            (!bindings) = act:askQuestion(?actor,!question, val(X));
            //(!bindings) = act:askQuestionFromString(?actor,"where are there !contentType", val(X));
            (!contentLocation) = op:get(!bindings, !x);
            act:positContents(!contentLocation);
            //TODO:brad: ask where item can be found
        }
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
    act:generateResponseFromString("okay");
}


//TODO:brad: not handling modifications for now
(Symbol ?recipeInstance) = executeRecipeGoal["creates goal state to submit to planner based on ?recipeID"](Symbol ?recipeID){

    //recipe format:
    //recipe(lunchbox_one,
    //    container(tray:container), //TODO:this should have type 'place'
    //    contains(
    //       counter(pillBottle,2),
    //       counter(screwBox,1),
    //       counter(ioCardcounter,1)
    //    )
    //)

    //goalFormat:
    // derived(
    //  label,
    //  and(
    //    fluent_equals(amount, tray:container, pillBottlecounter:counter, 2),
    //    fluent_equals(amount, tray:container, screwBoxcounter:counter, 1),
    //    fluent_equals(amount, tray:container, ioCardcounter:counter, 1)
    //  )
    //)
    //fluent_equals(amount, place, counter, #)


//recipe(repairkit,container(toolCaddy),contents(contains(physobj_0),contains(physobj_1)))
    Symbol !tmp;

    //decode lunchbox recipe
    edu.tufts.hrilab.fol.Variable !containerVar = "X"; //container name
    edu.tufts.hrilab.fol.Variable !contentsVar = "Y"; //Contents
    Predicate !queryPred;
    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","recipe",?recipeID,!containerVar,!contentsVar);
    op:log(debug,"queryPred: !queryPred");

    List !bindings;
    !bindings = act:queryBelief(!queryPred);
    op:log("info","bindings: !bindings");

    java.util.Map !binding;
    !binding = op:get(!bindings, 0);

    //Get the contents predicate from the bindings
    Predicate !contentsPred;
    !contentsPred = op:get(!binding,!contentsVar);
    op:log("debug","contentsPred: !contentsPred");

    List !contents;
    !contents = op:getArgs(!contentsPred);
    op:log("debug","contents: !contents");

    //Get the name of the container from the bindings
    Symbol !container;
    !container = op:get(!binding,!containerVar);
    !container = op:getArg(!container,0);
    op:log("debug","container: !container");

    //areaBoundToContainer(caddyArea, toolCaddy).
    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","areaBoundToContainer",!containerVar,!contentsVar);
    !bindings = act:queryBelief(!queryPred);
    op:log("info", "query pred for container: !queryPred");
    op:log("info","bindings: !bindings");
    !binding = op:get(!bindings, 0);
    !container = op:get(!binding,!containerVar);

    //TODO:brad: implement modification functionality
    //    log.info("ingredientDescriptors: "+ingredientDescriptors);
    //
    //    if(mod != null && !mod.getName().equals("none")){
    //      Symbol newVal = mod.get(0);
    //      Symbol toReplace = mod.get(1);
    //      ingredientDescriptors.remove(ingredientDescriptors.indexOf(toReplace));
    //      ingredientDescriptors.add(newVal);
    //      log.info("modified ingredientDescriptors: "+ingredientDescriptors);
    //    }

    //create goal predicate
    Predicate !goalPred;
    List !goalArgs;
    !goalArgs = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    Symbol !property;
    List !contentArgs;
    Symbol !contentAmount;
    String !functionName = "amount";
    Symbol !functionPred;
    Predicate !arg;

    foreach(!content : !contents) {
       op:log("debug","content: !content");

       !contentArgs = op:getArgs(!content);
       (!contentAmount) = op:get(!contentArgs,0);
       op:log("debug","contentAmount: !contentAmount");
       !property = op:get(!contentArgs,1);
       op:log("debug","property: !property");
       !functionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate",!functionName,!container,!property);
       op:log("debug","functionPred: !functionPred");
       !arg = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","fluent_equals",!functionPred,!contentAmount);
       op:log("debug","arg: !arg");
       op:add(!goalArgs,!arg);
    }

     !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and",!goalArgs);
//    ?goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(fluent_equals(amount(caddyArea,screw),1.0),fluent_equals(amount(caddyArea,smallgear),1.0))");
//and(fluent_equals(amount(caddyArea,screw),1.0),fluent_equals(amount(caddyArea,smallgear),1.0))

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


() = deliver["?actor delivers a lunch box of ?recipe"](Symbol ?recipe, Symbol ?location) {

   Term !tempMod;
   !tempMod = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "none(none)");

   act:deliver(?recipe,!tempMod,?location);
   op:log("info", "Delivered");

}


//Wrapper script that is used to assemble a "package"
//TODO:rename this to something more general? maybe assemblePackage
() = deliver["?actor delivers a lunch box of ?recipe"](edu.tufts.hrilab.fol.Symbol ?recipe, edu.tufts.hrilab.fol.Term ?mod, edu.tufts.hrilab.fol.Symbol ?location) {

  Symbol !recipeInstance;
  Predicate !goalPred;

  //use ?recipe to get contains pred from belief
//  !goalPred = act:createRecipeGoalUngrounded(?recipe, ?mod);

  !recipeInstance = act:executeRecipeGoal(?recipe);
  !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "delivered", !recipeInstance, ?location);
  goal:!goalPred;
}

() = assemble["?actor assembles the ?recipe"](edu.tufts.hrilab.fol.Symbol ?recipe) {
  act:executeRecipeGoal(?recipe);
}

(boolean ?return) = knowsContainerLocation["checks if the location of any container of ?containerType is known"](Symbol ?containerType){

    //decode lunchbox recipe
    Variable !areaVar = "X"; //container name
    Predicate !queryPred;
    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","areaBoundToContainer",!areaVar,?containerType);

    //Query belief for bindings
    List !bindings;
    !bindings = act:queryBelief(!queryPred);
    op:log(debug,"bindings: !bindings");

    ?return = op:isEmpty(!bindings);

    op:log(debug,"[knowsContainerLocation] returning: ?return . The truth value here is the opposite of what you would expect, but it's not very easy to negate in ASL.");
}

//TODO:brad: could we do a numeric query here?
(boolean ?return) = knowsContentLocation["checks if the location of any item (second arg of ?containsPred) is known"](Predicate ?containsPred){

    Variable !locationVar = "X";

    Symbol !contentType;
    List !containsArgs;
    !containsArgs = op:getArgs(?containsPred);
    !contentType = op:get(!containsArgs,1);

    Predicate !queryPred;
    !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","observableAt",!contentType,!locationVar);

    op:log(debug,"[knowsContentLocation] queryPred: !queryPred");

    //Query belief for bindings
    List !bindings;
    !bindings = act:queryBelief(!queryPred);
    op:log(debug,"[knowsContentLocation] bindings: !bindings");

    ?return = op:isEmpty(!bindings);

    op:log(debug,"[knowsContentLocation] returning: ?return . The truth value here is the opposite of what you would expect, but it's not very easy to negate in ASL.");
}

(List ?refIds) = positContents["posits that ?count instances of an item of type ?descriptor exist at ?location"](Predicate ?contentType) {

    //observableAt(2,screw:property,table1Area:area)
    Integer !count;
    Integer !i =0;
    Symbol !prop;
    String !propName;
    Predicate !property;
    Predicate !observablePred;
    Symbol !area;
    Symbol !refID;
    List !propsList;

    ?refIds = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    !propsList= op:newArrayList("edu.tufts.hrilab.fol.Term");
    !count = op:getArg(?contentType,0); //TODO:brad what if this is the string form of a number?
    op:log(debug,"[positContents] count: !count");

    !prop = op:getArg(?contentType,1);
    !propName = op:getName(!prop);
    //TODO:brad: how do we get the type info here?
    !property = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !propName,"X:physobj");
    !area = op:getArg(?contentType,2);
    op:log(debug,"[positContents] area: !area");

    !observablePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "observableAt(!propName,!area)");
    act:assertBelief(!observablePred);

    op:add(!propsList,!property);
    op:log(debug,"[positContents] props list: !propsList");
    for (!i=0; !i lt !count; !i ++) { //TODO:brad: seemingly !i=0 doesn't actually reset !i's value?
        op:log(debug,"[positContents] in loop: !i");
        !refID = act:positReference(!propsList);
        op:log(debug,"[positContents] posited new reference: !refID");
        op:add(?refIds,!refID);
    }
    op:log(debug,"[positContents] posited !count entities each with !propsList");
}

// generates a recovery plan for a failure, runs performance assessment, and submits the plan
() = generateRecoveryPlanAndAssess(Predicate ?brokenActionPredicate, List ?failureReasons, Predicate ?goal) {

    recovery: {
      actionStatuses: {FAIL_POSTCONDITIONS}
    }

    edu.tufts.hrilab.action.ActionConstraints !ac;
    edu.tufts.hrilab.action.state.StateMachine !sm;
    edu.tufts.hrilab.action.selector.ActionSelector !actionSelector;
    edu.tufts.hrilab.action.ParameterizedAction !parameterizedAction;

    Predicate !plannedActionFull;
    Predicate !goalPred;
    edu.tufts.hrilab.action.goal.Goal !goal;
    Long !gid;
    GoalStatus !goalStatus;
    edu.tufts.hrilab.action.ActionStatus !actionStatus;
    Predicate !bradAsHumanActor;
    Symbol !planningActor;

    // use self:agent as planning actor so all diarc agents can be used in planning -- is this necessary?
    !planningActor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "self", "agent");

    !bradAsHumanActor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "object(brad, human)");
    !plannedActionFull = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "planned", !planningActor);

    Predicate !goalState;
    !goalState = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and", ?failureReasons);
    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal", !planningActor, !goalState);

    Predicate !actionDescription;

    //for talking about performance assessment;
    Variable !x = "X";
    Predicate !queryPred;
    Predicate !timeEstimatePred;
    List !paBindings;
    java.util.Map !paBinding;
    Predicate !timeEstimate;
    String !estimateString;

    // for asking if it should execute the plan
    java.util.Map !bindings;
    Predicate !shouldExecuteResponse;

    op:log(debug, "[generateRecoveryPlanAndAssess] generated goal predicate !goalPred");

    //Generate failure response so it can be said if need be.
    //If no recovery plan is found, don't say it because dialogue will.
    Predicate !reportFailurePred;
    Predicate !executionPred;
    Predicate !becausePred;
    //TODO: if there are multiple failure reasons, negate each? Maybe we could do this automatically in NLG instead
    Predicate !failureReason;
    !failureReason = op:get(?failureReasons,0);
    !becausePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "because", "and(not(!failureReason))");
    !executionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "execution(!planningActor)");
    !reportFailurePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "failed", !executionPred, !becausePred);

    //get the plan with goal ?goalPred
    !sm = tsc:getStateMachine();
    !ac = op:newObject("edu.tufts.hrilab.action.ActionConstraints");
    !actionSelector = op:invokeStaticMethod("edu.tufts.hrilab.action.selector.ActionSelector", "getInstance");

    if (op:isNull(!actionSelector)) {
        op:log(error, "[generateRecoveryPlanAndAssess] action selector is null!");
    }
    !goal = op:newObject("edu.tufts.hrilab.action.goal.Goal", !goalPred);
    !parameterizedAction = op:invokeMethod(!actionSelector, "selectActionForGoal", !goal, !ac, !sm);
    //check if there was a valid plan using diarc agents. otherwise add "brad" as an actor to the domain and resubmit, then remove brad from the domain
    if (~op:isNull(!parameterizedAction)) {
        op:log(debug, "[generateRecoveryPlanAndAssess] successfully got planned parameterizedAction for !goalPred for self");
        //do performance assessment on this goal
        act:estimatePerformanceMeasures(!plannedActionFull, before);
        op:log(debug, "[generateRecoveryPlanAndAssess] performances measures run for planned action for goal !goalPred");

        //say that it failed
        act:generateResponse(!reportFailurePred);

        //Notify human that another plan was found and report how long it would be estimated to take
        act:generateResponseFromString("I have found a plan to recover");

//        //describe the plan
//        !actionDescription = act:getActDesc(!plannedActionFull);
//        //op:log(info, "description: !actionDescription");
//        act:generateResponse(!actionDescription);

        !timeEstimatePred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "durationOf", "Z", "Y", "X");
        !queryPred =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows", !planningActor, !timeEstimatePred);
        !paBindings = act:queryBelief(!queryPred);
        !paBinding = op:get(!paBindings, 0);
        (!timeEstimate) = op:get(!paBinding, !x);
        (!estimateString) = op:getName(!timeEstimate);

        !timeEstimatePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "durationOf(!parameterizedAction,before,!estimateString)");
        act:generateResponse(!timeEstimatePred);

        //Ask user for desire to execute the found plan

        (!bindings) = act:askQuestionFromString(!planningActor,"Do you want me to execute it?", val(X));
        (!shouldExecuteResponse) = op:get(!bindings, !x);

        if (op:equalsValue(!shouldExecuteResponse,yes())) {
            //finally, execute the planned action
            !gid = act:submitGoal(!plannedActionFull);
            !goalStatus = act:joinOnGoal(!gid);
            if (op:invokeMethod(!goalStatus, "isFailure")) {
              op:log("debug", "Action failed, goal: !gid");
              !failureConditions= act:getGoalFailConditions(!gid);
              !actionStatus= act:getActionStatus(!gid);
              op:log("debug", "action status: !actionStatus");
              exit(!actionStatus, !failureConditions);
            }
            op:log(debug, "[generateRecoveryPlanAndAssess] successfully ran the planned action");
        }
        else {
            exit(FAIL);//TODO:brad: what should this fail condition be
        }

    } else {
        op:log(debug, "[generateRecoveryPlanAndAssess] failed to get valid plan for self, re-generating plan with brad added to domain");
        act:assertBelief(!bradAsHumanActor);

        !plannedActionFull = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "planned", "fetch:fetch");

        !parameterizedAction = op:invokeMethod(!actionSelector, "selectActionForGoal", !goal, !ac, !sm);

        act:retractBelief(!bradAsHumanActor);
        if (op:isNull(!parameterizedAction)) {
            op:log(error, "[generateRecoveryPlanAndAssess] failed to generate plan with human actor added to domain");
            exit(FAIL); //todo: should this be a more informative failure?
        }

        //say that it failed
        act:generateResponse(!reportFailurePred);

        //Ask user for desire to execute the found plan
        (!bindings) = act:askQuestionFromString(!planningActor,"I have found a plan to recover, but it requires your help, can you help execute it now?", val(X));
        (!shouldExecuteResponse) = op:get(!bindings, !x);

        if (op:equalsValue(!shouldExecuteResponse,yes())) {
            //finally, execute the planned action
            !gid = act:submitGoal(!plannedActionFull);
            !goalStatus = act:joinOnGoal(!gid);
            if (op:invokeMethod(!goalStatus, "isFailure")) {
              op:log("debug", "Action failed, goal: !gid");
              !failureConditions= act:getGoalFailConditions(!gid);
              !actionStatus= act:getActionStatus(!gid);
              op:log("debug", "action status: !actionStatus");
              exit(!actionStatus, !failureConditions);
            }
            op:log(debug, "[generateRecoveryPlanAndAssess] successfully ran the planned action");
        }
        else{
            exit(FAIL);//TODO:brad: what should this fail condition be
        }
    }
    act:generateResponseFromString("Recovery completed resuming original goal");
}


() = askForHelp(Symbol ?objectRef:physobj, Symbol ?propertyType:property, Symbol ?area:area) {

    // for asking if it should execute the plan
    java.util.Map !bindings;
    Predicate !shouldExecuteResponse;
    Variable !x = "X";
    String !propertyStr;
    String !locStr;

    !propertyStr = op:invokeMethod(?propertyType, "getName");
    !locStr = op:invokeMethod(?area, "getName");

    // "Could you put a bandagebox at table E"
    !bindings = act:askQuestionFromString(?actor, "Could you put a !propertyStr at !locStr ?", val(X));
    !shouldExecuteResponse = op:get(!bindings, !x);
    if (~op:equalsValue(!shouldExecuteResponse,yes())) {
      op:log("info", "?actor refused to put ?propertyType at ?area");
      exit(FAIL);
    }
}

() = describeYourPlan() {

    Symbol !plannedActionFull;
    Predicate !actionDescription;

    !plannedActionFull = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "planned", ?actor);

    !actionDescription = act:getActDesc(!plannedActionFull);
    //op:log(info, "description: !actionDescription");
    act:generateResponse(!actionDescription);
}
