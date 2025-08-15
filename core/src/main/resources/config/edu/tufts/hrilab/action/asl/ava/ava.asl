import edu.tufts.hrilab.ava.message.response.Obstacle;
import edu.tufts.hrilab.ava.message.response.Position;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceUtil;
import java.lang.Boolean;
import java.lang.Long;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.StringTokenizer;
import edu.tufts.hrilab.action.goal.Goal;

/******************************** Move Escort Deliver Scripts ********************************/

() = escortToLocation(Symbol ?escortee, Symbol ?dstLoc) {
    String !locName = tsc:getMapReferenceName(?dstLoc);

    op:log(debug, "[escortToLocation] escorting to location named !locName");

    act:say("Hello ?escortee. Please come with me, I will show you to !locName");
//    async {
//        act:say("Hello ?escortee. Please come with me, I will show you to !locName. Please say Ready when you are ready to go");
//    }
//    act:waitForReadyAck();

    act:goToLocation(?dstLoc);
    act:say("Arrived at !locName");
    act:goHome();
}

() = escortFromLocationToLocation(Symbol ?escortee, Symbol ?escorteeLoc, Symbol ?dstLoc) {
    String !locName = tsc:getMapReferenceName(?dstLoc);

    act:goToLocation(?escorteeLoc);
//    async {
        act:say("Hello ?escortee. Please come with me, I will show you to !locName."); // Please say Ready when you are ready to go");
//    }
//    act:waitForReadyAck();

    act:goToLocation(?dstLoc);
    act:say("Arrived at !locName");
    op:sleep(10000);
    act:goToLocation(?escorteeLoc);
}

/**
 * TODO;daniel; This doesn't check if the object is in the robot's possession
 * Example command: "Take this object to the lobby"
 * Probably somehow check if an object is in the robot's  possession and then once it is it can drive to the location
 */
() = deliver(Symbol ?obj, Symbol ?dstLoc) {
    act:goToLocation(?dstLoc);
    act:say("Here is ?obj");
    op:log("info", "[deliverToLocation] Delivering ?obj to ?dstLoc");
}


/**
 * This is a recovery policy executed when a parameter to a function doesn't have a known location.
 * It attempts to find the location of the thing.
 */
//() = unknownLocationRecoveryPolicy(List ?failedActionSignature, List ?failureReasons, Predicate ?failedGoal) {
//    Symbol !thing;
//
//    recovery : {
//        failedActions: {escortToLocation(X,Y,Z), lookForPerson(X,Y)}
//        failureReasons: {at(X,Y), alwaysAt(X,Y)}
//        actionStatuses: {FAIL_PRECONDITIONS}
//    }
//
//    // Get the !thing from ?failureReasons [at(!thing,!thingLoc)]
//    String !failureReasonName;
//    Predicate !failureReason;
//    List !failureReasonArgs;
//    int !failureReasonArgsSize;
//    foreach (!failureReason : ?failureReasons) {
//        !failureReasonName = op:invokeMethod(!failureReason, "getName");
//        // Check if the name is "at"
//        if (op:invokeMethod(!failureReasonName, "equals", "at")) {
//            !failureReasonArgs = op:invokeMethod(!failureReason, "getArgs");
//            !failureReasonArgsSize = op:invokeMethod(!failureReasonArgs, "size");
//            // Ensure it has the args to check for the first argument
//            if (op:gt(!failureReasonArgsSize,0)) {
//                !thing = op:invokeMethod(!failureReasonArgs, "get", 0);
//            }
//        }
//    }
//
//    // TODO;daniel; this doesn't deal with weird responses (he is at the elevator, the elevator, yes, no, i don't know!!!)
//    Variable !y = "Y";
//    Predicate !atResponseForm = op:createPredicate("wantBel(X,self,at(!thing,!y))");
//
//    // check if you understand that question
//    // check for understanding in the ask question method lolll
//    boolean !understood = false;
//    Predicate !answerSemantics;
//    Predicate !confirmationSemantics;
//    Predicate !yesSemantics = op:createPredicate("val(yes)");
//    while (op:equals(!understood, false)) {
////        tsc:setSuggestedInput("!thing is at the kitchen");
//        !answerSemantics = act:askQuestionFromString(patient, "Do you know where is !thing?", !atResponseForm);
//        tsc:displayText("yes or no");
//        !confirmationSemantics = act:askQuestionFromString(patient, "Did I understand that correctly?", "val(X)");
//        !understood = op:equals(!confirmationSemantics, !yesSemantics);
//    }
//
//    Map !bindings = op:invokeMethod(!answerSemantics, "getBindings", !atResponseForm);
//    Symbol !thingLoc = op:get(!bindings, !y);
//    Predicate !assertPred = op:createPredicate("at(!thing,!thingLoc)");
//    act:assertBelief(!assertPred);
//}

//() = unconfirmedLocationRecoveryPolicy(Predicate ?failedActionSignature, List ?failureReasons, Predicate ?failedGoal) {
//    recovery : {
//        failedActions: {escortToLocation(X,Y,Z), goToLocation(X,Y)}
//        failureReasons: {confirmed(X)}
//        actionStatuses: {FAIL_PRECONDITIONS}
//    }
//
//    String !actionName = op:invokeMethod(?failedActionSignature, "getName");
//    if (op:equals(!actionName, "escortToLocation"))
//    String !location =
//
//    Predicate !answerSemantics = act:askQuestionFromString(patient, "Are you sure you want me to !actionName", "val(X)", 1, -1, true, "yes or no");
//}

/******************************** Battery Scripts ********************************/

// Monitors charge status continuously. Calling the robot to charge at certain thresholds.
() = monitorBattery() {
    int !sleepTime = 30000; // 30s How often to check the battery
    double !currBatteryLevel;
    double !lowChargeLevel = 30;
    double !veryLowChargeLevel = 15; // Threshold to suspend tasks and go home and charge immediately
    double !enoughChargeLevel = 50; // Threshold to stop charging
    boolean !sentGoalHome = false;
    boolean !sentUrgentGoalHome = false;
    Predicate !goHomeGoalPred = op:createPredicate("goHomeAndChargeToPercent(?actor, !enoughChargeLevel)");
    Predicate !urgentGoHomeGoalPred = op:createPredicate("urgentGoHomeAndChargeToPercent(?actor, !enoughChargeLevel)");
    Long !goHomeGoalID = -1;

    // TODO;daniel; try catch the failure of going home or cancels
    // TODO;daniel; monitor and reject tasks when charging potentially?
    while (true) {
        !currBatteryLevel = act:getBatteryLevel();
//        op:log(o", "[monitorBattery] battery level: !currBatteryLevel");

        if (op:lt(!currBatteryLevel, !veryLowChargeLevel) && op:equals(!sentUrgentGoalHome, false)) {
            // Urgent case
            act:generateResponseFromString("Very low battery. Going home now");
            op:log("info", "[monitorBattery] Very low battery. Going home now");

            if (~op:==(!goHomeGoalID, -1)) {
                op:log("info", "[monitorBattery] goHomeGoalID is !goHomeGoalID");
                act:cancelGoal(!goHomeGoalID); // Ensure low urgency go home is cancelled
            }
            !goHomeGoalID = act:submitGoal(!urgentGoHomeGoalPred);

            !sentUrgentGoalHome = op:newObject("java.lang.Boolean", true);
        } elseif (op:lt(!currBatteryLevel, !lowChargeLevel) && op:equals(!sentGoalHome, false)) {
            // Non urgent case
            act:generateResponseFromString("Low battery. Going home soon");
            op:log("info", "[monitorBattery] Low battery. Going home soon");

            !goHomeGoalID = act:submitGoal(!goHomeGoalPred);
            !sentGoalHome = op:newObject("java.lang.Boolean", true);
        } elseif (op:gt(!currBatteryLevel, !enoughChargeLevel)) {
            // Reset these statuses when done charging enough
            !sentGoalHome = op:newObject("java.lang.Boolean", false);
            !sentUrgentGoalHome = op:newObject("java.lang.Boolean", false);
        }

        op:sleep(!sleepTime);
    }
}

// Wraps goHomeAndChargeToPercent but is marked with SKIPPENDING/URGENT bypassing the queue
() = urgentGoHomeAndChargeToPercent(double ?enoughChargeLevel) {
    act:goHomeAndChargeToPercent(?enoughChargeLevel);
}

// Persistent goal until charged to certain threshold
() = goHomeAndChargeToPercent(double ?enoughChargeLevel) {
    int !sleepTime = 30000;
    double !currBatteryLevel;
    !currBatteryLevel = act:getBatteryLevel();

    act:goHome();

    while (op:lt(!currBatteryLevel, ?enoughChargeLevel)) {
        !currBatteryLevel = act:getBatteryLevel();
        op:sleep(!sleepTime);
    }
}

//Indefinitely blocking, but interruptible version of goHomeAndCharge. Used for Kiosk action
() = goHomeAndCharge() {
    act:goHome();
    act:blockToCharge();
}

// is this  used?
() = reportBatteryLevel() {
    double !batteryLevel; //percentage
    Predicate !responsePredicate;

    !batteryLevel = act:getBatteryLevel();
    !responsePredicate = op:createPredicate("batteryLevel(!batteryLevel)");
    act:generateResponse(!responsePredicate);
}

() = saveLocation(Symbol ?location) {
    Variable !x;
    effects : {
        success infer : not(at(?actor, !x));
        success infer : at(?actor, ?location);
    }
    tsc:saveLocation(?location);
}

() = saveLocationsToFile() {
    tsc:saveLocationsToFile();
}

() = sit() {
    tsc:setZLift(0.0);
}

() = stand() {
    tsc:setZLift(90.0);
}

//() = fallbackRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
//    recovery : {
//        actionRecovery : {
//            catchingActions : {goToLocation(X,Y)} //don't think this line practically matters
//            postRecovery : postrecovery_retry
//        }
//        failedActions: {goToLocation(X,Y,Z)}
//        maximumAttempts : 1
////        excludedGoals : {recoverGoToLocation(X,Y,Z,A)} //If you want 0 recovery of recovery (80% sure failing the recovery goTo will exit in a way that the initial failed goTo does not retry) //TODO: requiring this feels like indication of a bug - maxAttempts > 1 with failed action involved in recovery of itself
//        excludedGoals : {goToLocation(X,Y),recoverGoToLocation(X,Y,Z,recoverGoToLocation(X,A,B,C))} //If you want 1 layer of recovery for recovery
//    }
//
//    op:log(warn, "[fallbackRecovery] executing fallback recovery for goToLocation")
//    act:generateResponseFromString("Saying excuse me on robot");
//    act:say("Excuse me");
//}

() = recoverGoToLocation(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {goToLocation(X,Y)} //don't think this line practically matters
            postRecovery : postrecovery_retry
        }
        failedActions: {goToLocation(X,Y,Z)}
        maximumAttempts : 2
        excludedGoals : {recoverGoToLocation(X,Y,Z,A)} //If you want 0 recovery of recovery (80% sure failing the recovery goTo will exit in a way that the initial failed goTo does not retry) //TODO: requiring this feels like indication of a bug - maxAttempts > 1 with failed action involved in recovery of itself
        //excludedGoals : {recoverGoToLocation(X,Y,Z,recoverGoToLocation(X,A,B,C))} //If you want 1 layer of recovery for recovery
    }
    Predicate !failedActionPredicate;
    Symbol !toLocation;

    op:log(info, "[recoveryGoToLocation] ?failedActionPredicates, ?failureReasons, ?failedGoal");
    //Can make this unpacking safer (check null/size) if possible to hit this from other contexts. Don't think you can though
    !failedActionPredicate = op:get(?failedActionPredicates,0);
    !toLocation = op:invokeMethod(!failedActionPredicate,"get",1);
    String !locName = tsc:getMapReferenceName(!toLocation);
    act:generateResponseFromString("I had an issue going to location !locName. Trying again.");
//    act:say("Excuse me");
    op:log(warn, "[recoveryGoToLocation] re-attempting goToLocation(X)");
}

() = goToLocation(Symbol ?location) {
    Symbol !current;

    effects : {
        success : at(?actor, ?location);
        success infer : not(at(?actor, !current));
        success infer : at(?actor, ?location);
        nonperf infer : not(at(?actor, !current));
        nonperf infer : at(?actor, unknown);
        failure infer : not(at(?actor, !current));
        failure infer : at(?actor, unknown);
    }
    onInterrupt : {
        cancel : act:stop();
        suspend : act:stop();
    }

    !current = act:getCurrentLocation();

    act:goToLocation(?location, !current);
}

() = goHome() {
    Symbol !homeLocation;
    onInterrupt : {
        cancel : act:stop();
        suspend : act:stop();
    }
    !homeLocation = act:getDockRefId();
    //todo: this should not be hardcoded here.
    act:goToLocation(!homeLocation);
    act:goHomePrimitive();
}

/******************************** Misc Testing Scripts ********************************/

(Symbol ?currentLocation) = getCurrentLocation() {
    List !bindings;
    Map !binding;
    Predicate !queryPred;
    Variable !x = X;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");

    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("warn", "[getCurrentLocation] locQuery returned no results");
        ?currentLocation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "unknown");
    } else {
        !binding = op:get(!bindings, 0);
        ?currentLocation = op:get(!binding, !x);
    }
}

() = describeCurrentLocation() {
    Symbol !currentLocation;

    !currentLocation = act:getCurrentLocation();
    act:generateResponse(at(?actor, !currentLocation));
}

() = learnNewTask() {
    Predicate !actionSignature;
    Predicate !pred;
    String !taskName;
    !taskName = act:askForTaskName();

    if (op:isNull(!taskName)) {
        exit(FAIL, not(supplied(taskName)));
    }
    !taskName = op:invokeMethod(!taskName, "replaceAll", " +", "");

    !actionSignature = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !taskName);
    op:log(info,!actionSignature);
    async {
        act:waitForActionLearningStart(!actionSignature);

        op:log(info, !taskName);
        act:learningPrompt(!taskName);
    }
    act:learnAction(!actionSignature);
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knowHowTo(?actor,perform(!taskName))");
    act:generateResponse(!pred);
}

() = learningPrompt(String ?taskName) {
    Predicate !pred;
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "how(?actor,perform(?taskName))");
    op:log(info,"!pred");
    act:generateResponse(!pred);
}

() = endTaskLearning() {
    java.lang.Boolean !learningStateExists = False;

    !learningStateExists = tsc:endActionLearning(?actor);
    if (op:equals(!learningStateExists,False)) {
        act:generateResponseFromString("Not currently learning");
    }
}


(String ?name) = askForTaskName() {
    Symbol !response;
    Map !bindings;
    Variable !x = "X";
    java.util.List !responseFormList;
    Predicate !responseForm;

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "taskName(X)");
    op:add(!responseFormList,!responseForm);

    (!bindings) = act:askQuestionFromString(user,"What is the name of the task?", !responseFormList);
   if (op:invokeMethod(!bindings, "isEmpty")) {
        ?name = op:setNull();
    } else {
        !response = op:get(!bindings,!x);
        ?name = op:getName(!response);
    }
}

() = say(String ?literal) {
    Symbol !listener = person; // TODO: how to set this in general?
    edu.tufts.hrilab.slug.common.Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !listener);
    edu.tufts.hrilab.slug.common.UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setWords", ?literal);

    act:toggleVoiceOn();
    act:updateDialogueHistory(!utterance);
    act:sayText(?literal);
    act:toggleVoiceOff();
}

() = describeAllKnownLocations() {
    List !locationNames;
    java.lang.String !locationName;

    !locationNames = tsc:getMapLocationNames();
    if (op:isEmpty(!locationNames)) {
        act:generateResponseFromString("There are no saved locations");
    } else {
        act:generateResponseFromString("I know the following locations");
        foreach(!locationName : !locationNames) {
            act:generateResponseFromString(!locationName);
        }
    }
}

() = describeAllKnownPatrolPaths() {
    Predicate !queryPred;
    Variable !x = "X";
    Symbol !path;
    Symbol !pathName;
    Map !elem;
    List !bindings;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(X,Y)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        act:generateResponseFromString("I know the following patrol paths");
        foreach(!elem : !bindings) {
            !path = op:get(!elem,!x);
            !pathName = op:invokeMethod(!path,"getName");
            act:generateResponseFromString(!pathName);
        }
    } else {
        act:generateResponseFromString("There are no saved patrol paths");
    }
}

() = describeAllLocationsOnPatrolPath(Symbol ?pathName) {
    Predicate !queryPred;
    Variable !x = "X";
    Map !elem;
    List !bindings;
    Predicate !locationsPredicate;
    Predicate !generateResponsePredicate;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?pathName,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     !locationsPredicate = op:get(!elem,!x);
     !generateResponsePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "patrolPath(?pathName,contains(!locationsPredicate))");
     act:generateResponse(!generateResponsePredicate);
    } else {
        act:generateResponseFromString("I don't have a patrol path saved under the name ?pathName");
    }
}

() = forgetTaughtAction(Predicate ?actionSignature) {
    edu.tufts.hrilab.action.justification.Justification !actionJustification;
    op:log("debug","[forgetTaughtAction] forgetting ?actionSignature");

    !actionJustification = tsc:forgetTaughtAction(?actionSignature);
    if (op:invokeMethod(!actionJustification, "getValue")) {
        op:log("debug","[forgetTaughtAction] success");
    } else {
        exit(FAIL, !actionJustification);
    }
}

() = reportTaughtActions() {
    List !taughtActions;
    Predicate !actionPred;
    Integer !numTaught;
    Predicate !reportedPred;

    !taughtActions = tsc:getLearnedActionPredicates();
    if (op:isEmpty(!taughtActions)) {
        act:generateResponseFromString("I have not been taught any new actions");
    } else {
        !numTaught = op:invokeMethod(!taughtActions,"size");
        act:generateResponseFromString("I have been taught !numTaught new actions.");
        foreach (!actionPred : !taughtActions) {
            !reportedPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "iHaveBeenTaughtTo(!actionPred)");
            act:generateResponse(!reportedPred);
        }
    }
}

() = reportPredefinedActions() {
    List !instructableActions;
    Predicate !reportedPredicate;
    Integer !numPredefined;
    String !actionName;
    Map !bindings;
    Variable !x = "X";
    Symbol !confirmation;
    String !numToReportString;
    Integer !numToReport;
    Integer !numReported = 0;

    !instructableActions = tsc:getInstructableActions();
    if (op:isEmpty(!instructableActions)) {
        exit(FAIL,retrieve(exposedActionList));
    } else {
        !numPredefined = op:invokeMethod(!instructableActions,"size");
        !bindings = act:askQuestionFromString(user,"I have !numPredefined predefined actions. Would you like me to start listing them out?", val(X));
        if (op:invokeMethod(!bindings,"isEmpty")) {
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
        !confirmation = op:get(!bindings, !x);
        if (op:equalsValue(!confirmation, yes)) {
            !bindings = act:askQuestionFromString(user,"How many should I list at a time?", amount(X));
            if (op:invokeMethod(!bindings,"isEmpty")) {
                exit(FAIL,not(wasAnswered(myQuestion)));
            }
            !numToReportString = op:get(!bindings, !x);
            if (op:equalsValue(!numToReportString,"all")) {
                !numToReport = op:newObject("java.lang.Integer", !numPredefined);
            } else {
                !numToReport = op:newObject("java.lang.Integer", !numToReportString);
            }
            if (op:gt(!numToReport,0)) {
                foreach (!actionName : !instructableActions) {
                    if (op:equals(!numReported,!numToReport)) {
                        !bindings = act:askQuestionFromString(user,"Would you like me to keep going?", val(X));
                        if (op:invokeMethod(!bindings,"isEmpty")) {
                            exit(FAIL,not(wasAnswered(myQuestion)));
                        }
                        !confirmation = op:get(!bindings, !x);
                        if (op:equalsValue(!confirmation, yes)) {
                            !numReported = op:newObject("java.lang.Integer", 0);
                        } else {
                            //weird hack to get desired control flow ('break' from loop)
                            !numReported = op:++(!numReported);
                        }
                    } elseif (op:lt(!numReported,!numToReport)) {
                        !reportedPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "iKnowHowTo(!actionName)");
                        act:generateResponse(!reportedPredicate);
                        !numReported = op:++(!numReported);
                    }
                }
                act:generateResponseFromString("I am done listing the predefined actions I can be instructed to perform");
            } else {
            act:generateResponseFromString("Sorry, I am exiting this conversation context. Please try again and supply a positive number");
            }
        } else {
            act:generateResponseFromString("Okay, not listing any predefined actions");
        }
    }
}

//TODO: LLM only - fail gracefully if no LLM present
() = reportInstructableActions() {
    Integer !numTaught;
    Map !bindings;
    Symbol !type;
    Variable !x = "X";
    Boolean !validResponse = false;
    List !systemGoals;
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;

    !bindings = act:askQuestionFromString(user,"Would you like me to report all of the actions I have been taught, my predefined actions, or both?", actionType(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }

    !type = op:get(!bindings, !x);
    if (op:equalsValue(!type,"taught") || op:equalsValue(!type,"both")) {
        !validResponse = op:newObject("java.lang.Boolean", true);
        act:reportTaughtActions();
    }
    if (op:equalsValue(!type,"predefined") || op:equalsValue(!type,"both")) {
        !validResponse = op:newObject("java.lang.Boolean", true);
        !systemGoals = tsc:getSystemGoals();
        if (op:isEmpty(!systemGoals)) {
            !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reportPredefinedActions(?actor)");
            !goalID = act:submitGoal(!state);
            !goalStatus = act:joinOnGoal(!goalID);
        } else {
            //TODO: If goal is suspended, submit goal with high priority?
                  //If goal is active, report failure to report?
            act:generateResponseFromString("I cannot list all predefined actions while performing another task, please try again when I am not busy");
        }
    }
    if (op:equals(!validResponse,false)) {
        exit(FAIL,not(understood(responseToMyQuestion)));
    }
}

//TODO: how to split these two by language?
//      (Currently commenting out other form and softly allowing LLM to provide unpopulated semantics and handling in postprocessing)
//TODO: LLM only - fail gracefully if no LLM present
//Wrapper to adhere to queueEM
() = describeActionSteps(Predicate ?action) {
    Goal !goal;
    edu.tufts.hrilab.action.selector.ActionSelector !actionSelector;
    edu.tufts.hrilab.action.ParameterizedAction !parameterizedAction;
    Predicate !actionDescription;
    edu.tufts.hrilab.action.ActionConstraints !ac;
    edu.tufts.hrilab.action.state.StateMachine !sm;
    java.util.List !eventSpecs;
    Predicate !stepPredicate;
    java.util.List !actDescArgs;
    Predicate !failCond;
    Predicate !desc;
    Boolean !isTaughtAction;

    !isTaughtAction = tsc:isLearnedAction(?action);

    if (op:equals(!isTaughtAction,true)) {
        // get StateMachine for use in action selection
        !sm = tsc:getStateMachine();

        // pick action, and get event specs from selected action
        !goal = op:invokeStaticMethod("edu.tufts.hrilab.action.goal.Goal", "createGoal", ?action);
        !ac = op:newObject("edu.tufts.hrilab.action.ActionConstraints");
        !actionSelector = op:invokeStaticMethod("edu.tufts.hrilab.action.selector.ActionSelector", "getInstance");
        !parameterizedAction = op:invokeMethod(!actionSelector, "selectActionForGoal", !goal, !ac, !sm);

        // do null check for action
        if (op:isNull(!parameterizedAction)) {
            !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,how(?action)))");
            exit(FAIL, !failCond);
        }

        // construct final predicate to assert to belief
        !actDescArgs = op:newObject("java.util.ArrayList");
        !oldDescArgs = op:newObject("java.util.ArrayList");

        // build and assert final predicate to belief
        !stepPredicate = op:invokeMethod(!parameterizedAction, "getStepsInPredicateForm");
        op:invokeMethod(!actDescArgs, "add", ?action);
        op:invokeMethod(!actDescArgs, "add", !stepPredicate);
        !actionDescription = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionDescription", !actDescArgs);
        !desc = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionDescription", !actDescArgs);
        act:generateResponse(!desc);
    } else {
        act:generateResponseFromString("As of right now, I can only describe the actions which have been taught to me by a user. For predefined actions, please refer to my user guide");
    }
}

//() = describeActionSteps(String ?actionName) {
//    //TODO: LLM only - fail gracefully if no LLM present
//
//}

//Wrapper to adhere to queueEM
() = reportIfCanPerform(Predicate ?actionPredicate) {
    String !actionName;
    Boolean !hasArgs;
    Boolean !canPerform = false;

    op:log("info","[reportIfCanPerform] ?actionPredicate");
    //Workaround for inconsistent LLM semantics when specifying a general action name rather than signature
    !hasArgs = op:invokeMethod(?actionPredicate,"hasArgs");
    if (op:equals(!hasArgs,true)) {
        op:log("info","[reportIfCanPerform] predicate has args");
        if (tsc:actionExists(?actionPredicate)) {
            op:log("info","[reportIfCanPerform] action found to exist");
            !canPerform = op:newObject("java.lang.Boolean", true);
        }
    } else {
        !actionName = op:getName(?actionPredicate);
        op:log("info","[reportIfCanPerform] predicate has no args");
        if (tsc:actionExists(!actionName)) {
            op:log("info","[reportIfCanPerform] action found to exist");
            !canPerform = op:newObject("java.lang.Boolean", true);
        }
    }

    if (op:equals(!canPerform,true)) {
      act:generateResponseFromString("Yes");
      act:generateResponse(knowHow(?actionPredicate));
    } else {
      act:generateResponseFromString("No");
      act:generateResponse(not(knowHow(?actionPredicate)));
    }
}

() = learnTourActionOnCancel(Integer ?tracker, Predicate ?learnedActionPred) {
    if (op:gt(?tracker,0)) {
        tsc:cancelActionLearning(?actor,?learnedActionPred);
    }
    if (op:equals(?tracker,2)) {
        tsc:removeAllowInstructionsDuringQAForAgent(?actor);
    }
}

() = learnTourActionOnSuspend(Integer ?tracker, Predicate ?learnedActionPred) {
    if (op:gt(?tracker,0)) {
        tsc:pauseActionLearning(?actor,?learnedActionPred);
    }
    if (op:equals(?tracker,2)) {
        tsc:removeAllowInstructionsDuringQAForAgent(?actor);
    }
}

() = learnTourActionOnResume(Integer ?tracker, Predicate ?learnedActionPred) {
    if (op:gt(?tracker,0)) {
        tsc:resumeActionLearning(?actor,?learnedActionPred);
    }
    if (op:equals(?tracker,2)) {
        tsc:allowInstructionsDuringQAForAgent(?actor);
    }
}

//TODO: can definitely clean this code up a bit
() = learnTourAction() {
    onInterrupt :{
        cancel: act:learnTourActionOnCancel(!tracker,!actionPred);
        suspend: act:learnTourActionOnSuspend(!tracker,!actionPred);
        resume: act:learnTourActionOnResume(!tracker,!actionPred);
    }
    Map !bindings;
    Variable !x = "X";
    java.util.List !responseFormList;
    Predicate !responseForm;
    Symbol !responseBinding;
    Predicate !stepPredicate;
    String !tourName;
    Predicate !actionPred;
    Symbol !location;
    Integer !tracker = 0;

    //////////// Question Asking for tour name ////////////
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("taskName(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString(user,"What would you like the action to go on this upcoming tour sequence be called?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !responseBinding = op:get(!bindings, !x);
    !tourName = op:getName(!responseBinding);
    //TODO: what had caused this to be necessary? Was it ASL parser not being comfortable with action names with spaces?
    !tourName = op:invokeMethod(!tourName, "replaceAll", " +", "");


    //////////// Question Asking for tour's initial location ////////////
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("location(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString(user,"Where would you like the tour to begin?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !responseBinding = op:get(!bindings, !x);
    if (op:equalsValue(!responseBinding,"none")) {
        exit(FAIL,not(wasSupplied(anyLocation)));
    }


    //////////// Start action learning and add initial go to location step based on initial location provided ////////////
    // Q: Need to make all introspection actions ignore action learning? Should all actions which skip the queue not be learning steps?
             //I think yes, the only thing which makes this weird is the fact that itk and 'describeX' variations of the same action has no distinction currently
    !actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !tourName);
    noInterrupt {
        async {
            act:learnAction(!actionPred);
        }
        !tracker = op:newObject("java.lang.Integer", 1);
    }
    act:waitForActionLearningStart(!actionPred);
    !stepPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(?actor,!responseBinding)");
    act:submitGoal(!stepPredicate);
    act:pauseActionLearning(!actionPred);

    //////////// Question Asking for whether the user would like to do anything at the initial location ////////////
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("val(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString(user,"Do you want me to perform an action at the starting location?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        act:cancelActionLearning(!actionPred);
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !responseBinding = op:get(!bindings, !x);

    //////////// Question Asking for action at the initial location ////////////
    if (op:equalsValue(!responseBinding,"yes")) {
        !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
        !responseForm = op:createPredicate("taskPredicate(X)");
        op:add(!responseFormList,!responseForm);
        async {
            act:generateResponseFromString("What would you like me to do there?");
        }
        act:resumeActionLearning(!actionPred);
        noInterrupt {
            tsc:allowInstructionsDuringQAForAgent(?actor);
            !tracker = op:newObject("java.lang.Integer", 2);
        }
        !bindings = act:waitForLearningGoalSubmissionOrQuestionAnswer(?actor, "What would you like me to do there?", !responseFormList, !actionPred);
        if (op:isNull(!bindings)) {
            act:cancelActionLearning(!actionPred);
            tsc:removeAllowInstructionsDuringQAForAgent(?actor);
            exit(FAIL,occurred(unknownError));
        } elseif (~op:invokeMethod(!bindings,"isEmpty")) {
            //QA answered - submit answered action predicate to add to learning state
            !stepPredicate = op:get(!bindings,!x);
            act:submitGoal(!stepPredicate);
        }
        //else goal submitted as direct action instruction (Question asking is cleaned up by java action in this case)
        noInterrupt {
            tsc:removeAllowInstructionsDuringQAForAgent(?actor);
            !tracker = op:newObject("java.lang.Integer", 1);
        }
        // Pause action learning when doing next location QA
        act:pauseActionLearning(!actionPred);
    }


    //////////// Start general loop - 1. QA for next location 2. QA for action at location ////////////
    while (~op:equalsValue(!responseBinding,"none")) {
        //////////// 1. QA for next location on the path ////////////
        !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
        !responseForm = op:createPredicate("location(X)");
        op:add(!responseFormList,!responseForm);
        !bindings = act:askQuestionFromString(user,"What is the next location along the tour path?", !responseFormList);
        if (op:invokeMethod(!bindings,"isEmpty")) {
            act:cancelActionLearning(!actionPred);
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
        !responseBinding = op:get(!bindings, !x);

        //////////// 2. QA for action to perform at that location ////////////
        if (~op:equalsValue(!responseBinding,"none")) {
            // Resume action learning
            act:resumeActionLearning(!actionPred);

            // Add go to location step
            !stepPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(?actor,!responseBinding)");
            act:submitGoal(!stepPredicate);

            // QA for action
            !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
            !responseForm = op:createPredicate("taskPredicate(X)");
            op:add(!responseFormList,!responseForm);
            async {
                act:generateResponseFromString("What would you like me to do there?");
            }
            noInterrupt {
                !tracker = op:newObject("java.lang.Integer", 2);
                tsc:allowInstructionsDuringQAForAgent(?actor);
            }
            !bindings = act:waitForLearningGoalSubmissionOrQuestionAnswer(?actor, "What would you like me to do there?", !responseFormList, !actionPred);
            if (op:isNull(!bindings)) {
                act:cancelActionLearning(!actionPred);
                tsc:removeAllowInstructionsDuringQAForAgent(?actor);
                exit(FAIL,occurred(unknownError));
            } elseif (~op:invokeMethod(!bindings,"isEmpty")) {
                //QA answered - submit answered action predicate to add to learning state
                !stepPredicate = op:get(!bindings,!x);
                act:submitGoal(!stepPredicate);
            }
            //else goal submitted as direct action instruction (Question asking is cleaned up by java action in this case)
            noInterrupt {
                !tracker = op:newObject("java.lang.Integer", 1);
                tsc:removeAllowInstructionsDuringQAForAgent(?actor);
            }

            // Pause action learning
            act:pauseActionLearning(!actionPred);
        }
    }

    //Broke out of teaching loop - should not actively be in action learning and should be done teaching
    //// Commit learning state to learned action ////
    act:resumeActionLearning(!actionPred);
    act:endActionLearning(!actionPred);
    act:generateResponseFromString("I have learned the !tourName tour sequence. You can ask me to perform this by saying \"please perform !tourName\"");
}