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
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "val(X)";
    String !locName = tsc:getMapReferenceName(?dstLoc);

    op:log(debug, "[escortToLocation] escorting to location named !locName");

//    act:say("Hello ?escortee. Please come with me, I will show you to !locName");
//    async {
//        act:say("Hello ?escortee. Please come with me, I will show you to !locName. Please say Ready when you are ready to go");
//    }
//    act:waitForReadyAck();

    //TODO: what if val(no is returned??
    act:askQuestionFromString("Hello ?escortee. Please come with me, I will show you to !locName. Please say Hey Ava, I'm Ready when you are ready to go", !waitForAckForm, 1, 300);
    act:say("Okay, please follow me");

    act:goToLocation(?dstLoc);
    act:say("Arrived at !locName");
    //act:goHome();
}

() = escortFromLocationToLocation(Symbol ?escortee, Symbol ?escorteeLoc, Symbol ?dstLoc) {
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "val(X)";

    String !locName = tsc:getMapReferenceName(?dstLoc);

    act:goToLocation(?escorteeLoc);
////    async {
//        act:say("Hello ?escortee. Please come with me, I will show you to !locName. Please say Ready when you are ready to go");
////    }
////    act:waitForReadyAck();
    act:askQuestionFromString("Hello ?escortee. Please come with me, I will show you to !locName. Please say Hey Ava, I'm Ready when you are ready to go", !waitForAckForm, 1, 300);
    act:say("Okay, please follow me");

    act:goToLocation(?dstLoc);
    act:say("Arrived at !locName");
    //op:sleep(10000);
    //act:goToLocation(?escorteeLoc);
}

() = follow() {
    act:generateResponseFromString("Following you until you tell me to stop");
    act:avaFollow();
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
    java.lang.Boolean !docked = False;

    !docked = act:isDocked();

    if (op:equals(!docked, False)) {
        act:goHome();
        act:blockToCharge();
    }
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
            catchingActions : {goToLocation(X,Y)}
            postRecovery : postrecovery_retry
        }
        //failedActions: {goToLocation(X,Y,Z)}
        failedActions: {goToLocation(X,Y)}
        maximumAttempts : 1 //TODO: there is an issue here with recovery occurring more times in a row than desired because
                            //      the failure information doesn't match because the failed actions list doesn't match
                            //      (the 'currentLocation' in the goToLocation(actor,destination,current) doesn't match)
                            //      I believe the fix is, when doing failure information equality while detecting duplicate failures,
                            //      to only check for equality between the actually matched fields within lists of reasons?)
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
        failure infer : not(at(?actor, !current));
    }
    onInterrupt : {
        cancel : act:stop();
        suspend : act:stop();
    }

    !current = act:getCurrentLocation();

    // with the same place in both slots the two-arg primitive can never verify: its observable success
    // effects include at(?location) and not(at(!current)), and the engine cannot observe both true for one
    // place. skip the drive; this action's own at(?actor, ?location) effect still verifies we are there
    if (op:equals(!current, ?location)) {
        op:log("info", "[goToLocation] already at ?location");
    } else {
        act:goToLocation(?location, !current);
    }
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
    // always answered from the pose, never from belief: belief only changes at action boundaries, so it lags
    // or lies during and after motion. the observation also keeps belief in step for everything that reads it
    if (~obs:at(?actor, ?currentLocation)) {
        op:log("info", "[getCurrentLocation] not at any known location");
        ?currentLocation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "unknown");
    }
}

() = learnNewTaskOnResume(Integer ?tracker, String ?taskName, Predicate ?actionSignature) {
    //if (op:gt(?tracker,1)) {
    //    tsc:resumeActionLearning(?actor,?actionSignature);
    //}
    if (op:equals(?tracker,3)) {
        act:learningPrompt(?taskName);
    }
}

//() = learnNewTaskOnSuspend(Integer ?tracker, String ?taskName, Predicate ?actionSignature) {
//    if (op:gt(?tracker,1)) {
//        tsc:pauseActionLearning(?actor,?actionSignature);
//    }
//}
//
//() = learnNewTaskOnCancel(Integer ?tracker, String ?taskName, Predicate ?actionSignature) {
//    if (op:gt(?tracker,1)) {
//        tsc:cancelActionLearning(?actor,?actionSignature);
//    }
//}

() = learnNewTask() {
    onInterrupt : {
        resume: act:learnNewTaskOnResume(!tracker, !taskName, !actionSignature);
        //suspend: act:learnNewTaskOnSuspend(!tracker, !taskName, !actionSignature);
        //cancel: act:learnNewTaskOnCancel(!tracker, !taskName, !actionSignature);
    }
    Predicate !actionSignature;
    Predicate !pred;
    String !taskName;
    Integer !tracker = 0;
    java.util.List !responseFormList;
    Predicate !responseForm;
    Symbol !response;
    Map !bindings;
    Variable !x = "X";

    !taskName = act:askForTaskName();

    if (op:isNull(!taskName)) {
        exit(FAIL, not(supplied(taskName)));
    }
    !taskName = op:invokeMethod(!taskName, "replaceAll", " +", "");
    !tracker = op:++(!tracker);

    !actionSignature = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !taskName);
    op:log(info,!actionSignature);
    async {
        act:learnAction(!actionSignature);
    }
    act:waitForActionLearningStart(!actionSignature);
    !tracker = op:++(!tracker);

    op:log(info, !taskName);
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("taskPredicate(X)");
    op:add(!responseFormList,!responseForm);
    act:learningPrompt(!taskName);
    !tracker = op:++(!tracker);

    !bindings = act:waitForResponse("how do you perform the task", !responseFormList, ?actor);
    while (~op:invokeMethod(!bindings,"isEmpty")) {
        !response = op:get(!bindings,!x);
        if (op:equalsValue(!response,"none")) {
            !bindings = op:newObject("java.util.HashMap");
        } else {
            act:submitGoal(!response);
            act:generateResponseFromString("Okay, that step was added to the task being learned");
            !bindings = act:waitForResponse("how do you perform the task", !responseFormList, ?actor);
        }
    }

    act:endActionLearning(!actionSignature);
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

    (!bindings) = act:askQuestionFromString("What is the name of the task?", !responseFormList, 1, 300);
   if (op:isNull(!bindings) || op:invokeMethod(!bindings, "isEmpty")) {
        ?name = op:setNull();
    } else {
        !response = op:get(!bindings,!x);
        ?name = op:getName(!response);
    }
}

() = skippendingsay(String ?literal) {
    act:say(?literal);
}

() = sayLocally(String ?literal) {
    act:say(?literal);
}

() = say(String ?literal) {
    //Symbol !listener = this:getOriginalSubmitter();
    Symbol !listener = local_user;
    edu.tufts.hrilab.slug.common.Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !listener);
    edu.tufts.hrilab.slug.common.UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setWords", ?literal);

    act:toggleVoiceOn();
    act:updateDialogueHistory(!utterance);
    act:sayText(!utterance);
    act:toggleVoiceOff();
}

() = learnTourActionOnCancel(Integer ?tracker, Predicate ?learnedActionPred) {
    if (op:gt(?tracker,0)) {
        tsc:cancelActionLearning(?actor,?learnedActionPred);
    }
}

() = learnTourActionOnSuspend(Integer ?tracker, Predicate ?learnedActionPred) {
    if (op:gt(?tracker,0)) {
        tsc:pauseActionLearning(?actor,?learnedActionPred);
    }
}

() = learnTourActionOnResume(Integer ?tracker, Predicate ?learnedActionPred) {
    if (op:gt(?tracker,0)) {
        tsc:resumeActionLearning(?actor,?learnedActionPred);
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
    Symbol !submitter;
    !submitter = this:getOriginalSubmitter();

    //////////// Question Asking for tour name ////////////
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("taskName(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString("What would you like the action to go on this upcoming tour sequence to be called?", !responseFormList, 1, 300);
    if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !responseBinding = op:get(!bindings, !x);
    !tourName = op:getName(!responseBinding);
    //TODO: asl parser cannot handle action names with spaces, extend parser or do something like this?
    !tourName = op:invokeMethod(!tourName, "replaceAll", " +", "_");

    //////////// Question Asking for tour's initial location ////////////
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("location(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString("Where would you like the tour to begin?", !responseFormList, 1, 300);
    if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
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

    //////////// Question Asking for whether the user would like to do anything at the initial location ////////////
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("val(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("taskPredicate(X)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString("Do you want me to perform an action at the starting location?", !responseFormList, 1, 300);
    if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
        act:cancelActionLearning(!actionPred);
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !responseBinding = op:get(!bindings, !x);

    //////////// Question Asking for action at the initial location ////////////
    if (~op:equalsValue(!responseBinding,"no")) {
        if (op:equalsValue(!responseBinding,"yes")) {
            !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
            !responseForm = op:createPredicate("taskPredicate(X)");
            op:add(!responseFormList,!responseForm);
            !bindings = act:askQuestionFromString("What would you like me to do there?", !responseFormList, 1, 300);
        }
        if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
            act:cancelActionLearning(!actionPred);
            exit(FAIL,not(wasAnswered(myQuestion)));
        } else {
            //QA answered - submit answered action predicate to add to learning state
            !stepPredicate = op:get(!bindings,!x);
            act:submitGoal(!stepPredicate);
        }
    }


    //////////// Start general loop - 1. QA for next location 2. QA for action at location ////////////
    while (~op:equalsValue(!responseBinding,"none")) {
        //////////// 1. QA for next location on the path ////////////
        !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
        !responseForm = op:createPredicate("location(X)");
        op:add(!responseFormList,!responseForm);
        !bindings = act:askQuestionFromString("What is the next location along the tour path?", !responseFormList, 1, 300);
        if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
            act:cancelActionLearning(!actionPred);
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
        !responseBinding = op:get(!bindings, !x);

        //////////// 2. QA for action to perform at that location ////////////
        if (~op:equalsValue(!responseBinding,"none")) {
            // Add go to location step
            !stepPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(?actor,!responseBinding)");
            act:submitGoal(!stepPredicate);

            // QA for action
            !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
            !responseForm = op:createPredicate("taskPredicate(X)");
            op:add(!responseFormList,!responseForm);
            !bindings = act:askQuestionFromString("What would you like me to do there?", !responseFormList, 1, 300);
            if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
                act:cancelActionLearning(!actionPred);
                exit(FAIL,not(wasAnswered(myQuestion)));
            } else {
                //QA answered - submit answered action predicate to add to learning state
                !stepPredicate = op:get(!bindings,!x);
                act:submitGoal(!stepPredicate);
            }
        }
    }

    //// Commit learning state to learned action ////
    act:endActionLearning(!actionPred);
    act:generateResponse("and(learnedTourSequence(!tourName),canAskToPerformBySaying(pleasePerform,!tourName))");
    //act:generateResponseFromString("I have learned the !tourName tour sequence. You can ask me to perform this by saying \"please perform !tourName\"");
}

() = updateKioskTimeout() {
    Map !bindings;
    Variable !x = "X";
    Variable !y = "Y";
    Symbol !time;
    Symbol !addl;
    Boolean !success;

    !bindings = act:askQuestionFromString("How long should the inactivity timeout be?", relativeTime(X,Y), 1, 300);
    if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !time = op:get(!bindings, !x);
    !addl = op:get(!bindings, !y);

    !success = tsc:setKioskTimeout(!time,!addl);
    if (op:equals(!success,true)) {
        act:generateResponseFromString("Okay, I have updated the kiosk timeout");
    } else {
        exit(FAIL,notParse(suppliedTime));
    }
}

() = updateKioskAction() {
    Map !bindings;
    Variable !x = "X";
    Predicate !goalPredicate;

    !bindings = act:askQuestionFromString("What should the new kiosk action be?", taskPredicate(X), 1, 300);
    if (op:isNull(!bindings) || op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !goalPredicate = op:get(!bindings, !x);
    tsc:setKioskGoal(!goalPredicate);
    act:generateResponseFromString("Okay, I have updated the kiosk action");
}

() = cancelSingleTaskInQueue(Integer ?index) {
    Boolean !success;

    !success = act:cancelPendingGoalByIndex(?index);
    if (op:equals(!success,false)) {
        act:generateResponseFromString("I could not find a task in the queue at the supplied index");
    } else {
        act:generateResponseFromString("I have removed the task from the queue.");
    }
}

//() = toggleDisplay() {
//    Predicate !ww_and_asr_goal;
//    Boolean !haveDisplay;
//
//    !ww_and_asr_goal = op:createPredicate("listenForWWandASR(?actor)");
//    !haveDisplay = tsc:getHaveDisplay();
//    if (op:equalsValue(!haveDisplay,true)) {
//        try {
//            op:log("debug","[toggleDisplay] suspending ww and asr goal");
//            act:suspendGoalBlocking(!ww_and_asr_goal);
//        } catch(FAIL) {
//            op:log("warn","[toggleDisplay] could not suspend listenForWWandASR");
//        }
//    } else {
//        try {
//            op:log("debug","[toggleDisplay] resuming ww and asr goal");
//            act:resumeGoal(!ww_and_asr_goal);
//        } catch(FAIL) {
//            op:log("warn","[toggleDisplay] could not resume listenForWWandASR");
//        }
//    }
//    op:log("debug","[toggleDisplay] calling tsc");
//    tsc:toggleDisplay();
//}

() = switchDisplayToTelepresence() {
    //TODO: is it technically unsafe if this action gets canceled/suspended before its first step that !ww_and_asr_goal is null?
    onInterrupt : {
        resume: act:suspendGoalBlocking(!ww_and_asr_goal);
        cancel: act:resumeGoal(!ww_and_asr_goal);
        suspend: act:resumeGoal(!ww_and_asr_goal);
    }
    Predicate !ww_and_asr_goal = op:createPredicate("listenForWWandASR(?actor)");

    try {
        op:log("debug","[switchDisplayToTelepresence] suspending ww and asr goal");
        act:suspendGoalBlocking(!ww_and_asr_goal);
    } catch(FAIL) {
        op:log("warn","[switchDisplayToTelepresence] could not suspend listenForWWandASR");
    }

    op:log("debug","[switchDisplayToTelepresence] calling tsc");
    act:switchDisplayToTelepresencePrimitive();

    noInterrupt {
        try {
            op:log("debug","[switchDisplayToTelepresence] resuming ww and asr goal");
            act:resumeGoal(!ww_and_asr_goal);
        } catch(FAIL) {
            op:log("warn","[switchDisplayToTelepresence] could not resume listenForWWandASR");
        }
    }
}