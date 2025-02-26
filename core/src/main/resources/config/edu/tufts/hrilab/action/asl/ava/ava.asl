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

/******************************** Move Escort Deliver Scripts ********************************/

() = escortToLocation(Symbol ?escortee, Symbol ?dstLoc) {
    Symbol !current;

    conditions : {
       pre infer : not(at(?actor, unknown));
    }

    !current = act:getCurrentLocation();
    act:escortToLocation(?escortee, !current, ?dstLoc);
}

() = escortToLocation(Symbol ?escortee, Symbol ?escorteeLoc, Symbol ?dstLoc) {
    // always forget the location of the person escorting
    effects : {
        always infer : not(at(?escortee, ?escorteeLoc));
    }

    Predicate !assertPred = op:createPredicate("at(?escortee,?escorteeLoc)");
    act:assertBelief(!assertPred);

    act:goToLocation(?escorteeLoc);
    act:sayText("Please come with me");

    // Check if the person is not at the robot
    Predicate !queryPred = op:createPredicate("at(?escortee,robot)");
    Predicate !escortConfirmationPred = op:createPredicate("ack(i am here to escort you to,?dstLoc)");
    List !beliefs = act:queryBelief(!queryPred);

    Predicate !answerSemantics;
    Predicate !yesSemantics = op:createPredicate("val(yes)");
    Predicate !noSemantics = op:createPredicate("val(no)");
    if (op:invokeMethod(!beliefs, "isEmpty")) {
        act:lookForPerson(?escortee);
        act:generateResponse(!escortConfirmationPred);
//        tsc:setSuggestedInput("yes or no");
        !answerSemantics = act:askQuestionFromString(patient, "Is that okay", !yesOrNoResponseForm);
        if (op:equals(!answerSemantics, !yesSemantics)) {
            op:log("info", "Given yes");
            // Continue
        } elseif (op:equals(!answerSemantics, !noSemantics)) {
            op:log("info", "Given no");
            act:generateResponseFromString("Let me know if you need assistance later.");
            op:log("info", "?escortee does not want to be escorted to ?dstLoc");
            exit(CANCEL);
        } else {
            op:log("info", "Given something else");
            // TODO;daniel; Doesn't handle unexpected replies that are not yes or no
        }
    }

    Predicate !pleaseFollowPred = op:createPredicate("ack(please follow me to,?dstLoc)");
    op:log("debug", "before generating response");
    act:generateResponse(!pleaseFollowPred);

    op:log("debug", "going to start going to location bruh");
    act:goToLocation(?dstLoc); //TODO;daniel; make sure that this uses the more complicated go to location with person following code and update the persons destination location (also should the robot generally try to forget where otehr things are)

    Predicate !arrivalPred = op:createPredicate("ack(here is,?dstLoc)");
    act:generateResponse(!arrivalPred);

    op:log("info", "[escortToLocation] Escorting ?escortee from ?escorteeLoc to ?dstLoc");
}
/**
 * TODO;daniel; This doesn't check if the object is in the robot's possession
 * Example command: "Take this object to the lobby"
 * Probably somehow check if an object is in the robot's  possession and then once it is it can drive to the location
 */
() = deliverToLocation(Symbol ?obj, Symbol ?dstLoc) {
    act:goToLocation(?dstLoc);
    op:log("info", "[deliverToLocation] Delivering ?obj to ?dstLoc");
}

() = lookForPerson(Symbol ?person) {
    Symbol !personLoc;

    conditions : {
        pre : at(?person, !personLoc);
    }

    Predicate !iWillFindPred = op:createPredicate("ack(i will find ?person at,!personLoc)");
    act:generateResponse(!iWillFindPred);

    act:goToLocation(!personLoc);

    // Check if the person is not at the robot find the person and confirm that they can go with you
    Predicate !yesSemantics = op:createPredicate("val(yes)");
    Predicate !noSemantics = op:createPredicate("val(no)");
//    tsc:setSuggestedInput("yes or no");
    Predicate !answerSemantics = act:askQuestionFromString(patient, "Hello, I am looking for ?person. Are you here?", !yesOrNoResponseForm);

    Predicate !retractPred = op:createPredicate("at(?person,!x)");
    if (op:equals(!answerSemantics, !yesSemantics)) {
        op:log("info", "Given yes");
        // Person is found function can be completed
    } elseif (op:equals(!answerSemantics, !noSemantics)) {
        op:log("info", "Given no");
        // This will cause the unknownLocationRecoveryPolicy to be called
        act:retractBelief(!retractPred);
        act:lookForPerson(?person);
    } else {
        op:log("info", "Given something else");
        // This will cause the question to be asked again
        act:lookForPerson(?person);
    }
}

/**
 * This is a recovery policy executed when a parameter to a function doesn't have a known location.
 * It attempts to find the location of the thing.
 */
() = unknownLocationRecoveryPolicy(List ?failedActionSignature, List ?failureReasons, Predicate ?failedGoal) {
    Symbol !thing;

    recovery : {
        failedActions: {escortToLocation(X,Y,Z), lookForPerson(X,Y)}
        failureReasons: {at(X,Y), alwaysAt(X,Y)}
        actionStatuses: {FAIL_PRECONDITIONS}
    }

    // Get the !thing from ?failureReasons [at(!thing,!thingLoc)]
    String !failureReasonName;
    Predicate !failureReason;
    List !failureReasonArgs;
    int !failureReasonArgsSize;
    foreach (!failureReason : ?failureReasons) {
        !failureReasonName = op:invokeMethod(!failureReason, "getName");
        // Check if the name is "at"
        if (op:invokeMethod(!failureReasonName, "equals", "at")) {
            !failureReasonArgs = op:invokeMethod(!failureReason, "getArgs");
            !failureReasonArgsSize = op:invokeMethod(!failureReasonArgs, "size");
            // Ensure it has the args to check for the first argument
            if (op:gt(!failureReasonArgsSize,0)) {
                !thing = op:invokeMethod(!failureReasonArgs, "get", 0);
            }
        }
    }

    // TODO;daniel; this doesn't deal with weird responses (he is at the elevator, the elevator, yes, no, i don't know!!!)
    Variable !y = "Y";
    Predicate !atResponseForm = op:createPredicate("wantBel(X,self,at(!thing,!y))");

    // check if you understand that question
    // check for understanding in the ask question method lolll
    boolean !understood = false;
    Predicate !answerSemantics;
    Predicate !confirmationSemantics;
    Predicate !yesSemantics = op:createPredicate("val(yes)");
    while (op:equals(!understood, false)) {
//        tsc:setSuggestedInput("!thing is at the kitchen");
        !answerSemantics = act:askQuestionFromString(patient, "Do you know where is !thing?", !atResponseForm);
        tsc:displayText("yes or no");
        !confirmationSemantics = act:askQuestionFromString(patient, "Did I understand that correctly?", "val(X)");
        !understood = op:equals(!confirmationSemantics, !yesSemantics);
    }

    Map !bindings = op:invokeMethod(!answerSemantics, "getBindings", !atResponseForm);
    Symbol !thingLoc = op:get(!bindings, !y);
    Predicate !assertPred = op:createPredicate("at(!thing,!thingLoc)");
    act:assertBelief(!assertPred);
}

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
    double !lowChargeLevel = 40; // Threshold to finish tasks and then go home and charge
    double !veryLowChargeLevel = 20; // Threshold to suspend tasks and go home and charge immediately
    double !enoughChargeLevel = 50; // Threshold to stop charging
    boolean !sentGoalHome = false;
    boolean !sentUrgentGoalHome = false;
    Predicate !goHomeGoalPred = op:createPredicate("goHomeAndCharge(?actor, !enoughChargeLevel)");
    Predicate !urgentGoHomeGoalPred = op:createPredicate("urgentGoHomeAndCharge(?actor, !enoughChargeLevel)");
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

// Wraps goHomeAndCharge but is marked with SKIPPENDING/URGENT bypassing the queue
() = urgentGoHomeAndCharge(double ?enoughChargeLevel) {
    act:goHomeAndCharge(?enoughChargeLevel);
}

// Persistent goal until charged to certain threshold
() = goHomeAndCharge(double ?enoughChargeLevel) {
    int !sleepTime = 30000;
    double !currBatteryLevel;
    !currBatteryLevel = act:getBatteryLevel();

    act:goHome();

    while (op:lt(!currBatteryLevel, ?enoughChargeLevel)) {
        !currBatteryLevel = act:getBatteryLevel();
        op:sleep(!sleepTime);
    }
}

// is this  used?
() = evaluateBatteryLevel () {
    double !batteryThreshold = 99.0; //90 %
    double !runtimeThreshold = 1000.0; //1000 seconds
    double !batteryLevel; //percentage
    double !runtime; //seconds
    int !batterySleep = 3000;
    String !message = "I am going home to recharge. Goodbye.";

    !batteryLevel = act:getBatteryLevel();
    !runtime = act:getRuntime();

    if (op:lt(!batteryLevel, !batteryThreshold) || op:lt(!runtime, !runtimeThreshold)) {
        op:sleep(!batterySleep);
        act:generateResponseFromString(!message);
        op:sleep(!batterySleep);
        act:goHome();
    }
}

() = saveLocation(Symbol ?location) {
    effects : {
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

() = goToLocation(Symbol ?location) {
    Symbol !current;

    conditions : {
        pre infer : at(?actor, !current);
        //todo: pete should this have a nonperf effect?
    }
    effects : {
        success : at(?actor, ?location);
        success infer : not(at(?actor, !current));
        nonperf infer : not(at(?actor, !current));
        nonperf infer : at(?actor, unknown);
        failure infer : not(at(?actor, !current));
        failure infer : at(?actor, unknown);
    }
    onInterrupt : {
        cancel : act:stop();
        suspend : act:stop();
    }

    act:goToLocation(?location, !current);
}

() = goHome() {
    Symbol !homeLocation = location_0:location;
    onInterrupt : {
        cancel : act:stop();
        suspend : act:stop();
    }
    //todo: this should not be hardcoded here.
    act:goToLocation(!homeLocation);
    act:goHomePrimitive();
}

/******************************** Misc Testing Scripts ********************************/

(Symbol ?currentLocation) = getCurrentLocation() {
    List !bindings;
    Map !binding;
    Symbol !startLoc = "unknown";
    Predicate !queryPred;
    Variable !x = X;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");

    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("warn", "[getCurrentLocation] locQuery returned no results");
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

    //Doing this as a kind of failsafe, not sure if we want such similar semantics at all though
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "pathName(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "taskName(X)");
    op:add(!responseFormList,!responseForm);

    (!bindings) = act:askQuestionFromString(?actor,"What is the name of the task?", !responseFormList);
   if (op:invokeMethod(!bindings, "isEmpty")) {
        ?name = op:setNull();
    } else {
        !response = op:get(!bindings,!x);
        ?name = op:getName(!response);
    }
}

() = say(String ?literal) {
    act:toggleVoiceOn();
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