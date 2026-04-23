import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Boolean;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
import java.util.Map;
import edu.tufts.hrilab.action.goal.Goal;

(String ?output) = generateOutputStringFromSemantics(Predicate ?semantics) {
    Symbol !addressee;
    !addressee = this:getOriginalSubmitter();
    java.lang.Boolean !false = false;

    edu.tufts.hrilab.slug.common.Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !addressee);
    edu.tufts.hrilab.slug.common.UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setSemantics", ?semantics);
    //!utterance = act:applyPragmaticMeaning(!utterance);
    //!utterance = act:convertSemanticsToText(!utterance);
    op:log("info","!utterance !false");
    !utterance = tsc:convertSemanticsToTextLLMService(!utterance, !false);

    ?output = op:invokeMethod(!utterance,"getWordsAsString");
}

(String ?output) = generateBatteryLevelString() {
    double !batteryLevel; //percentage
    Predicate !responsePredicate;

    !batteryLevel = act:getBatteryLevel();
    !responsePredicate = op:createPredicate("batteryLevel(!batteryLevel)");
    ?output = act:generateOutputStringFromSemantics(!responsePredicate);
}

(String ?output) = generateCurrentLocationString() {
    Symbol !currentLocation;

    !currentLocation = act:getCurrentLocation();
    ?output = act:generateOutputStringFromSemantics(at(?actor, !currentLocation));
}

(String ?output) = generateAllKnownLocationsString() {
    List !locationNames;
    java.lang.String !locationName;
    java.lang.String !locationNameStr = "";
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !locationNames = tsc:getMapLocationNames();
    if (op:isEmpty(!locationNames)) {
        ?output = op:newObject("java.lang.String","There are no saved locations");
    } else {
        op:invokeMethod(!sb,"append","I know the following locations: ");
        foreach(!locationName : !locationNames) {
            op:invokeMethod(!sb,"append",!sep);
            // Explicitly convert to String to avoid numeric-looking names being coerced to integers
            !locationNameStr = op:invokeMethod(!locationName, "toString");
            op:invokeMethod(!sb,"append",!locationNameStr);
            !sep = op:newObject("java.lang.String",", ");
            //act:generateResponseFromString(!locationNameStr);
        }
        ?output = op:invokeMethod(!sb,"toString");
    }
}

(String ?output) = generateAllKnownPatrolPathsString() {
    Predicate !queryPred;
    Variable !x = "X";
    Symbol !path;
    Symbol !pathName = "";
    Map !elem;
    List !bindings;
    java.lang.String !locationNameStr = "";
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";


    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(X,Y)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        op:invokeMethod(!sb,"append","I know the following patrol paths: ");
        foreach(!elem : !bindings) {
            op:invokeMethod(!sb,"append",!sep);
            !path = op:get(!elem,!x);
            !pathName = op:invokeMethod(!path,"getName");
            op:invokeMethod(!sb,"append",!pathName);
            !sep = op:newObject("java.lang.String",", ");
        }
        ?output = op:invokeMethod(!sb,"toString");
    } else {
        ?output = op:newObject("java.lang.String","There are no saved patrol paths");
    }
}

(String ?output) = generateAllLocationsOnPatrolPathString(Symbol ?pathName) {
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
     ?output = act:generateOutputStringFromSemantics(!generateResponsePredicate);
    } else {
     ?output = op:newObject("java.lang.String","I don't have a patrol path saved under the name ?pathName");
    }
}
(String ?output) = generateWifiAtLocationString(Symbol ?location) {
    Predicate !queryPred;
    Variable !x = "X";
    Map !elem;
    List !bindings;
    Symbol !rssi;
    java.lang.String !rssiStr;
    java.lang.String !label;
    java.lang.String !locName;

    !locName = act:locationRefToName(?location);
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifiStrength(?location,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        !rssi = op:get(!elem,!x);
        !rssiStr = op:invokeMethod(!rssi, "toString");
        !label = act:rssiQualityLabel(!rssi);
        ?output = op:invokeStaticMethod("java.lang.String", "format", "wifi at %s was %s dBm, %s", !locName, !rssiStr, !label);
    } else {
        ?output = op:invokeStaticMethod("java.lang.String", "format", "I have not measured wifi at %s", !locName);
    }
}

(String ?output) = generateWifiSummaryString() {
    Predicate !queryPred;
    Variable !loc = "L";
    Variable !rssiVar = "R";
    Map !elem;
    List !bindings;
    Symbol !location;
    Symbol !rssi;
    java.lang.String !locStr;
    java.lang.String !rssiStr;
    java.lang.String !label;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifiStrength(L,R)");
    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        ?output = op:newObject("java.lang.String","I have not measured wifi anywhere");
    } else {
        foreach(!elem : !bindings) {
            op:invokeMethod(!sb,"append",!sep);
            !location = op:get(!elem,!loc);
            !rssi = op:get(!elem,!rssiVar);
            !locStr = act:locationRefToName(!location);
            !rssiStr = op:invokeMethod(!rssi, "toString");
            !label = act:rssiQualityLabel(!rssi);
            op:invokeMethod(!sb,"append",!locStr);
            op:invokeMethod(!sb,"append"," ");
            op:invokeMethod(!sb,"append",!rssiStr);
            op:invokeMethod(!sb,"append"," dBm ");
            op:invokeMethod(!sb,"append",!label);
            !sep = op:newObject("java.lang.String",", ");
        }
        ?output = op:invokeMethod(!sb,"toString");
    }
    act:generateResponseFromString(?output);
}

(String ?output) = generateTemperatureAtLocationString(Symbol ?location) {
    Predicate !queryPred;
    Variable !x = "X";
    Map !elem;
    List !bindings;
    Symbol !temp;
    java.lang.String !tempStr;
    java.lang.String !locName;

    !locName = act:locationRefToName(?location);
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "temperature(?location,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        !temp = op:get(!elem,!x);
        !tempStr = op:invokeMethod(!temp, "toString");
        ?output = op:invokeStaticMethod("java.lang.String", "format", "temperature at %s was %s degrees", !locName, !tempStr);
    } else {
        ?output = op:invokeStaticMethod("java.lang.String", "format", "I have not measured temperature at %s", !locName);
    }
}

(String ?output) = generateTemperatureSummaryString() {
    Predicate !queryPred;
    Variable !loc = "L";
    Variable !tempVar = "T";
    Map !elem;
    List !bindings;
    Symbol !location;
    Symbol !temp;
    java.lang.String !locStr;
    java.lang.String !tempStr;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "temperature(L,T)");
    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        ?output = op:newObject("java.lang.String","I have not measured temperature anywhere");
    } else {
        foreach(!elem : !bindings) {
            op:invokeMethod(!sb,"append",!sep);
            !location = op:get(!elem,!loc);
            !temp = op:get(!elem,!tempVar);
            !locStr = act:locationRefToName(!location);
            !tempStr = op:invokeMethod(!temp, "toString");
            op:invokeMethod(!sb,"append",!locStr);
            op:invokeMethod(!sb,"append"," ");
            op:invokeMethod(!sb,"append",!tempStr);
            op:invokeMethod(!sb,"append"," degrees");
            !sep = op:newObject("java.lang.String",", ");
        }
        ?output = op:invokeMethod(!sb,"toString");
    }
    act:generateResponseFromString(?output);
}

(String ?output) = generateTaughtActionsString() {
    List !taughtActions;
    Predicate !actionPred;
    Integer !numTaught;
    Predicate !reportedPred;
    String !actionName = "";
    String !numToReportString;
    Integer !numToReport;
    Integer !numReported = 0;
    Map !bindings;
    Variable !x = "X";
    Symbol !confirmation;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !taughtActions = tsc:getLearnedActionPredicates();
    if (op:isEmpty(!taughtActions)) {
        ?output = op:newObject("java.lang.String","I have not been taught any new actions");
    } else {
        !numTaught = op:invokeMethod(!taughtActions,"size");
        op:invokeMethod(!sb,"append","I have been taught !numTaught new actions. They are named: ");
        foreach (!actionPred : !taughtActions) {
            op:invokeMethod(!sb,"append",!sep);
            !actionName = op:getName(!actionPred);
            op:invokeMethod(!sb,"append",!actionName);
            !sep = op:newObject("java.lang.String",", ");
        }
        ?output = op:invokeMethod(!sb,"toString");
    }
}

(String ?output) = generatePredefinedActionsString() {
    List !instructableActions;
    Integer !numPredefined;
    String !actionName = "";
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !instructableActions = tsc:getInstructableActions();
    if (op:isEmpty(!instructableActions)) {
        ?output = op:newObject("java.lang.String","I could not retrieve my predefined action list");
    } else {
        !numPredefined = op:invokeMethod(!instructableActions,"size");
        op:invokeMethod(!sb,"append","I have !numPredefined predefined actions. They are: ");
        foreach (!actionName : !instructableActions) {
           op:invokeMethod(!sb,"append",!sep);
           op:invokeMethod(!sb,"append",!actionName);
           !sep = op:newObject("java.lang.String",", ");
       }
       ?output = op:invokeMethod(!sb,"toString");
   }
}

//Wrapper to adhere to queueEM
(String ?output) = generateActionStepsString(Predicate ?action) {
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
            ?output = act:generateOutputStringFromSemantics(!failCond);
            return;
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
        ?output = act:generateOutputStringFromSemantics(!desc);
    } else {
        ?output = op:newObject("java.lang.String","As of right now, I can only describe the actions which have been taught to me by a user. For predefined actions, please refer to my user guide");
    }
}

//Wrapper to adhere to queueEM
(String ?output) = generateIfCanPerformString(Predicate ?actionPredicate) {
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
      //act:generateResponseFromString("Yes");
      ?output = act:generateOutputStringFromSemantics(knowHow(?actionPredicate));
    } else {
      //act:generateResponseFromString("No");
      ?output = act:generateOutputStringFromSemantics(not(knowHow(?actionPredicate)));
    }
}

(String ?output) = generateKioskSettingsString() {
    Predicate !kioskGoalPredicate;
    Boolean !enabled;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");

    !enabled = tsc:getKioskModeEnabled();
    if (op:equals(!enabled,true)) {
        op:invokeMethod(!sb,"append","Kiosk mode is currently enabled.");
    } else {
        op:invokeMethod(!sb,"append","Kiosk mode is currently disabled.");
    }

    !kioskGoalPredicate = tsc:getWrappedKioskGoalPredicate();
    ?output = act:generateOutputStringFromSemantics(!kioskGoalPredicate);
    op:invokeMethod(!sb,"append",?output);
    ?output = op:invokeMethod(!sb,"toString");
}

(String ?output) = generateSavedVideosString() {
    List !videoNames;
    Integer !numVideos;
    String !videoName = "";
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !videoNames = tsc:getVideoNames(false);

    if (op:isEmpty(!videoNames)) {
        ?output = op:newObject("java.lang.String","I currently have no saved videos");
    } else {
        !numVideos = op:invokeMethod(!videoNames,"size");
        op:invokeMethod(!sb,"append","I have !numVideos saved videos. They are called: ");
        foreach (!videoName : !videoNames) {
           op:invokeMethod(!sb,"append",!sep);
           op:invokeMethod(!sb,"append",!videoName);
           !sep = op:newObject("java.lang.String",", ");
        }
        ?output = op:invokeMethod(!sb,"toString");
    }
}

(String ?output) = generateVideoVolumeString() {
    Integer !volume;

    !volume = tsc:getVideoVolume();
     ?output = op:newObject("java.lang.String","video volume is currently set to !volume percent");
}

(String ?output) = generateIsVideoSavedString(Symbol ?videoName) {
    Boolean !match;
    String !name;

    !name = op:getName(?videoName);
    !match = tsc:isVideoSaved(!name);
    if (op:equalsValue(!match,true)) {
        ?output = op:newObject("java.lang.String","Yes, I have a video saved under the name !name");
    } else {
        ?output = op:newObject("java.lang.String","I do not have a video saved under the name !name");
    }
}

(String ?output) = generateSavedImagesString() {
    List !imageNames;
    Integer !numImages;
    String !imageName = "";
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !imageNames = tsc:getImageNames(false);

    if (op:isEmpty(!imageNames)) {
        ?output = op:newObject("java.lang.String","I currently have no saved images");
    } else {
        !numImages = op:invokeMethod(!imageNames,"size");
        op:invokeMethod(!sb,"append","I have !numImages saved images. They are called: ");
        foreach (!imageName : !imageNames) {
           op:invokeMethod(!sb,"append",!sep);
           op:invokeMethod(!sb,"append",!imageName);
           !sep = op:newObject("java.lang.String",", ");
        }
        ?output = op:invokeMethod(!sb,"toString");
    }
}

//Is there really no good way to call an action for result from a String/Predicate?
//() = itkToolWrapper(Predicate ?semantics) {
//  String !output;
//  Symbol !user = this:getOriginalSubmitter();
//  !output = act:?semantics;
//  tsc:addToolResponse(!user, ?actor,!output);
//}
//
//() = itkResponseWrapper(Predicate ?semantics) {
//  String !output;
//  !output = act:?semantics;
//  act:generateResponseFromString(!output);
//}

() = getBatteryLevelString() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateBatteryLevelString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getCurrentLocationString() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateCurrentLocationString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getAllKnownLocations() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateAllKnownLocationsString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getAllKnownPatrolPaths() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateAllKnownPatrolPathsString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getAllLocationsOnPatrolPath(Symbol ?pathName) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateAllLocationsOnPatrolPathString(?pathName);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getTaughtActions() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateTaughtActionsString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getPredefinedActions() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generatePredefinedActionsString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getActionSteps(Predicate ?action) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateActionStepsString(?action);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getIfCanPerform(Predicate ?actionPredicate) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateIfCanPerformString(?actionPredicate);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getKioskSettings() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateKioskSettingsString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getSavedVideos() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateSavedVideosString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getVideoVolume() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateVideoVolumeString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getIsVideoSaved(Symbol ?videoName) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateIsVideoSavedString(?videoName);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getSavedImages() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateSavedImagesString();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
() = getCurrentPendingScheduledGoalInformation(java.lang.Boolean ?offContextState) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = tsc:getFullGoalsSummary(?actor,?offContextState);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}

() = explainHowToTeachATask() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = op:newObject("java.lang.String","You can teach me how to perform new tasks for future instruction simply by saying \"please learn a new task sequence!\". I can take it from there and guide you through supplying all the information I need to learn the task. Alternatively, ask me for an explanation of what a taught task sequence consists of and you can supply all the necessary information yourself!");
    //String !contextId = this:getCommand();
    //!output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    //if (~op:invokeMethod(!output,"isEmpty")) {
    act:generateResponseFromString(!output);
    if (op:equalsValue(!user,"local_user") || op:equalsValue(!user,"local_visitor")) {
        act:triggerWW();
    }
    //}
}

() = explainWhatALearnedTaskIs() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = op:newObject("java.lang.String","Taught task sequences consist of a few simple pieces of information: a label to save the task under, a sequence of actions to perform, and a corresponding sequence of locations defining where to perform each action. Supplying this information will result in a task which is instructable through reference to the supplied label.");
    //String !contextId = this:getCommand();
    //!output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    //if (~op:invokeMethod(!output,"isEmpty")) {
    act:generateResponseFromString(!output);
    if (op:equalsValue(!user,"local_user") || op:equalsValue(!user,"local_visitor")) {
        act:triggerWW();
    }
    //}
}

() = explainWhatActionsCanBePerformedAlongALearnedTask() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = op:newObject("java.lang.String","Steps of a taught task sequence can include things like playing a video, saying a phrase, taking a wi-fi measurement, and more. I can also simply pause briefly at each location if desired.");
    //String !contextId = this:getCommand();
    //!output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    //if (~op:invokeMethod(!output,"isEmpty")) {
    act:generateResponseFromString(!output);
    if (op:equalsValue(!user,"local_user") || op:equalsValue(!user,"local_visitor")) {
        act:triggerWW();
    }
    //}
}