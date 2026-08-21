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
    !utterance = tsc:convertSemanticsToText(!utterance);

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
        op:invokeMethod(!sb,"append",". Please list no more than 5 locations at a time in any response, unless explicitly asked for all/more/different locations. You may still provide the total number of locations you have saved and list 5 in your initial response though.");
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
    Symbol !label;
    java.lang.String !labelStr;
    java.lang.String !locName;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifiStrength(?location,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        !rssi = op:get(!elem,!x);
        !labelStr = act:rssiQualityLabel(!rssi);
        !label = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !labelStr);
        ?output = act:generateOutputStringFromSemantics(wifiStrength(?location, !rssi, !label));
    } else {
        !locName = act:locationRefToName(?location);
        ?output = op:invokeStaticMethod("java.lang.String", "format", "I have not measured wifi at %s", !locName);
    }
}

(String ?output) = generateWifiSummaryString(Symbol ?reportLocation) {
    Predicate !queryPred;
    Predicate !reading;
    Predicate !summaryPred;
    Variable !loc = "L";
    Variable !rssiVar = "R";
    Map !elem;
    List !bindings;
    Symbol !location;
    Symbol !rssi;
    Symbol !label;
    java.lang.String !labelStr;
    java.util.List !readings = op:newObject("java.util.ArrayList");
    java.util.List !readingArgs;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifiStrength(L,R)");
    !bindings = act:queryBelief(!queryPred);
    foreach(!elem : !bindings) {
        !location = op:get(!elem,!loc);
        if (~op:equals(!location, ?reportLocation)) {
            !rssi = op:get(!elem,!rssiVar);
            !labelStr = act:rssiQualityLabel(!rssi);
            !label = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !labelStr);
            !readingArgs = op:newObject("java.util.ArrayList");
            op:invokeMethod(!readingArgs, "add", !location);
            op:invokeMethod(!readingArgs, "add", !rssi);
            op:invokeMethod(!readingArgs, "add", !label);
            !reading = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifiStrength", !readingArgs);
            op:invokeMethod(!readings, "add", !reading);
        }
    }
    if (op:isEmpty(!readings)) {
        ?output = op:newObject("java.lang.String","I have not measured wifi anywhere");
    } else {
        !summaryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifiMeasurements", !readings);
        ?output = act:generateOutputStringFromSemantics(!summaryPred);
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
    Predicate !response;
    !output = act:generateBatteryLevelString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getCurrentLocationString() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateCurrentLocationString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getAllKnownLocations() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateAllKnownLocationsString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getAllKnownPatrolPaths() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateAllKnownPatrolPathsString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getAllLocationsOnPatrolPath(Symbol ?pathName) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateAllLocationsOnPatrolPathString(?pathName);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getTaughtActions() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateTaughtActionsString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getPredefinedActions() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generatePredefinedActionsString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getActionSteps(Predicate ?action) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateActionStepsString(?action);
    !output = op:newObject("java.lang.String","!output Summarize this description if relaying to the user.");
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getIfCanPerform(Predicate ?actionPredicate) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateIfCanPerformString(?actionPredicate);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getKioskSettings() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateKioskSettingsString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getSavedVideos() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateSavedVideosString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getVideoVolume() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateVideoVolumeString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getIsVideoSaved(Symbol ?videoName) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateIsVideoSavedString(?videoName);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getSavedImages() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateSavedImagesString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}
() = getCurrentPendingScheduledGoalInformation() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = tsc:getFullGoalsSummary(?actor);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

() = getInformationAroundHowToTeachATask() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = op:newObject("java.lang.String","You can teach me how to perform new tasks for future instruction simply by saying \"please learn a new task sequence!\". I can take it from there and guide you through supplying all the information I need to learn the task. Alternatively, ask me for an explanation of what a taught task sequence consists of and you can supply all the necessary information yourself!");
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

() = getInformationAroundWhatALearnedTaskIs() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = op:newObject("java.lang.String","Taught task sequences consist of a few simple pieces of information: a label to save the task under, a sequence of actions to perform, and a corresponding sequence of locations defining where to perform each action. Supplying this information will result in a task which is instructable through reference to the supplied label.");
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

() = getInformationAroundWhatActionsCanBePerformedAlongALearnedTask() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = op:newObject("java.lang.String","Steps of a taught task sequence can include things like playing a video, saying a phrase, taking a wi-fi measurement, and more. I can also simply pause briefly at each location if desired.");
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

() = getInformationAroundTeachingNewActions() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = op:newObject("java.lang.String","- Teaching new actions: A user can begin teaching you how to perform new tasks for future instruction simply by saying \"please learn a new task sequence!\" (after which you will call the \"learnTaskSequence\" tool). You can take it from there and guide them through supplying all the information you need to learn the task. Alternatively, they can ask you for an explanation of what a taught task sequence consists of and then be able to supply all the necessary information themselves!\\n- Taught Task Components: Taught task sequences consist of a few simple pieces of information: a label to save the task under, a sequence of actions to perform, and a corresponding sequence of locations defining where to perform each action. Supplying this information will result in a task which is instructable through reference to the supplied label.\\n- Possible Taught Task Sequence Steps: Steps of a taught task sequence can include things like playing a video, saying a phrase, taking a wi-fi measurement, and more. You can also simply pause briefly at each location if desired.\\n- Please use the supplied information to 1. answer the user's questions to the best of your ability and/or 2. Inform what tool calls to make to fulfill their requests. If none of this information is relevant, you may ignore it");
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

() = getInformationAroundTeachingLocationsAndNavigation() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = op:newObject("java.lang.String","Fact List:\\n- The set of saved locations available to you are presented in the location list.\\n- You only know how to navigate between saved locations. You cannot travel to any other position in the environment without explicitly saving it first.\\n- The only way to save a new location is if the user instructs you to save your current position as a new location (where you will subsequently call the \"saveLocation\" tool).\\n- The only way you can get to a new position in order to save a new location is by following the user to somewhere new.\\n- The only way to follow the user is if they instruct you to (where you will subsequently call the \"followUser\" tool). Do not tell the user you are following them unless you have done so and/or your active task is follow.\\n- The only way you will stop following a user is if they instruct you to stop following (where you will subsequently call the \"stopFollow\" tool)\\n#\\nResponse target: Please use the supplied information to 1. answer the user's questions to the best of your ability and/or 2. Inform what tool calls to make to fulfill their requests. If none of this information is relevant, you may ignore it.");
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

() = checkPastInteractionHistoryForMissingInformation(Symbol ?user, Symbol ?agentTeam) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = tsc:getExtendedConversationHistoryString(?user, ?agentTeam);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}