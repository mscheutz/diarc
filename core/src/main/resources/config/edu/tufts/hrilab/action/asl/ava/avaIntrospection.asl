import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Boolean;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
import java.util.Map;
import edu.tufts.hrilab.action.goal.Goal;

// is this  used?
() = reportBatteryLevel() {
    double !batteryLevel; //percentage
    Predicate !responsePredicate;

    !batteryLevel = act:getBatteryLevel();
    !responsePredicate = op:createPredicate("batteryLevel(!batteryLevel)");
    act:generateResponse(!responsePredicate);
}

() = describeCurrentLocation() {
    Symbol !currentLocation;

    !currentLocation = act:getCurrentLocation();
    act:generateResponse(at(?actor, !currentLocation));
}

() = describeAllKnownLocations() {
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = HIGH;
    Boolean !blockingTaskExists;
    Symbol !submitter;
    !submitter = this:getOriginalSubmitter();

    !blockingTaskExists = act:blockingTaskExists();
    if (op:equals(!blockingTaskExists,true)) {
        act:generateResponseFromString("I cannot list all of my saved locations while performing another task, please try again when I am not busy");
    } else {
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "describeAllKnownLocationsAction(?actor)");
        !goalID = act:submitGoal(!state,!execType,!priorityTier,!submitter);
        !goalStatus = act:joinOnGoal(!goalID);
    }
}

() = describeAllKnownLocationsAction() {
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
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = HIGH;
    Boolean !blockingTaskExists;
    Symbol !submitter;
    !submitter = this:getOriginalSubmitter();

    !blockingTaskExists = act:blockingTaskExists();
    if (op:equals(!blockingTaskExists,true)) {
        act:generateResponseFromString("I cannot list taught actions while performing another task, please try again when I am not busy");
    } else {
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reportTaughtActionsAction(?actor)");
        !goalID = act:submitGoal(!state,!execType,!priorityTier,!submitter);
        !goalStatus = act:joinOnGoal(!goalID);
    }
}

() = reportTaughtActionsAction() {
    List !taughtActions;
    Predicate !actionPred;
    Integer !numTaught;
    Predicate !reportedPred;
    String !actionName;
    String !numToReportString;
    Integer !numToReport;
    Integer !numReported = 0;
    Map !bindings;
    Variable !x = "X";
    Symbol !confirmation;

    !taughtActions = tsc:getLearnedActionPredicates();
    if (op:isEmpty(!taughtActions)) {
        act:generateResponseFromString("I have not been taught any new actions");
    } else {
        !numTaught = op:invokeMethod(!taughtActions,"size");
        if (op:gt(!numTaught,5)) {
            !bindings = act:askQuestionFromString("I have been taught !numTaught new actions. How many should I list at a time?", amount(X));
            if (op:invokeMethod(!bindings,"isEmpty")) {
                exit(FAIL,not(wasAnswered(myQuestion)));
            }
            !numToReportString = op:get(!bindings, !x);
            if (op:equalsValue(!numToReportString,"all")) {
                !numToReport = op:newObject("java.lang.Integer", !numTaught);
            } else {
                !numToReport = op:newObject("java.lang.Integer", !numToReportString);
            }
            if (op:gt(!numToReport,0)) {
                foreach (!actionPred : !taughtActions) {
                    if (op:equals(!numReported,!numToReport)) {
                        !bindings = act:askQuestionFromString("Would you like me to keep going?", val(X));
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
                        !actionName = op:getName(!actionPred);
                        act:generateResponseFromString(!actionName);
                        !numReported = op:++(!numReported);
                    }
                }
                act:generateResponseFromString("I am done listing the actions I have been taught");
            } else {
            act:generateResponseFromString("Sorry, I am exiting this conversation context. Please try again and supply a positive number");
            }
        } else {
            act:generateResponseFromString("I have been taught !numTaught new actions. They are named");
            foreach (!actionPred : !taughtActions) {
                //!reportedPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "iHaveBeenTaughtTo(!actionPred)");
                //act:generateResponse(!reportedPred);
                !actionName = op:getName(!actionPred);
                act:generateResponseFromString(!actionName);
            }
        }
    }
}

//TODO: If allowing to reach this directly through language, then need to similarly add check for blockingTaskExists and
//      associated logic
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
        !bindings = act:askQuestionFromString("I have !numPredefined predefined actions. Would you like me to start listing them out?", val(X));
        if (op:invokeMethod(!bindings,"isEmpty")) {
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
        !confirmation = op:get(!bindings, !x);
        if (op:equalsValue(!confirmation, yes)) {
            !bindings = act:askQuestionFromString("How many should I list at a time?", amount(X));
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
                        !bindings = act:askQuestionFromString("Would you like me to keep going?", val(X));
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

(Boolean ?exists) = blockingTaskExists() {
    List !systemGoals;
    Predicate !systemGoal;
    java.lang.Long !goalId;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier;

    ?exists = op:newObject("java.lang.Boolean", "false");
    !systemGoals = tsc:getSystemGoals(?actor);
    if (~op:isEmpty(!systemGoals)) {
        //Assumes QueueEM here
        !systemGoal = op:get(!systemGoals,0);
        !goalId = tsc:getGoalId(!systemGoal);
        !goalStatus = tsc:getGoalStatus(!goalId);
        !priorityTier = act:getGoalPriorityTier(!goalId);
        if (~op:equalsValue(!goalStatus,"SUSPENDED") && ~op:equalsValue(!priorityTier,"KIOSK")) {
            ?exists = op:newObject("java.lang.Boolean", "true");
        }
    }
}


//TODO: LLM only - fail gracefully if no LLM present
() = reportInstructableActions() {
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = HIGH;
    Boolean !blockingTaskExists;
    Symbol !submitter;
    !submitter = this:getOriginalSubmitter();

    !blockingTaskExists = act:blockingTaskExists();
    if (op:equals(!blockingTaskExists,true)) {
        act:generateResponseFromString("I cannot list instructable actions while performing another task, please try again when I am not busy");
    } else {
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reportInstructableActionsAction(?actor)");
        !goalID = act:submitGoal(!state,!execType,!priorityTier,!submitter);
        !goalStatus = act:joinOnGoal(!goalID);
    }
}

() = reportInstructableActionsAction() {
    Map !bindings;
    Symbol !type;
    Variable !x = "X";
    Boolean !validResponse = false;

    !bindings = act:askQuestionFromString("Would you like me to report all of the actions I have been taught, my predefined actions, or both?", actionType(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }

    !type = op:get(!bindings, !x);
    if (op:equalsValue(!type,"taught") || op:equalsValue(!type,"both")) {
        !validResponse = op:newObject("java.lang.Boolean", true);
        act:reportTaughtActionsAction();
    }
    if (op:equalsValue(!type,"predefined") || op:equalsValue(!type,"both")) {
        !validResponse = op:newObject("java.lang.Boolean", true);
        act:reportPredefinedActions();
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

() = reportKioskSettings() {
    Predicate !kioskGoalPredicate;
    Boolean !enabled;

    !enabled = tsc:getKioskModeEnabled();
    if (op:equals(!enabled,true)) {
        act:generateResponseFromString("Kiosk mode is currently enabled.");
    } else {
        act:generateResponseFromString("Kiosk mode is currently disabled.");
    }

    !kioskGoalPredicate = tsc:getWrappedKioskGoalPredicate();
    act:generateResponse(!kioskGoalPredicate);
}

() = listSavedVideos() {
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = HIGH;
    Boolean !blockingTaskExists;
    Symbol !submitter;
    !submitter = this:getOriginalSubmitter();

    !blockingTaskExists = act:blockingTaskExists();
    if (op:equals(!blockingTaskExists,true)) {
        act:generateResponseFromString("I cannot list saved videos while performing another task, please try again when I am not busy");
    } else {
        //TODO: if the current action is the kiosk action, then submit with SKIPPENDING priority?
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "listSavedVideosAction(?actor)");
        !goalID = act:submitGoal(!state,!execType,!priorityTier,!submitter);
        !goalStatus = act:joinOnGoal(!goalID);
    }
}

() = listSavedVideosAction() {
    List !videoNames;
    Integer !numVideos;
    String !videoName;

    !videoNames = tsc:getVideoNames(false);

    if (op:isEmpty(!videoNames)) {
        act:generateResponseFromString("I currently have no saved videos");
    } else {
        !numVideos = op:invokeMethod(!videoNames,"size");
        act:generateResponseFromString("I have !numVideos saved videos. They are called");
        foreach (!videoName : !videoNames) {
            //TODO: bug in generateResponseFromString where the string is just a number
            act:generateResponseFromString(" !videoName");
        }
    }
}

() = reportVideoVolume() {
    Integer !volume;

    !volume = tsc:getVideoVolume();
    act:generateResponseFromString("video volume is currently set to !volume percent");
}

() = reportIsVideoSaved(Symbol ?videoName) {
    Boolean !match;
    String !name;

    !name = op:getName(?videoName);
    !match = tsc:isVideoSaved(!name);
    if (op:equalsValue(!match,true)) {
        act:generateResponseFromString("Yes, I have a video saved under the name !name");
    } else {
        act:generateResponseFromString("I do not have a video saved under the name !name");
    }
}

() = listSavedImages() {
    Predicate !state;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = HIGH;
    Boolean !blockingTaskExists;
    Symbol !submitter;
    !submitter = this:getOriginalSubmitter();

    !blockingTaskExists = act:blockingTaskExists();
    if (op:equals(!blockingTaskExists,true)) {
        act:generateResponseFromString("I cannot list saved images while performing another task, please try again when I am not busy");
    } else {
        //TODO: if the current action is the kiosk action, then submit with SKIPPENDING priority?
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "listSavedImagesAction(?actor)");
        !goalID = act:submitGoal(!state,!execType,!priorityTier,!submitter);
        !goalStatus = act:joinOnGoal(!goalID);
    }
}

() = listSavedImagesAction() {
    List !imageNames;
    Integer !numImages;
    String !imageName;

    !imageNames = tsc:getImageNames(false);

    if (op:isEmpty(!imageNames)) {
        act:generateResponseFromString("I currently have no saved images");
    } else {
        !numImages = op:invokeMethod(!imageNames,"size");
        act:generateResponseFromString("I have !numImages saved images. They are called");
        foreach (!imageName : !imageNames) {
            //TODO: bug in generateResponseFromString where the string is just a number
            act:generateResponseFromString(" !imageName");
        }
    }
}