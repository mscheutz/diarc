import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.Map;
import java.util.List;
import java.lang.String;
import java.lang.Long;
import java.lang.Integer;
import java.lang.Double;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.Justification;

() = scheduleGoal() {

    Map !bindings;
    Variable !x = "X";
    Variable !y = "Y";
    Predicate !goalPredicate;
    Predicate !scheduledGoal;
    Predicate !confirmation;
    Symbol !time;
    Symbol !addl;
    Symbol !interval;
    Symbol !intervalUnits;
    java.util.List !responseFormList;
    Predicate !responseForm;
    java.lang.Boolean !relativeTime = false;

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("absoluteTime(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("absoluteTime(X,Y)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("relativeTime(X,Y)");
    op:add(!responseFormList,!responseForm);


    !bindings = act:askQuestionFromString(?actor, "What would you like to do?", taskPredicate(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !goalPredicate = op:get(!bindings, !x);
    op:log(debug, "[scheduleGoal] goal !goalPredicate");

    !bindings = act:askQuestionFromString(?actor, "When should this task be scheduled for?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !time = op:get(!bindings, !x);
    !addl = op:get(!bindings, !y);
    op:log("debug",!addl);
    if (~op:isNull(!addl)) {
        if (op:equalsValue(!addl, pm) || op:equalsValue(!addl, am)) {
            !time = act:parseTime(!time, !addl);
        } else {
            !relativeTime = op:newObject("java.lang.Boolean", true);
        }
    }
    op:log(debug, "[scheduleGoal] time/units: !time/!addl");

    !bindings = act:askQuestionFromString(?actor, "Should this task be repeated?", val(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !confirmation = op:get(!bindings, !x);

    if (op:equalsValue(!confirmation, yes())) {
      //Only allowing relativeTime(X,Y) response form here for code simplicity.
      !bindings = act:askQuestionFromString(?actor, "How often?", !responseForm);
      if (op:invokeMethod(!bindings,"isEmpty")) {
          exit(FAIL,not(wasAnswered(myQuestion)));
      }
      !interval = op:get(!bindings, !x);
      !intervalUnits = op:get(!bindings, !y);

      if (op:equals(!relativeTime,true)) {
        !scheduledGoal = op:createPredicate("scheduled(!goalPredicate, period(!interval, !intervalUnits), start(!time, !addl))"); // TODO: EAK: is this right?
      } else {
        !scheduledGoal = op:createPredicate("scheduled(!goalPredicate, period(!interval, !intervalUnits), start(!time))"); // TODO: EAK: is this right?
      }
    } else {
      if (op:equals(!relativeTime,true)) {
        !scheduledGoal = op:createPredicate("scheduled(!goalPredicate, start(!time, !addl))"); // TODO: EAK: is this right?
      } else {
        !scheduledGoal = op:createPredicate("scheduled(!goalPredicate, start(!time))"); // TODO: EAK: is this right?
      }
    }
    op:log(info, "[scheduleGoal] scheduledGoal: !scheduledGoal");

    act:submitGoal(!scheduledGoal);

    act:generateResponse(?actor, "added(!scheduledGoal)", direct);
}

() = modifyScheduledGoal() {
    Predicate !scheduledGoalPred;
    java.lang.Long !goalId;
    java.lang.String !goalPredName;
    Predicate !innerGoalPred;
    java.lang.String !innerGoalPredName;
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Variable !y = "Y";
    Symbol !response;
    Symbol !yResponse;
    java.lang.Boolean !success;
    java.lang.Integer !numArgs;
    java.util.List !responseFormList;
    Predicate !responseForm;
    edu.tufts.hrilab.action.goal.GoalInfo !goalInfo;
    java.lang.Boolean !isPersistent;
    java.lang.Boolean !modifyInterval = false; //true == modify next execution time
    edu.tufts.hrilab.action.goal.GoalType !goalType;
    java.lang.Long !timeMillis;
    java.lang.Boolean !relativeTime = false;
    Predicate !updatePred;

    op:log("debug", "[modifyScheduledGoal] in method");

    !scheduledGoalPred = act:selectScheduledGoal();
    !goalId = tsc:getGoalId(!scheduledGoalPred);
    !goalInfo = tsc:getGoalInfo(!goalId);
    !goalType = op:invokeMethod(!goalInfo,"getGoalType");
    op:log("info", "[modifyScheduledGoal] have goal !scheduledGoalPred with id !goalId and type !goalType");

    //Scheduled Persistent Scheduled or Persistent Scheduled cases
    if (~op:equalsValue(!goalType,"SCHEDULED")) {
        //Ask to modify either inner or outer timing
        !bindings = act:askQuestionFromString(?actor, "Would you like to shift the timing of executions, or the interval at which they are executed?", timingOf(X));
        if (~op:invokeMethod(!bindings,"isEmpty")) {
            !response = op:get(!bindings,!x);
            if (op:equalsValue(!response,execution)) {
                op:log("debug", "[modifyScheduledGoal] got response to modify first execution");
            } elseif (op:equalsValue(!response,interval)) {
                op:log("debug", "[modifyScheduledGoal] got response to modify scheduled goal interval");
                !modifyInterval = op:newObject("java.lang.Boolean", true);
            } else {
                exit(FAIL,not(understand(?actor,whatToModify)));
            }
        } else {
            exit(FAIL,not(understand(?actor,whatToModify)));
        }
    }

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("absoluteTime(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("absoluteTime(X,Y)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("relativeTime(X,Y)");
    op:add(!responseFormList,!responseForm);
    if (op:equals(!modifyInterval,true)) {
        //Scheduled Persistent Scheduled or Persistent Scheduled Interval
        !bindings = act:askQuestionFromString(?actor, "How frequently would you like to schedule this task instead?", !responseFormList);
    } else {
        if (op:equalsValue(!goalType,"SCHEDULED")) {
            !bindings = act:askQuestionFromString(?actor, "When would you like to schedule this task instead?", !responseFormList);
        } else {
            !bindings = act:askQuestionFromString(?actor, "When should this task's next execution occur?", !responseFormList);
        }
    }

    if (~op:invokeMethod(!bindings,"isEmpty")) {
        !response = op:get(!bindings,!x);
        !yResponse = op:get(!bindings,!y);
        if (~op:isNull(!yResponse)) {
            if (op:equalsValue(!yResponse, pm) || op:equalsValue(!yResponse, am)) {
                !response = act:parseTime(!response, !yResponse);
            } else {
                !relativeTime = op:newObject("java.lang.Boolean", true);
            }
        }

        if (op:equals(!modifyInterval,true)) {
            if (op:equals(!relativeTime,true)) {
                !updatePred = op:createPredicate("period", !response, !yResponse); // TODO: EAK: is this right?
                !success = tsc:updateScheduledGoal(!goalId, !updatePred);
            } else {
                !updatePred = op:createPredicate("period", !response); // TODO: EAK: is this right?
                !success = tsc:updateScheduledGoal(!goalId, !!updatePred);
            }
        } else {
            if (op:equals(!relativeTime,true)) {
                !updatePred = op:createPredicate("start", !response, !yResponse); // TODO: EAK: is this right?
                !success = tsc:updateScheduledGoal(!goalId, !updatePred);
            } else {
                !updatePred = op:createPredicate("start", !response); // TODO: EAK: is this right?
                !success = tsc:updateScheduledGoal(!goalId, !updatePred);
            }
        }
    } else {
        exit(FAIL,not(supplied(timing)));
    }

    if (op:equals(!success,true)) {
        act:generateResponseFromString("Ok");
        //TODO: report updated scheduling details? maybe easiest to implement as a tsc for 'getScheduledStateString(gid)'
        return;
    } else {
      //TODO: Would be hard, but nice, to differentiate between failure reasons and return some justification. Difficult
      //    to get the specific failure case generally from inner java code though.
      exit(FAIL);
    }
}

(Predicate ?scheduledGoalPred) = selectScheduledGoal() {
    List !scheduledGoals;
    Predicate !goalPred;
    Integer !numGoals;
    Integer !goalIndex;
    Integer !spokenIndex;
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Symbol !response;
    java.lang.String !numberString;
    java.lang.Integer !goalNumber;
    Predicate !goalPredExtended;
    Long !gid;

    !scheduledGoals = tsc:getScheduledGoals();
    op:log(debug, "[modifyScheduledGoal] current scheduled goals: !scheduledGoals");
    !numGoals = op:invokeMethod(!scheduledGoals, "size");

    if (op:isEmpty(!scheduledGoals)) {
        //TODO: test NLG
        exit(FAIL,not(exist(scheduledGoals)));
        //act:generateResponse("There are no currently scheduled goals");
        //return;
    } elseif (op:equals(!numGoals,1)) {
        ?scheduledGoalPred = op:get(!scheduledGoals,0);
        //TODO: should this function use submitted pred or 'current state' pred
        act:generateResponseFromString("There is currently one scheduled task");
        act:generateResponse(!scheduledGoalPred);
        return;
    } else {
        act:generateResponseFromString("There are multiple currently scheduled tasks");
        for (!goalIndex=0; !goalIndex lt !numGoals; !goalIndex ++) {
            !goalPred = op:get(!scheduledGoals,!goalIndex);
            !spokenIndex = op:++(!goalIndex);
            act:generateResponseFromString("Task !spokenIndex");
            act:generateResponse(!goalPred);
        }
        act:suspendWaitForResponse(option(X),?actor,false);
        !bindings = act:askQuestionFromString(?actor, "Which task did you want to modify?", option(X));
        act:resumeWaitForResponse(option(X),?actor);
        if (~op:invokeMethod(!bindings,"isEmpty")) {
            !response = op:get(!bindings,!x);
            !numberString = op:getName(!response);
            !goalNumber = op:newObject("java.lang.Integer", !numberString);
            !goalNumber = op:--(!goalNumber);
            if (op:lt(!goalNumber,!numGoals)) {
                ?scheduledGoalPred = op:get(!scheduledGoals,!goalNumber);
                return;
            } else {
                act:generateResponseFromString("Task !numberString was not a valid option");
            }
        }
    }
    exit(FAIL,not(know(?actor,whichGoalYouWantToModify)));
}

() = listScheduledGoals() {
    List !scheduledGoals;
    Integer !numGoals;
    Predicate !goalPred;

    !scheduledGoals = tsc:getScheduledGoals();
    //op:log(debug, "[listScheduledGoals] current scheduled goals: !scheduledGoals");
    if (op:isEmpty(!scheduledGoals)) {
        act:generateResponseFromString("There are no scheduled tasks");
    } else {
        foreach (!goalPred : !scheduledGoals) {
            act:generateResponse(!goalPred);
        }
    }
}

() = cancelAllScheduledGoals() {
    List !scheduledGoalIds;
    Integer !numGoals;
    Predicate !goalPred;
    Long !gid;

    !scheduledGoalIds = tsc:getScheduledGoalIds();
    !numGoals = op:invokeMethod(!scheduledGoalIds, "size");
    if (op:isEmpty(!scheduledGoalIds)) {
        act:generateResponseFromString("There are no scheduled tasks");
    } else {
        foreach (!gid : !scheduledGoalIds) {
            act:cancelGoal(!gid);
        }
        act:generateResponseFromString("cancelled all scheduled tasks");
    }
}

() = cancelScheduledGoal() {
    Predicate !scheduledGoalPred;
    Long !goalId;

    op:log("info", "[modifyScheduledGoal] in method");
    !scheduledGoalPred = act:selectScheduledGoal();
    !goalId = tsc:getGoalId(!scheduledGoalPred);
    act:cancelGoal(!goalId);
    act:generateResponseFromString("the scheduled task has been canceled");
}

() = forEach(Predicate ?collection, Symbol ?actionName) {
    Predicate !goalPred;
    List !args;
    Symbol !arg;
    Long !gid;
    GoalStatus !goalStatus;
    Justification !failureJustification;
    List !failureReasons;

    !args = op:invokeMethod(?collection, "getArgs");
    foreach (!arg : !args) {
        !goalPred = op:createPredicate("?actionName(?actor, !arg)");
        !gid = act:submitGoal(!goalPred);
        !goalStatus = act:joinOnGoal(!gid);
        if (op:invokeMethod(!goalStatus, "isFailure")) {
          !failureJustification = act:getGoalFailConditions(!gid);
          exit(FAIL, !failureJustification);
        } elseif (op:equalsValue(!goalStatus,"CANCELED")) {
            exit(CANCEL);
        }
    }
}

() = definePatrolPath() {
    Map !bindings;
    Variable !x = "X";
    Symbol !pathName;
    String !nameString;
    java.util.List !responseFormList;
    Predicate !responseForm;

    //Doing this as a kind of failsafe, not sure if we want such similar semantics at all though
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("pathName(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("taskName(X)");
    op:add(!responseFormList,!responseForm);

    !bindings = act:askQuestionFromString(?actor,"What is the name of the path?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !pathName = op:get(!bindings, !x);
    act:definePatrolPath(!pathName);
}

() = definePatrolPath(Symbol ?name) {
    Map !bindings;
    Variable !x = "X";
    Predicate !locationId;
    String !tmpString;
    Symbol !tmp;
    Predicate !location = "none()";
    List !locations;
    Predicate !collectionDefinition;
    Predicate !locationsPredicate;

    !locations = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    !bindings = act:askQuestionFromString(?actor,"What's the first location on the path?", location(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !locationId = op:get(!bindings, !x);
    if (op:equalsValue(!locationId,"none()")) {
        exit(FAIL,not(wasSupplied(anyLocation)));
    }
    !tmpString = op:invokeMethod(!locationId, "getName");
    !tmp = op:createSymbol(!tmpString);
    op:log(debug, "location: !tmp");

    while(~op:equalsValue(!locationId,"none()")){
        op:add(!locations, !tmp);
        !bindings = act:askQuestionFromString(?actor,"What's the next location on the path?", location(X));
        if (op:invokeMethod(!bindings,"isEmpty")) {
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
        !locationId = op:get(!bindings, !x);
        !tmpString = op:invokeMethod(!locationId, "getName");
        !tmp = op:createSymbol(!tmpString);
        op:log(debug, "location: !locationId");
    }
    act:generateResponseFromString("Added new patrol path ?name");
    !locationsPredicate = op:createPredicate("and", !locations);
    !collectionDefinition = op:createPredicate("collection(?name, X)");
    act:retractBelief(!collectionDefinition);
    !collectionDefinition = op:createPredicate("collection(?name, !locationsPredicate)");
    op:log(debug, "[definePatrolPath] collection definition: !collectionDefinition");
    act:assertBelief(!collectionDefinition);


}

() = patrol(Symbol ?collectionName, Symbol ?actionName) {

  Predicate !queryPred;
  Variable !x = "X";
  Predicate !locations;
  Map !elem;
  List !bindings;
  Predicate !goalPred;

  !queryPred = op:createPredicate("collection(?collectionName,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     !locations = op:get(!elem,!x);
     act:generateResponseFromString("Starting patrol on path ?collectionName with ?actionName");
     act:forEach(!locations, ?actionName);
  } else {
    exit(FAIL, not(known(patrol, ?collectionName)));
  }
  !goalPred = op:createPredicate("sayFinishedPatrol(?actor, ?collectionName, ?actionName)");
  act:submitGoal(!goalPred);
}

() = sayFinishedPatrol(Symbol ?collectionName, Symbol ?actionName) {
  act:generateResponseFromString("Finished patrol on path ?collectionName with ?actionName");
}

() = wifiEvaluation(Symbol ?locationId) {
    Integer !result = 0;
    Variable !x;

    effects : {
        success : not(wifi(?locationId, !x));
        success : wifi(?locationId, !result);
    }

    act:goToLocation(?locationId);
    op:log(info, "[wifiEvaluation] evaluating wifi quality at ?locationId");
    !result = act:getWifiSignalStrength();
}

() = reportWifiForLocation(Symbol ?location) {
  Predicate !queryPred;
  Variable !x = "X";
  Map !elem;
  List !bindings;
  Symbol !result;
  Predicate !resultPredicate;

  !queryPred = op:createPredicate("wifi(?location,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
    !elem = op:get(!bindings, 0);
    !result = op:get(!elem,!x);
    act:generateResponse("wifiQualityAt(?location, !result)");
  } else {
      exit(FAIL, not(known(wifiQualityAt(?location))));
  }
}

() = reportWifiForPatrol(Symbol ?collectionName) {
  Predicate !queryPred;
  Variable !x = "X";
  List !locations;
  Map !elem;
  List !bindings;
  Symbol !result;
  Predicate !resultPredicate;
  Symbol !location;

  !queryPred = op:createPredicate("collection(?collectionName,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     !result = op:get(!elem,!x);
     !locations = op:invokeMethod(!result, "getArgs");
     foreach(!location : !locations) {
        act:reportWifiForLocation(!location);
     }
  } else {
    exit(FAIL, not(known(patrol, ?collectionName)));
  }
}

() = takePhoto(Symbol ?location:location) {
    act:goToLocation(?location);
    act:generateResponse("taking(photo(at(?location)))");
}
