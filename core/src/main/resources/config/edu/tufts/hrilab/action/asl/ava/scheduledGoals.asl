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
    Symbol !interval;
    Symbol !time;
    Symbol !addl;
    Symbol !intervalUnits;
    Symbol !timeSymbol = "none";
    java.util.List !responseFormList;
    Predicate !responseForm;

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "time(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "time(X,Y)");
    op:add(!responseFormList,!responseForm);


    !bindings = act:askQuestionFromString(?actor, "What would you like to do?", taskPredicate(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !goalPredicate = op:get(!bindings, !x);
    op:log(debug, "[scheduleGoal] goal !goalPredicate");

    //!bindings = act:askQuestionFromString(?actor, "When should I perform this goal?", time(X));
    //!time = op:get(!bindings, !x);
    //op:log(debug, "[scheduleGoal] time !time");

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
            !timeSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "notNone"); //this is a bad way to do this!
        }
    } else {
        !timeSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "notNone"); //this is a bad way to do this!
    }
    op:log(debug, "[scheduleGoal] time/units: !time/!addl");

    !bindings = act:askQuestionFromString(?actor, "Should this task be repeated?", val(X));
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !confirmation = op:get(!bindings, !x);

    //TODO: clean this up
    if (op:equalsValue(!confirmation, yes())) {
      !bindings = act:askQuestionFromString(?actor, "How often?", !responseFormList);
      if (op:invokeMethod(!bindings,"isEmpty")) {
          exit(FAIL,not(wasAnswered(myQuestion)));
      }
      !interval = op:get(!bindings, !x);
      !intervalUnits = op:get(!bindings, !y);
      if (op:equalsValue(!timeSymbol, "none")) {
          if (op:isNull(!intervalUnits)) {
            !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval)), !time, !addl)");
          } else {
            if (op:equalsValue(!intervalUnits, pm) || op:equalsValue(!intervalUnits, am)) {
                !interval = act:parseTime(!interval, !intervalUnits);
                !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval)), !time, !addl)");
            } else {
                !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval, !intervalUnits)), !time, !addl)");
            }
          }
      } else  {
          if (op:isNull(!intervalUnits)) {
            !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval)), !time)");
          } else {
            if (op:equalsValue(!intervalUnits, pm) || op:equalsValue(!intervalUnits, am)) {
                !interval = act:parseTime(!interval, !intervalUnits);
                !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval)), !time)");
            } else {
                !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval, !intervalUnits)), !time)");
            }
          }
      }
    } else {
      if (op:equalsValue(!timeSymbol, "none")) {
          !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, !goalPredicate, !time, !addl)");
      } else {
          !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, !goalPredicate, !time)");
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
    java.lang.Boolean !modifyInnerGoal = false;
    java.lang.Integer !numArgs;
    java.util.List !responseFormList;
    Predicate !responseForm;

    op:log("info", "[modifyScheduledGoal] in method");
    !scheduledGoalPred = act:selectScheduledGoal();
    !goalId = tsc:getGoalId(!scheduledGoalPred);
    op:log("info", "[modifyScheduledGoal] have goal !scheduledGoalPred with id !goalId");

    //TODO: theoretically could have endlessly nested scheduled/persistent goals, not sure how to handle that generally other than canceling/resubmitting with entire new specification
    !goalPredName = op:invokeMethod(!scheduledGoalPred, "getName");
    if (op:equals(!goalPredName,"persistent")) {
        !innerGoalPred = op:invokeMethod(!scheduledGoalPred,"get",1);
        !innerGoalPredName = op:invokeMethod(!innerGoalPred, "getName");
        if (op:equals(!innerGoalPredName,"scheduled")) {
            //Ask to modify either inner or outer timing
            !bindings = act:askQuestionFromString(?actor, "Would you like to modify the timing of the first execution, or the interval at which it is executed?", timingOf(X));
            if (~op:invokeMethod(!bindings,"isEmpty")) {
                !response = op:get(!bindings,!x);
                if (op:equalsValue(!response,execution)) {
                    op:log("info", "[modifyScheduledGoal] got response to modify first execution");
                } elseif (op:equalsValue(!response,interval)) {
                    op:log("info", "[modifyScheduledGoal] got response to modify scheduled goal interval");
                    !modifyInnerGoal = op:newObject("java.lang.Boolean", true);
                } else {
                    exit(FAIL,not(understand(?actor,whatToModify)));
                }
            } else {
                exit(FAIL,not(understand(?actor,whatToModify)));
            }
        }
    }

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "time(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "time(X,Y)");
    op:add(!responseFormList,!responseForm);
    !bindings = act:askQuestionFromString(?actor, "When would you like to schedule this task instead?", !responseFormList);
    if (~op:invokeMethod(!bindings,"isEmpty")) {
        !response = op:get(!bindings,!x);
        !yResponse = op:get(!bindings,!y);
        op:log("debug", "!response");
        op:log("debug", "!yResponse");
        if (op:isNull(!yResponse)) {
            !success = tsc:updateScheduledGoalTime(!goalId, !response, !modifyInnerGoal);
        } else {
            if (op:equalsValue(!yResponse, pm) || op:equalsValue(!yResponse, am)) {
                !response = act:parseTime(!response, !yResponse);
                !success = tsc:updateScheduledGoalTime(!goalId, !response, !modifyInnerGoal);
            } else {
                !success = tsc:updateScheduledGoalTime(!goalId, !response, !yResponse, !modifyInnerGoal);
            }
        }
        if (op:equals(!success,True)) {
            act:generateResponseFromString("Ok");
            return;
        } else {
            exit(FAIL,cannot(parse(?actor,theSuppliedTime)));
        }
    }
    exit(FAIL,not(supplied(timing)));

    ////todo: remove this hack. should query for actual goals and actual semantics
    //act:sayText("There is one goal scheduled");
    //act:sayText("Scheduled goal #1 is:");
    //act:sayText("Perform lab patrol with wifi evaluation at 12:00 p.m.");

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

    !scheduledGoals = act:getScheduledGoals();
    op:log(debug, "[modifyScheduledGoal] current scheduled goals: !scheduledGoals");
    !numGoals = op:invokeMethod(!scheduledGoals, "size");

    if (op:isEmpty(!scheduledGoals)) {
        //TODO: test NLG
        exit(FAIL,not(exist(scheduledGoals)));
        //act:generateResponse("There are no currently scheduled goals");
        //return;
    } elseif (op:equals(!numGoals,1)) {
        ?scheduledGoalPred = op:get(!scheduledGoals,0);
        act:generateResponseFromString("There is currently one scheduled task");
        act:generateResponse(?scheduledGoalPred);
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
    List !scheduledGoalIds;
    Integer !numGoals;
    Predicate !goalPred;
    Long !gid;
    edu.tufts.hrilab.action.goal.GoalInfo !goalInfo;
    java.lang.Boolean !isPersistent;
    java.lang.Boolean !isScheduled;

    !scheduledGoalIds = tsc:getScheduledGoalIds();
    //op:log(debug, "[listScheduledGoals] current scheduled goals: !scheduledGoals");
    !numGoals = op:invokeMethod(!scheduledGoalIds, "size");
    if (op:isEmpty(!scheduledGoalIds)) {
        act:generateResponseFromString("There are no scheduled tasks");
    } else {
        foreach (!gid : !scheduledGoalIds) {
            !goalPred = tsc:getWrappedScheduledGoalPredicate(!gid);
            act:generateResponse(!goalPred);
        }
    }
}

() = forEach(Predicate ?collection, Symbol ?actionName) {
    Predicate !goalPred;
    List !args;
    Symbol !arg;
    Long !gid;
    GoalStatus !goalStatus;
    Justification !failureJustification;
    List !failureReasons;

    onInterrupt : {
      cancel : act:cancelGoal(!gid);
      suspend : act:suspendGoal(!gid);
      resume : act:resumeGoal(!gid);
    }

    !args = op:invokeMethod(?collection, "getArgs");
    foreach (!arg : !args) {
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?actionName(?actor, !arg)");
        !gid = act:submitGoal(!goalPred);
        !goalStatus = act:joinOnGoal(!gid);
        if (op:invokeMethod(!goalStatus, "isFailure")) {
          !failureJustification = act:getGoalFailConditions(!gid);
          exit(FAIL, !failureJustification);
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
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "pathName(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "taskName(X)");
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
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !tmpString);
    op:log(debug, "location: !tmp");

    while(~op:equalsValue(!locationId,"none()")){
        op:add(!locations, !tmp);
        !bindings = act:askQuestionFromString(?actor,"What's the next location on the path?", location(X));
        if (op:invokeMethod(!bindings,"isEmpty")) {
            exit(FAIL,not(wasAnswered(myQuestion)));
        }
        !locationId = op:get(!bindings, !x);
        !tmpString = op:invokeMethod(!locationId, "getName");
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !tmpString);
        op:log(debug, "location: !locationId");
    }
    act:generateResponseFromString("Added new patrol path ?name");
    !locationsPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and", !locations);
    !collectionDefinition = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?name, X)");
    act:retractBelief(!collectionDefinition);
    !collectionDefinition = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?name, !locationsPredicate)");
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

  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?collectionName,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     !locations = op:get(!elem,!x);
     act:generateResponseFromString("Starting patrol on path ?collectionName with ?actionName");
     act:forEach(!locations, ?actionName);
  } else {
    exit(FAIL, known(patrol, ?collectionName));
  }
  !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "sayFinishedPatrol(?actor, ?collectionName, ?actionName)");
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

  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "wifi(?location,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
    !elem = op:get(!bindings, 0);
    !result = op:get(!elem,!x);
    act:generateResponse("wifiQualityAt(?location, !result)");
  } else {
      exit(FAIL, known(wifi(?location)));
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

  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?collectionName,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     !result = op:get(!elem,!x);
     !locations = op:invokeMethod(!result, "getArgs");
     foreach(!location : !locations) {
        act:reportWifiForLocation(!location);
     }
  } else {
    exit(FAIL, known(patrol, ?collectionName));
  }
}


//TODO: these can probably just be moved to a core asl script (though is
() = describeCurrentGoal["describes ?actor's current active task(s)"](){

    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;
    java.lang.String !status;
    Predicate !goalPred;

    !goalPreds = act:getSystemGoalsPredicates();
    !resultsSize = op:size(!goalPreds);
    if(op:gt(!resultsSize,0)){
        foreach(!goalPred: !goalPreds){
            !status= op:getName(!goalPred);
            !goalPred= op:getArg(!goalPred,0);
            act:generateResponseFromString("!status task: ");
            act:generateResponse(!goalPred);
        }
    } else {
        act:generateResponseFromString("no current task");
    }
}

() = describePendingGoals["describes ?actor's pending goals task"](){
    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;

    !goalPreds = act:getPendingGoals();
    !resultsSize = op:size(!goalPreds);

    if(op:gt(!resultsSize,0)){
       foreach(!goalPred: !goalPreds){
           act:generateResponse(!goalPred);
       }
    } else{
       act:generateResponseFromString("no pending tasks");
    }

}

() = repeatLastQuestion() {
    java.lang.String !questionString;
    !questionString = tsc:getLastQuestion(?actor);
    if (op:invokeMethod(!questionString,"isEmpty")) {
        act:generateResponseFromString("I have no currently pending questions");
    } else {
        act:generateResponseFromString(!questionString);
    }
}

() = takePhoto(Symbol ?location:location) {
    act:goToLocation(?location);
    act:generateResponse("taking(photo(at(?location)))");
}