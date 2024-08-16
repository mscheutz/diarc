import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;

() = checkCapableOf["checks if post condition is capable of bing reached by ?actor"](Predicate ?goal) {
    Term !toAssert;
    Term !otherActor;

    effects : {
      success : knows(?actor,capableOf(?goal));
    }

    if (act:actionExists(?goal)) {
      !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "capableOf(?goal)");
      act:assertBelief(!toAssert);
    } else {
      // TODO: this shouldn't actually fail, but is currently needed to handle the inconsistent
      //       way that ITKs are handled
      exit(FAIL);
    }
}

// Get the diarc actor from the ?query. Returns null if no diarc actors are found.
(Symbol ?otherActor) = getActorFromQuery(Predicate ?query) {
    java.util.List !leaves;

    // try to extract other actors from ?query, so ?actor knows who to "ask"
    !leaves = op:invokeMethod(?query, "getOrderedLeaves");
    ?otherActor = op:setNull();
    foreach(!leaf : !leaves) {
      if (act:isDiarcAgent(!leaf)) {
        ?otherActor = op:newObject("edu.tufts.hrilab.fol.Symbol", !leaf);
      }
    }
}

// Ask ?otherActor about ?query. This is used for ITKs when the query state is about another actor.
// e.g., dempster, does shafer see an obstacle
() = askAbout(Symbol ?otherActor, Predicate ?query, Predicate ?returnQuery) {

    if (?otherActor.goal:knows(?otherActor,?query)) {
      op:log("debug", "askActor: knows goal succeeded.");
      ?returnQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?otherActor,?query)");
    } elseif (?otherActor.goal:observe(?query)) {
      op:log("debug", "askActor: observation goal succeeded.");
      ?returnQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?query");
    } else {
      op:log("debug", "askActor: knows and obs goal failed, ?otherActor doesn't know about query: ?query.");
      exit(FAIL);
    }
}

() = getCurrGoals["get list of current goals and assert it to belief"]() {
    java.util.List !allGoals;
    java.util.List !filteredGoals;
    Predicate !currentGoals;
    Predicate !oldGoals;
    Predicate !tmpPred;
    Variable !tmpVar;
    java.lang.Integer !goalSize;
    java.lang.String !goalName;

    effects : {
      success : knows(?actor,currently(is(?actor,!tmpPred)));
    }

    op:log("debug", "start get current goals");

    // get current goals for ?actor
    !allGoals = act:getCurrentGoals();

    // remove handling nlp goal and getting currgoals from the goal list
    !filteredGoals = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    foreach (!currGoal : !allGoals) {
      !goalName = op:invokeMethod(!currGoal, "getName");
      if (~op:equals(!goalName, "handled") && ~op:equals(!goalName, "getCurrGoals")) {
        op:invokeMethod(!filteredGoals, "add", !currGoal);
      }
    }

    // construct and retract a generic predicate from belief to clear previous entries
    !tmpVar = op:newObject("edu.tufts.hrilab.fol.Variable", "X");
    !oldGoals = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?actor,currently(is(?actor,!tmpVar)))");
    act:retractBelief(!oldGoals);

    // do null check for action
    !goalSize = op:invokeMethod(!filteredGoals, "size");
    if (op:gt(!goalSize, 1)) {
      !tmpPred = op:newObject("edu.tufts.hrilab.fol.Predicate", "and", !filteredGoals);
    } elseif (op:isEmpty(!filteredGoals)) {
      !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "doing(nothing)");
    } else {
      !tmpPred = op:invokeMethod(!filteredGoals, "get", 0);
    }

    op:log("debug", "end get current goals");
}

() = getTime["get the system time and assert it to belief"]() {
    java.time.LocalTime !time;
    Term !toAssert;
    java.lang.Integer !hourInt;
    java.lang.Integer !minInt;
    Symbol !hour;
    Symbol !min;

    effects : {
      success : knows(?actor,currentTime(!hour,!min));
    }

    // retract old time
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?actor,currentTime(HOUR,MIN))");
    act:retractBelief(!toAssert);

    !time = op:invokeStaticMethod("java.time.LocalTime", "now");
    op:log("info", "time is !time");
    !hourInt = op:invokeMethod(!time, "getHour");
    !minInt = op:invokeMethod(!time, "getMinute");
    !hour = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!hourInt");
    !min = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!minInt");
}

(Predicate ?ret) = getActDesc["get step by step of action description and assert it to belief "](Predicate ?goalPred) {
    Goal !goal;
    edu.tufts.hrilab.action.selector.ActionSelector !actionSelector;
    edu.tufts.hrilab.action.ParameterizedAction !parameterizedAction;
    Predicate !actionDescription;
    Predicate !oldDescription;
    edu.tufts.hrilab.action.ActionConstraints !ac;
    edu.tufts.hrilab.action.state.StateMachine !sm;
    edu.tufts.hrilab.action.db.ActionDBEntry !actionDBEntry;
    java.util.List !eventSpecs;
    Predicate !stepPredicate;
    Variable !tmpVar;
    Predicate !tmpPred;
    java.util.List !actDescArgs;
    java.util.List !oldDescArgs;
    java.lang.Double !prob;
    java.lang.String !probstr;
    Symbol !probsym;
    Predicate !failCond;


    effects : {
      success : knows(?actor, actionDescription(?goalPred,!stepPredicate));
    }

    op:log("debug", "start get action description");

    // construct and retract a generic predicate from belief to clear previous entries
    !oldDescription = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?actor,actionDescription(?goalPred,Y))");
    act:retractBelief(!oldDescription);

    // get StateMachine for use in action selection
    !sm = act:getStateMachine();

    // pick action, and get event specs from selected action
    !goal = op:newObject("edu.tufts.hrilab.action.goal.Goal", ?goalPred);
    !ac = op:newObject("edu.tufts.hrilab.action.ActionConstraints");
    !actionSelector = op:invokeStaticMethod("edu.tufts.hrilab.action.selector.ActionSelector", "getInstance");
    !parameterizedAction = op:invokeMethod(!actionSelector, "selectActionForGoal", !goal, !ac, !sm);

    // do null check for action
    if (op:isNull(!parameterizedAction)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,how(?goalPred)))");
        exit(FAIL, !failCond);
    }

    // construct final predicate to assert to belief
    !actDescArgs = op:newObject("java.util.ArrayList");
    !oldDescArgs = op:newObject("java.util.ArrayList");

    // construct and retract a generic predicate from belief to clear previous entries
    !tmpVar = op:newObject("edu.tufts.hrilab.fol.Variable", "X");
    op:invokeMethod(!oldDescArgs, "add", ?goalPred);
    op:invokeMethod(!oldDescArgs, "add", !tmpVar);
    !oldDescription = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionDescription", !oldDescArgs);
    act:retractBelief(!oldDescription);

    // build and assert final predicate to belief
    !stepPredicate = op:invokeMethod(!parameterizedAction, "getStepsInPredicateForm");
    op:invokeMethod(!actDescArgs, "add", ?goalPred);
    op:invokeMethod(!actDescArgs, "add", !stepPredicate);
    !actionDescription = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionDescription", !actDescArgs);
    ?ret = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionDescription", !actDescArgs);

    act:assertBelief(!actionDescription);

    // retract a generic predicate from belief to clear previous entries
    !oldDescription = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "probabilityOf", !oldDescArgs);
    act:retractBelief(!oldDescription);

    // get success probability and add it to belief
    op:log("warn", "getSuccessProbability is currently disabled.");
    //op:invokeStaticMethod("edu.tufts.hrilab.action.db.Database", "getSuccessProbability", !parameterizedAction, !sm, !prob);
    //op:log("debug", "action prob !prob");
    //op:invokeStaticMethod("java.lang.Double", "toString", !prob, !probstr);
    //!tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "probabilityOf(?goalPred,!probstr)");
    //act:assertBelief(!tmpPred);
    op:log("debug", "end get action description");
}

() = getContextDescription["get description for step of (specified by location) currently executing goal "](Predicate ?location) {
    int !numLocArgs;
    Predicate !goalPredicate;
    java.util.List !goals;
    Goal !goal;
    edu.tufts.hrilab.action.description.ContextDescription !contextDescription;
    java.util.List !contextPredicates;
    Predicate !oldDescription;
    Predicate !failCond;
    Predicate !steps;

    effects : {
      success : knows(?actor,contextDescription(?location,!steps));
    }

    op:log("debug", "start get context description");

    // construct and retract a generic predicate from belief to clear previous entries
    !oldDescription = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(A,contextDescription(X,Y))");
    act:retractBelief(!oldDescription);

    // get Goal instance from goalPredicate
    !numLocArgs = op:invokeMethod(?location, "size");
    if (op:==(!numLocArgs, 2)) {
      !goalPredicate = op:invokeMethod(?location, "get", 1);
    } else {
      !goalPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(mostRecent)");
    }

    // if asking about sub-goal (e.g., nod part of dance goal) then we will need to get current goal
    // instead of goal specified in the goalPredicate-->
    // TODO: this assumes that an agent only has a single top-level goal
    try {
      !goals = act:getGoal(!goalPredicate);
      !goal = op:get(!goals,0);
    } catch(FAIL) {
      !goalPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(mostRecent)");
      !goals = act:getGoal(!goalPredicate);
      !goal = op:get(!goals,0);
    }

    // get ContextDescription from goal's action interpreter based on ?location
    !contextDescription = op:invokeMethod(!goal, "getContextDescription", ?location);

    // check if goal was found
    if (op:isNull(!contextDescription)) {
        op:log("debug", "[getContextDescription] no matching context description found.");
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(propertyOf(action,found))");
        exit(FAIL, !failCond);
    }

    // get relevant predicate descriptions (will be asserted to belief via the "knows" post-conds)
    !steps = op:invokeMethod(!contextDescription, "getStepsInPredicateForm");

    op:log("debug", "end get context description");
}

() = forgetThat(Symbol ?requester, Predicate ?predicate) {
    Predicate !belPred;
    Predicate !queryPred;
    Symbol !agent;
    java.util.List !bindings;
    java.util.Collection !valueset;
    java.util.HashMap !elem;
    Predicate !failCond;

    conditions : {
        pre : bel(?actor,?predicate);
        pre : trust(?actor,?requester);
    }

    // TODO: EAK: can't we just retract bel(X,?predicate) instead of having to iterate over all diarc agents?
//      !belPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "believes(X,?predicate)");
//      op:log("info", "forgetting !belPred");
//      act:retractBelief(!belPred);

    // retract where X equals diarcAgent
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "diarcAgent(?actor)");

    if (act:querySupport(!queryPred)) {
      !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "diarcAgent(X)");
      !bindings = act:queryBelief(!queryPred);
      foreach(!elem : !bindings) {
        !valueset = op:invokeMethod(!elem, "values");
        foreach(!agent : !valueset) {
          !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "diarcAgent(!agent)");
          if (act:querySupport(!queryPred)) {
            !belPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "believes(!agent,?predicate)");
            op:log("info", "forgetting !belPred");
            act:retractBelief(!belPred);
          }
        }
      }
    } else {
      !belPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "believes(?actor,?predicate)");
      op:log("info", "forgetting !belPred");
      act:retractBelief(!belPred);
    }

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "bel(?actor,?predicate)");
    if (act:querySupport(!queryPred)) {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "is(that,protectedInformation)");
      exit(FAIL, !failCond);
    }
}

() = forgetHow(Symbol ?requester, Predicate ?actionSignature) {
    conditions : {
        pre : trust(?actor,?requester);
    }

    op:log("debug", "forgetting ?effect");
    act:removeActionsWithSignature(?actionSignature);
}

() = tell(Predicate ?goal) {
    long !goalID;
    Symbol !addressee;
    GoalStatus !goalStatus;
    edu.tufts.hrilab.action.ActionStatus !actionStatus;
    edu.tufts.hrilab.action.justification.Justification !failConds;
    java.lang.Boolean !val;

    try {
      // submit goal for other actor
      !goalID = act:submitGoal(?goal);

      // return from join every 250 ms so tell action can be cancelled
      !goalStatus = act:joinOnGoal(!goalID, 250);
      while (~op:invokeMethod(!goalStatus, "isTerminated")) {
        !goalStatus = act:joinOnGoal(!goalID, 250);
      }
    } catch(CANCEL) {
      op:log("debug", "catching cancel and canceling sub-goal");
      act:cancelGoal(!goalID);
    }

    // if goal failed, relay failures
    !actionStatus = act:getActionStatus(!goalID);
    if (op:invokeMethod(!goalStatus, "isFailure")) {
      !failConds = act:getGoalFailConditions(!goalID);
      exit(!actionStatus, !failConds);
    }
}

() = cancelGoal(Predicate ?goalPredicate = goal(mostRecent)) {
    java.util.List !goals;
    Goal !goal;
    java.lang.Long !goalId;

    !goals = act:getGoal(?goalPredicate);
    if (~op:isEmpty(!goals)) {
        !goal = op:get(!goals,0);
        op:log("debug", "[cancelGoal] canceling goal: !goal.");
        !goalId =  op:invokeMethod(!goal, "getId");
        act:cancelGoal(!goalId);
    } else {
        op:log("warn", "[cancelGoal] No goal found for predicate: ?goalPredicate.");
    }
}

() = suspendGoal(Predicate ?goalPredicate = goal(mostRecent)) {
    java.util.List !goals;
    Goal !goal;
    java.lang.Long !goalId;

    !goals = act:getGoal(?goalPredicate);
    if (~op:isEmpty(!goals)) {
        !goal = op:get(!goals,0);
        op:log("debug", "[suspendGoal] suspending goal: !goal.");
        !goalId =  op:invokeMethod(!goal, "getId");
        act:suspendGoal(!goalId);
    } else {
         op:log("warn", "[suspendGoal] No goal found for predicate: ?goalPredicate.");
     }
}

() = resumeGoal(Predicate ?goalPredicate = goal(mostRecent)) {
    java.util.List !goals;
    Goal !goal;
    java.lang.Long !goalId;

    !goals = act:getGoal(?goalPredicate);
    if (~op:isEmpty(!goals)) {
        !goal = op:get(!goals,0);
        op:log("debug", "[resumeGoal] resuming goal: !goal.");
        !goalId =  op:invokeMethod(!goal, "getId");
        act:resumeGoal(!goalId);
    } else {
          op:log("warn", "[resumeGoal] No goal found for predicate: ?goalPredicate.");
      }
}

(java.util.List ?goals) = getGoal(Predicate ?goalPredicate = goal(mostRecent)) {
    Predicate !mostRecentGoalPred;
    Goal !queryGoal;
    Goal !goal;

    !mostRecentGoalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(mostRecent)");
    if (op:equals(?goalPredicate, !mostRecentGoalPred)) {
      // find 3rd most recently submitted goal (this goal will be the most recent
      // goal that isn't this cancel goal or a dialogue goal)
      //          BEFORE goal submission. Change back if changing that (in achieveState in handleSemantics.asl)
      //      Additional issue is that if the sayText immediately terminates, whether because it fails or isn't
      //      implemented in a mock setup, then this index will be incorrect
        ?goals = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
        !goal = act:getCurrentGoal(?actor, 1);
        if (~op:isNull(!goal)) {
            op:add(?goals,!goal);
        }
    } else {
      // find matching goal
      !queryGoal = op:newObject("edu.tufts.hrilab.action.goal.Goal", ?goalPredicate);
      ?goals = act:getCurrentGoals(!queryGoal);
    }

    // check if goal was found
    if (op:isEmpty(?goals)) {
      op:log("debug", "[getGoal] no matching current goal found.");
      exit(FAIL, not(propertyOf(goal,found)));
    }

    op:log("debug", "[getGoal] have goals: ?goals.");
}

//====================== Translation Generation =======================

() = translateGoal(edu.tufts.hrilab.fol.Predicate ?goal){
   java.lang.Integer !contextID;

   (!contextID)= act:getContextForGoal(?goal);
   act:translate(!contextID);
}

() = translateLastGoal(){
  java.lang.Integer !contextCount;

  //get size of Context tree
  (!contextCount) = act:getCurrentContextCount();

  //remove index of this goal
  (!contextCount) = op:-(!contextCount,4);

  act:translate(!contextCount);
}

() = estimatePerformanceMeasures(edu.tufts.hrilab.fol.Predicate ?goalPred, edu.tufts.hrilab.fol.Symbol ?temporal, edu.tufts.hrilab.fol.Predicate ?assessmentModification = "if(none())") {
//() = estimatePerformanceMeasures(edu.tufts.hrilab.fol.Predicate ?goalPred, edu.tufts.hrilab.fol.Symbol ?temporal, edu.tufts.hrilab.fol.Predicate ?assessmentModification,  java.lang.Integer ?numSamples) {
  edu.tufts.hrilab.fol.Symbol !prob;
  edu.tufts.hrilab.fol.Symbol !duration;
  edu.tufts.hrilab.fol.Symbol !mostLikely;
  org.apache.commons.lang3.tuple.Triple ?assessment;

  effects : {
    success : knows(?actor,probabilityOf(?goalPred, ?temporal, !prob));
    success : knows(?actor,probabilityOf(?goalPred, ?temporal, ?assessmentModification, !prob));
    success : knows(?actor,durationOf(?goalPred, ?temporal, !duration));
    success : knows(?actor,durationOf(?goalPred, ?temporal, ?assessmentModification, !duration));
    success : knows(?actor,mostLikelyToFailOf(?goalPred, ?temporal, !mostLikely));
    success : knows(?actor,mostLikelyToFailOf(?goalPred, ?temporal, ?assessmentModification, !mostLikely));
  }
  ?assessment = op:estimatePerformanceMeasures(?goalPred, ?temporal, ?assessmentModification, 100);
  if (~op:isNull(?assessment)) {
    !prob = op:invokeMethod(?assessment, "getLeft");
    !duration = op:invokeMethod(?assessment, "getMiddle");
    !mostLikely = op:invokeMethod(?assessment, "getRight");
  }
}
