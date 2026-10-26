import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.action.goal.GoalFilter;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.Justification;
import java.lang.Long;

() = resumeCurrentTask() {
    act:resumeSystemGoals();
    //act:generateResponseFromString("Resumed");
}

() = suspendCurrentTask() {
    act:suspendSystemGoalsBlocking();
    //act:generateResponseFromString("Suspended");
}

() = cancelCurrentTask() {
    act:cancelSystemGoalsWrapper();
    //act:generateResponseFromString("Current task has been canceled");
}

() = cancelAllQueuedTasks() {
    act:cancelAllPendingGoals();
    //act:generateResponseFromString("Queued tasks have been canceled");
}

() = cancelAllCurrentAndQueuedTasks() {
    act:cancelAllPendingGoals();
    act:cancelSystemGoalsWrapper();
    //act:generateResponseFromString("Existing tasks have been canceled");
}

() = cancelQueuedTasks(Predicate ?indices) {
    tsc:cancelPendingGoalsByIndices(?indices);
}

() = suspendSystemGoalsBlocking() {
    java.util.List !goalIds;
    java.lang.Long !goalId;
    java.util.List !systemGoalPredicates;
    java.lang.Integer !numGoals;
    java.lang.Boolean !success = false;
    Predicate !pred;
    java.util.Map !bindings;
    java.util.List !bindingArgs;
    Variable !x = "X";
    Predicate !response;
    Predicate !freezeGoal;
    Justification !failureConditions;
    GoalStatus !status;
    java.lang.Long !timeout = 0;
    GoalFilter !systemGoalFilter = SYSTEM;

    //Suspend system goal for agent
    !goalIds = tsc:suspendSystemGoals(?actor);
    op:log("debug","[suspendSystemGoalsBlocking] suspended goalIds: !goalIds");
    if (op:isNull(!goalIds)) {
        //Returning null means multiple disjoint children have goals
        exit(FAIL,not(know(?actor,whichGoalToSuspend)));
    } elseif (op:isEmpty(!goalIds)) {
        //No valid goal to suspend was found (whether already suspended or none exist)
        exit(FAIL,not(found(activeGoalToSuspend)));
    } else {
        //Should never return more than one goalId in the queue case.
        //Wait for goal to be suspended
        !goalId = op:get(!goalIds,0);
        !status = tsc:getGoalStatus(!goalId);
        op:log("debug","[suspendSystemGoalsBlocking] have initial status for goal to suspend: !status");
        if (~op:equalsValue(!status,SUSPENDED)) {
            !status = tsc:waitForNextGoalStatusUpdate(!goalId,!timeout,!status);
            op:log("debug","[suspendSystemGoalsBlocking] have status update for suspended goal: !status");
        }
        //Need to write this way because we can't short circuit ||, throws exception when comparing null
        if (~op:isNull(!status)) {
            if (op:equalsValue(!status,SUSPENDED)) {
                !success = op:newObject("java.lang.Boolean", "true");
            }
        }
        if (op:equalsValue(!success,false)) {
            //Goal meant to be suspended has already terminated.
            !systemGoalPredicates = tsc:getGoalPredicatesForActor(!systemGoalFilter, ?actor);
            op:log("debug","[suspendSystemGoalsBlocking] Suspend failed, current goals now: !systemGoalPredicates");
            if (op:isEmpty(!systemGoalPredicates)) {
                //No new goals started execution as a result of this one completing, fail on suspend
                exit(FAIL,alreadyComplete(goalToSuspend));
            } else {
                !numGoals = op:invokeMethod(!systemGoalPredicates, "size");
                op:log("debug","[suspendSystemGoalsBlocking] found !numGoals new goals");
                if (op:gt(!numGoals,1)) {
                    op:log("debug","[suspendSystemGoalsBlocking] freezing to ask user what they want execution state to be");
                    //TODO: just fail in this case?
                    //More than one goal started execution as a result, freeze and ask if desired to remain frozen
                    !freezeGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "freeze",?actor,?actor);
                    act:submitGoal(!freezeGoal);
                    !bindings = act:askQuestionFromString("I have frozen execution because I could not suspend the desired task in time and multiple other tasks have started. Would you like me to keep execution frozen?", val(X));
                    if (op:invokeMethod(!bindings,"isEmpty")) {
                        //Question ignored, just end freeze and fail
                        act:endFreeze(?actor);
                        exit(FAIL,alreadyComplete(goalToSuspend));
                    }
                    !response = op:get(!bindings, !x);
                    if (op:equalsValue(!response,yes())) {
                        //TODO this can only happen is freeze/endFreeze are in the domain (which they should be if this case is possible)
                        return;
                    } elseif (op:equalsValue(!response,no())) {
                        act:generateResponseFromString("okay, resuming execution");
                        act:endFreeze(?actor);
                    } else {
                        act:endFreeze(?actor);
                        exit(FAIL,not(handle(answerToMyQuestion)));
                    }
                } else {
                    op:log("debug","[suspendSystemGoalsBlocking] attempting to suspend new goal");
                    //One goal started execution as a result - attempt to suspend it and ask if want it to remain suspended
                    !pred = op:get(!systemGoalPredicates,0);
                    try {
                        act:suspendGoalBlocking(!pred);
                        !success = op:newObject("java.lang.Boolean", "true");
                    } catch (!failureConditions) {
                        op:log("info","[suspendSystemGoalBlocking] caught failure justification from suspending next goal !failureConditions");
                        !success = op:newObject("java.lang.Boolean", "false");
                    }
                    if (op:equals(!success,true)) {
                        op:log("debug","[suspendSystemGoalsBlocking] Suspend of new goal succeeded, asking for desired state");
                        !bindings = act:askQuestionFromString("I could not suspend the desired task in time, but have suspended the next one. Would you like me to keep this task suspended?", val(X));
                        if (op:invokeMethod(!bindings,"isEmpty")) {
                            //Question ignored, just end freeze and fail
                            act:resumeGoal(!pred);
                            exit(FAIL,alreadyComplete(goalToSuspend));
                        }
                        !response = op:get(!bindings, !x);
                        op:log("debug","[suspendSystemGoalBlocking] have question response: !response");
                        if (op:equalsValue(!response,yes())) {
                            return;
                        } elseif (op:equalsValue(!response,no())) {
                            act:generateResponseFromString("okay, resuming execution");
                            act:resumeGoal(!pred);
                        } else {
                            act:resumeGoal(!pred);
                            exit(FAIL,not(handle(answerToMyQuestion)));
                        }
                    } else {
                        op:log("debug","[suspendSystemGoalsBlocking] Suspend of new goal failed, exiting as failure");
                        //If we manage to miss suspending the next goal as well, just fail out
                        exit(FAIL,alreadyComplete(goalToSuspend));
                    }
                }
            }
        }
    }
}

() = cancelSystemGoalsWrapper() {
    java.util.List !goalIds;
    java.lang.Long !goalId;
    java.util.List !systemGoalPredicates;
    java.lang.Integer !numGoals;
    java.lang.Boolean !success = false;
    Predicate !pred;
    java.util.Map !bindings;
    java.util.List !bindingArgs;
    Variable !x = "X";
    Predicate !response;
    Predicate !freezeGoal;
    Justification !failureConditions;
    GoalStatus !status;
    java.lang.Long !timeout = 0;
    GoalFilter !systemGoalFilter = SYSTEM;

    //cancel system goal for agent
    !goalIds = tsc:cancelSystemGoals(?actor);
    op:log("debug","[cancelSystemGoalsBlocking] canceled goalIds: !goalIds");
    if (op:isNull(!goalIds)) {
        //Returning null means multiple disjoint children have goals
        exit(FAIL,not(know(?actor,whichGoalToCancel)));
    } elseif (op:isEmpty(!goalIds)) {
        //No valid goal to cancel was found (whether already canceled or none exist)
        exit(FAIL,not(found(activeGoalToCancel)));
    } else {
        //Should never return more than one goalId in the queue case.
        //Wait for goal to be canceled
        !goalId = op:get(!goalIds,0);
        !status = tsc:getGoalStatus(!goalId);
        op:log("debug","[cancelSystemGoalsBlocking] have initial status for goal to cancel: !status");
        if (~op:equalsValue(!status,CANCELED)) {
            !status = tsc:waitForNextGoalStatusUpdate(!goalId,!timeout,!status);
            op:log("debug","[cancelSystemGoalsBlocking] have status update for canceled goal: !status");
        }
        //Need to write this way because we can't short circuit ||, throws exception when comparing null
        if (~op:isNull(!status)) {
            if (op:equalsValue(!status,CANCELED)) {
                !success = op:newObject("java.lang.Boolean", "true");
            }
        }

        if (op:equalsValue(!success,false)) {
            //Goal meant to be canceled has already terminated.
            !systemGoalPredicates = tsc:getGoalPredicatesForActor(!systemGoalFilter, ?actor);
            op:log("debug","[cancelSystemGoalsBlocking] Cancel failed, current goals now: !systemGoalPredicates");
            exit(FAIL,alreadyComplete(taskToCancel));
        }
    }
}

() = describeCurrentGoal["describes ?actor's current active task(s)"](){
    GoalStatus !status;
    java.lang.String !statusStr;
    Predicate !goalPred;
    Long !goalId;

    GoalFilter !goalFilter = SYSTEM;
    java.util.List !goalIds = tsc:getGoalIds(!goalFilter);
    op:log("info", "GoalIds: !goalIds");
    if (~op:isEmpty(!goalIds)) {
        foreach (!goalId: !goalIds) {
            !status = tsc:getGoalStatus(!goalId);
            !statusStr = op:invokeMethod(!status, "toString");
            !statusStr = op:invokeMethod(!statusStr, "toLowerCase");
            !goalPred = tsc:getGoalPredicate(!goalId);
            act:generateResponse(goal(status(!statusStr),predicate(!goalPred)));
            //act:reportGoalStatus(!status);
            //act:generateResponse(!goalPred);
        }
    } else {
        act:generateResponse(report(not(have(?actor, currentGoal))));
    }
}

() = describePendingGoals["describes ?actor's pending goals task"](){
    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;

    GoalFilter !pendingGoalFiler = PENDING;
    !goalPreds = act:getGoalPredicates(!pendingGoalFiler);
    !resultsSize = op:invokeMethod(!goalPreds, "size");

    if(op:gt(!resultsSize,0)){
       foreach(!goalPred: !goalPreds){
           act:generateResponse(!goalPred);
       }
    } else{
       act:generateResponseFromString("no pending tasks");
    }
}

() = checkKnowsActiveGoals["describes ?actor's current active task(s)"](){

    effects : {
      success : knows(?actor,currentGoals(are(?actor,!currentGoalsPred)));
    }

    java.lang.Integer !resultsSize=0;
    java.util.List !currentGoalsPreds = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    Predicate !currentGoalPred;
    Predicate !currentGoalsPred;
    java.lang.String !status;
    Predicate !goalPred;
    java.lang.Integer !goalsSize;
    Variable !tmpVar;
    Predicate !oldGoalsPred;
    long !goalId;

    op:log("debug","[checkKnowsActiveGoals]");

    // construct and retract a generic predicate from belief to clear previous entries
    !tmpVar = op:createVariable("X");
    !oldGoalsPred = op:createPredicate("knows(?actor,currentGoals(are(?actor,!tmpVar)))");
    act:retractBelief(!oldGoalsPred);

    GoalFilter !goalFilter = SYSTEM;
    java.util.List !goalIds = tsc:getGoalIds(!goalFilter);
    !resultsSize = op:size(!goalIds);
    if (op:gt(!resultsSize,0)) {
        foreach (!goalId: !goalIds) {
            !status = tsc:getGoalStatus(!goalId);
            !goalPred = tsc:getGoalPredicate(!goalId);
            !currentGoalPred = op:createPredicate(!status,!goalPred);
            op:add(!currentGoalsPreds,!currentGoalPred);
        }
        !goalsSize = op:invokeMethod(!currentGoalsPreds, "size");
        if (op:gt(!goalsSize,1)) {
            !currentGoalsPred = op:createPredicate("and",!currentGoalsPreds);
        } else {
            !currentGoalsPred = op:invokeMethod(!currentGoalsPreds,"get",0);
        }
    } else {
        !currentGoalsPred = op:createPredicate("doing(nothing)");
    }
    op:log("debug","[checkKnowsActiveGoals] !currentGoalsPred");
}