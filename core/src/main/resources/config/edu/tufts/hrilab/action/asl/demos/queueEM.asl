() = suspendSystemGoalsBlocking() {
    java.util.List !goalIds;
    java.lang.Long !goalId;
    java.util.List !systemGoalPredicates;
    java.lang.Integer !numGoals;
    java.lang.Boolean !success;
    edu.tufts.hrilab.fol.Predicate !pred;
    java.util.Map !bindings;
    java.util.List !bindingArgs;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Predicate !response;

    //Suspend system goal for agent
    !goalIds = tsc:suspendSystemGoals(?actor);
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
        !status = tsc:waitForNextGoalStatusUpdate(!goalId);
        if (op:isNull(!status) || ~op:equalsValue(!status,SUSPENDED)) {
            //Goal meant to be suspended has already terminated.
            !systemGoalPredicates = tsc:getSystemGoalsPredicates(?actor);
            if (op:isEmpty(!systemGoalsPredicates)) {
                //No new goals started execution as a result of this one completing, fail on suspend
                exit(FAIL,alreadyComplete(goalToSuspend));
            } else {
                !numGoals = op:size(!systemGoalPredicates);
                if (op:gt(!numGoals,1)) {
                    //TODO: just fail in this case?
                    //More than one goal started execution as a result, freeze and ask if desired to remain frozen
                    act:freeze(?actor);
                    !bindings = act:askQuestionFromString(?actor,"I have frozen execution because I could not suspend the desired task in time and multiple other tasks have started. Would you like me to keep execution frozen?", val(X));
                    if (op:invokeMethod(!bindings,"isEmpty")) {
                        //Question ignored, just end freeze and fail
                        act:endFreeze(?actor);
                        exit(FAIL,alreadyComplete(goalToSuspend));
                    }
                    !response = op:get(!bindings, !x);
                    if (op:equalsValue(!x,yes)) {
                        //TODO this can only happen is freeze/endFreeze are in the domain (which they should be if this case is possible)
                        return;
                    } elseif (op:equalsValue(!x,no)) {
                        act:generateResponseFromString("okay, resuming execution");
                        act:endFreeze();
                    } else {
                        act:endFreeze();
                        exit(FAIL,not(handle(answerToMyQuestion)));
                    }
                } else {
                    //One goal started execution as a result - attempt to suspend it and ask if want it to remain suspended
                    !pred = op:get(!systemGoalsPredicates,0);
                    !success = act:suspendGoalBlocking(!pred);
                    if (op:equals(!success,true)) {
                        !bindings = act:askQuestionFromString(?actor,"I could not suspend the desired task in time, but have suspended the next one. Would you like me to keep this task suspended?", val(X));
                        if (op:invokeMethod(!bindings,"isEmpty")) {
                            //Question ignored, just end freeze and fail
                            act:resumeGoal(!pred);
                            exit(FAIL,alreadyComplete(goalToSuspend));
                        }
                        !response = op:get(!bindings, !x);
                        if (op:equalsValue(!x,yes)) {
                            return;
                        } elseif (op:equalsValue(!x,no)) {
                            act:generateResponseFromString("okay, resuming execution");
                            act:resumeGoal(!pred);
                        } else {
                            act:resumeGoal(!pred);
                            exit(FAIL,not(handle(answerToMyQuestion)));
                        }
                    } else {
                        //If we manage to miss suspending the next goal as well, just fail out
                        exit(FAIL,alreadyComplete(goalToSuspend));
                    }
                }
            }
        }
    }
}

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

() = checkKnowsActiveGoals["describes ?actor's current active task(s)"](){

    effects : {
      success : knows(?actor,currentGoals(are(?actor,!currentGoalsPred)));
    }

    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;
    java.util.List !currentGoalsPreds = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    Predicate !currentGoalPred;
    Predicate !currentGoalsPred;
    java.lang.String !status;
    Predicate !goalPred;
    java.lang.Integer !goalsSize;
    Variable !tmpVar;
    Predicate !oldGoalsPred;

    op:log("info","[checkKnowsActiveGoals]");

    // construct and retract a generic predicate from belief to clear previous entries
    !tmpVar = op:newObject("edu.tufts.hrilab.fol.Variable", "X");
    !oldGoalsPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?actor,currentGoals(are(?actor,!tmpVar)))");
    act:retractBelief(!oldGoalsPred);

    !goalPreds = act:getSystemGoalsPredicates();
    !resultsSize = op:size(!goalPreds);
    if(op:gt(!resultsSize,0)){
        foreach(!goalPred: !goalPreds){
            !status= op:getName(!goalPred);
            !goalPred= op:getArg(!goalPred,0);
            !currentGoalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate",!status,!goalPred);
            op:add(!currentGoalsPreds,!currentGoalPred);
        }
        !goalsSize = op:invokeMethod(!currentGoalsPreds, "size");
        if (op:gt(!goalsSize,1)) {
            !currentGoalsPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","and",!currentGoalsPreds);
        } else {
            !currentGoalsPred = op:invokeMethod(!currentGoalsPreds,"get",0);
        }
    } else {
        !currentGoalsPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","doing(nothing)");
    }
    op:log("info","[checkKnowsActiveGoals] !currentGoalsPred");
}