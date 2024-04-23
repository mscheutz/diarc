() = modifyLikeInteractive[""](edu.tufts.hrilab.fol.Predicate ?newScriptName, edu.tufts.hrilab.fol.Predicate ?oldScriptName) {

    //todo: does not modify always effect to match the new action (i.e. the new action will still have the old actions number in the always effect)
    //todo: current dict entries required modification of experiment action names for this modification to e.g. runExperiment7 (note "run" and numeral for number rather than text)

    edu.tufts.hrilab.fol.Predicate !likeGoal;
    java.util.Map !bindings;
    java.util.List !bindingArgs;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Predicate !modification;
    java.lang.String !modName;
    edu.tufts.hrilab.fol.Predicate !location = "none()";

    !likeGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "like", ?newScriptName, ?oldScriptName);

    !bindings = act:askQuestionFromString(?actor,"what are the differences", mod(X));
    !modification = op:get(!bindings, !x);
    op:log(debug, "modification !modification");
    act:modifyAction(!likeGoal,!modification,!location);

    !bindings = act:askQuestionFromString(?actor,"okay are there any more differences", mod(X));
    !modification = op:get(!bindings, !x);
    !modName = op:getName(!modification);
    while(op:!=(!modName,"none")){
        act:modifyAction(?newScriptName,!modification,!location);
        !bindings = act:askQuestionFromString(?actor,"okay are there any more differences", mod(X));
        !modification = op:get(!bindings, !x);
        !modName = op:getName(!modification);
    }

    act:generateResponseFromString("okay");
}

() = testGenerateLikeAction[""]() {

    edu.tufts.hrilab.fol.Predicate !oldScriptPred;
    edu.tufts.hrilab.fol.Predicate !newScriptPred;
    edu.tufts.hrilab.fol.Predicate !modification;
    edu.tufts.hrilab.fol.Predicate !modificationTwo;

    edu.tufts.hrilab.fol.Symbol !robot = "robotone:agent";
    edu.tufts.hrilab.fol.Symbol !location = "location_42:location";
    edu.tufts.hrilab.fol.Symbol !locationTwo = "location_50:location";

    !oldScriptPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "experimentSix()");
    !newScriptPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "experimentSeven()");

    !modification = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "replace(takeMaterialsToLocation(!locationTwo), takeMaterialsToLocation(!location))");
    !modificationTwo = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "replace(performComponentAnalysisAt(!locationTwo), performSpectroscopyAt(!location))");

    self.act:generateLikeAction(!newScriptPred, !oldScriptPred, !modification, !modificationTwo);

}

() = checkAgentsAndSubmitExperiments["top level policy for checking agent availability and submitting performExperimentAndRecover to assign experiments to agents. should be submitted as a persistent goal."]() {
    edu.tufts.hrilab.fol.Symbol !robotone = "robotone:agent";
    edu.tufts.hrilab.fol.Symbol !robotwo = "robottwo:agent";
    edu.tufts.hrilab.fol.Predicate !performExperimentPred;
    edu.tufts.hrilab.fol.Symbol !availableAgent;
    java.lang.Boolean !queueEmpty;


    op:log(debug, "[checkAgentsAndSubmitExperiments] executing top level policy");
    op:sleep(5000);

    !availableAgent = !robotone.act:getAvailableAgent();
    !queueEmpty = !robotone.act:isQueueEmpty();
//    op:log(debug, "[checkAgentsAndSubmitExperiments] !availableAgent");
//    op:log(debug, "[checkAgentsAndSubmitExperiments] !queueEmpty");

    if (~op:isNull(!availableAgent) && ~!robotone.act:isQueueEmpty()) {
        if (op:equals(!availableAgent,!robotone)) {
            !performExperimentPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "performExperimentAndRecover1(self:agent)");
        } else {
            !performExperimentPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "performExperimentAndRecover2(self:agent)");
        }
        //EW: need to get around check for same goal already existing - is there a good way to do this? The agents themselves don't have access to submitGoal
        //    and adding an explicit actor arg only works if the argument is the same as the actor performing the action. Revisit later.
        //    For now just making a duplicate action definition
        act:submitGoalDirectly(!performExperimentPred);
    }
//     else {
//        op:log(debug, "[checkAgentsAndSubmitExperiments] not submitting next experiment");
//    }
}

//todo: the reason there are two of these actions is because we aren't actually handling agent availability or allocation of tasks for repairs with the nasa goal queue implementation.
() = performExperimentAndRecover1["action for executing experiments at the head of the goal queue and recovering from individual tube failures reported bottom-up."]() {
    edu.tufts.hrilab.fol.Symbol !robotone = "robotone:agent";
    java.lang.Long !gid;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.fol.Predicate !failPredUnwrapped;
    edu.tufts.hrilab.fol.Symbol !failTube;
    edu.tufts.hrilab.fol.Predicate !failPred;
    java.util.List !failPredList;
    edu.tufts.hrilab.fol.Predicate !repairPred;

//    if (~!robotone.act:isQueueEmpty()) {
    op:log(info, "[performExperimentAndRecover] submitting head of queue");
    !gid = !robotone.act:submitQueueHead();
    !goalStatus = !robotone.act:joinOnGoalQueue(!gid);
    op:log(info, "[performExperimentAndRecover] out joinOnGoal with status !goalStatus");
    //If successful, allow this script to terminate in order to be resubmitted
    //otherwise, repair the tube which caused the failure.
    //todo: this is duplicating existing functionality in action recovery in asl. replace with correct pattern?
    if (op:invokeMethod(!goalStatus, "isFailure")) { //todo: need to check for cancelled case as well.
        try {
            act:getGoalFailConditions(!gid);
        }
        catch (FAIL_RETURNVALUE, !failConds) {
            op:log(info, "[performExperimentAndRecover] failure justification is !failConds");
            !failPredList = op:invokeMethod(!failConds, "getPredicates");
            !failPred = op:invokeMethod(!failPredList, "get", 0);
            !failPredUnwrapped = op:invokeMethod(!failPred, "get", 0);
            !failTube = op:invokeMethod(!failPredUnwrapped, "get", 0);

            op:log(info, "[performExperimentAndRecover] attempting to repair !failTube");
            !repairPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(!robotone, not(broken(!failTube)))");
            op:log(debug, "[performExperimentAndRecover] submitting new repair goal !repairPred");
            !robotone.act:confirmTubeBroken(!failTube);
            !gid = act:submitGoal(!repairPred);
            act:joinOnGoal(!gid);
        }
    }
//    }
}

() = performExperimentAndRecover2["action for executing experiments at the head of the goal queue and recovering from individual tube failures reported bottom-up."]() {
    edu.tufts.hrilab.fol.Symbol !robotone = "robottwo:agent";
    java.lang.Long !gid;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.fol.Predicate !failPredUnwrapped;
    edu.tufts.hrilab.fol.Symbol !failTube;
    edu.tufts.hrilab.fol.Predicate !failPred;
    java.util.List !failPredList;
    edu.tufts.hrilab.fol.Predicate !repairPred;

//    if (~!robotone.act:isQueueEmpty()) {
    op:log(info, "[performExperimentAndRecover] submitting head of queue");
    !gid = !robotone.act:submitQueueHead();
    !goalStatus = !robotone.act:joinOnGoalQueue(!gid);
    op:log(info, "[performExperimentAndRecover] out joinOnGoal with status !goalStatus");
    //If successful, allow this script to terminate in order to be resubmitted
    //otherwise, repair the tube which caused the failure.
    //todo: this is duplicating existing functionality in action recovery in asl. replace with correct pattern?
    if (op:invokeMethod(!goalStatus, "isFailure")) { //todo: need to check for cancelled case as well.
        try {
            act:getGoalFailConditions(!gid);
        }
        catch (FAIL_RETURNVALUE, !failConds) {
            op:log(info, "[performExperimentAndRecover] failure justification is !failConds");
            !failPredList = op:invokeMethod(!failConds, "getPredicates");
            !failPred = op:invokeMethod(!failPredList, "get", 0);
            !failPredUnwrapped = op:invokeMethod(!failPred, "get", 0);
            !failTube = op:invokeMethod(!failPredUnwrapped, "get", 0);

            op:log(info, "[performExperimentAndRecover] attempting to repair !failTube");
            !repairPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goal(!robotone, not(broken(!failTube)))");
            op:log(debug, "[performExperimentAndRecover] submitting new repair goal !repairPred");
            !robotone.act:confirmTubeBroken(!failTube);
            !gid = act:submitGoal(!repairPred);
            act:joinOnGoal(!gid);
        }
    }
//    }
}

() = execTopLevelPolicy["Execute the top level control policy for the spacestation, using both previously scheduled goals and response to failure."]() {

    edu.tufts.hrilab.fol.Symbol !robotone = "robotone:agent";
    java.lang.Long !gid;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;

    op:log(info, "[execTopLevelPolicy] executing health maintenance policy");
    op:sleep(1000);

    //Check if there is a goal waiting in the queue. If not, then allow this action to be called again
    //  (we could just make submitQueueHead block until a goal is added to the queue instead)
    if (~!robotone.act:isQueueEmpty()) {
        op:log(info, "[execTopLevelPolicy] found goals in queue, submitting head");
        //Submit the head of the queue
        !gid = !robotone.act:submitQueueHead();
        //Wait for the goal to terminate
        !goalStatus = !robotone.act:joinOnGoalQueue(!gid);
        op:log(info, "[execTopLevelPolicy] out joinOnGoal with status !goalStatus");
        //If successful, allow this script to terminate in order to be resubmitted
        //Otherwise enact health check/repair logic
        if (op:invokeMethod(!goalStatus, "isFailure")) { //todo: need to check for cancelled case as well.
            act:execHealthEvalAndRepair();
        }
    }
}

//todo: is it ok to be asserting to belief in this sense action on the state of tube breakage?
() = confirmTubeBroken["?actor senses to check if a tube is broken at the given location and updates belief accordingly"](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location) {
    conditions : {
        pre : at(?actor, ?tube);
    }

    java.lang.Boolean !tubeBroken;
    edu.tufts.hrilab.fol.Predicate !tubeBrokenStatePredicate;
    !tubeBrokenStatePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "broken", ?tube);

    !tubeBroken = ?actor.act:senseIfTubeBroken(?tube);

    if (op:equals(!tubeBroken, True)) {
        op:log(info, "[confirmTubeBroken] ?actor has confirmed that ?tube is broken");
        act:assertBelief(!tubeBrokenStatePredicate);
    }
    else {
        op:log(info, "[confirmTubeBroken] ?actor has identified that ?tube is NOT broken");
        act:retractBelief(!tubeBrokenStatePredicate);
    }
}

() = clearsamples[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location, edu.tufts.hrilab.fol.Symbol ?wing:area) {
    conditions : {
        pre : broken(?tube);
        pre : at(?actor, ?tube);
        pre : in(?tube, ?wing);
        pre : sealed(?wing);
    }
    effects : {
        success : clear(?tube);
    }
    op:log(info,"?actor clearing samples");
}

() = cleanfilter[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location, edu.tufts.hrilab.fol.Symbol ?prep:location, edu.tufts.hrilab.fol.Symbol ?wing:area) {
    conditions : {
        pre : broken(?tube);
        pre : at(?actor, ?prep);
        pre : bound(?tube, ?prep);
        pre : in(?tube, ?wing);
        pre : sealed(?wing);
        pre : clear(?tube);
    }

    effects : {
        success : clean(?prep);
    }

    op:log(info,"?actor cleaning filter");
}

() = patchleak[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location, edu.tufts.hrilab.fol.Symbol ?prep:location, edu.tufts.hrilab.fol.Symbol ?wing:area) {
    conditions : {
        pre : clear(?tube);
        pre : clean(?prep);
        pre : broken(?tube);
        pre : at(?actor, ?tube);
        pre : in(?tube, ?wing);
        pre : sealed(?wing);
    }

    effects : {
        success : repaired(?tube);
        success : not(broken(?tube));
    }

    ?actor.act:repairLocation(?tube);
    op:log(info,"?actor patching leak");
}

() = opendoor["opens door between ?from to ?to"](edu.tufts.hrilab.fol.Symbol ?actor:station, edu.tufts.hrilab.fol.Symbol ?from:area, edu.tufts.hrilab.fol.Symbol ?to:area) {
    conditions : {
        pre : adjacent(?from, ?to);
    }
    effects : {
        success : open(?from, ?to);
        success : open(?to, ?from);
        success : not(sealed(?from));
        success : not(sealed(?to));
    }
    op:log(info,"[opendoor] opening ?from to ?to");
}

() = closedoor["closes door between ?from to ?to"](edu.tufts.hrilab.fol.Symbol ?actor:station, edu.tufts.hrilab.fol.Symbol ?from:area, edu.tufts.hrilab.fol.Symbol ?to:area) {
    conditions : {
        pre : adjacent(?from, ?to);
    }
    effects : {
        success : not(open(?from, ?to));
        success : not(open(?to, ?from));
        success : sealed(?from);
        success : sealed(?to);
    }
    op:log(info,"[closedoor] closing ?from to ?to");
}

//this might exist elsewhere, check out navigation.asl?
() = goTo["?actor moves to ?location"](edu.tufts.hrilab.fol.Symbol ?to) {
//    conditions : {
//        pre : at(?actor, ?from);
//        //todo: this can be done with exists(...) i believe
//        pre : open(?fromwing, ?towing);
//        //or open ?to ?from
//        pre : in(?from, ?fromwing);
//        pre : in(?to, ?towing);
//        pre : inWing(?actor, ?fromwing);
//    }
//    effects : {
//        success : not(at(?actor, ?from));
//        success : at(?actor, ?to);
//        success : inWing(?actor, ?towing);
//        success : not(inWing(?actor, ?fromwing));
//    }
    op:log(info,"[goto] ?actor going to ?to");
    act:goToLocation(?to);
}

//this might exist elsewhere, check out navigation.asl?
() = goto["?actor moves to ?location"](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?from:location, edu.tufts.hrilab.fol.Symbol ?to:location, edu.tufts.hrilab.fol.Symbol ?fromwing:area, edu.tufts.hrilab.fol.Symbol ?towing:area) {
    conditions : {
        pre : at(?actor, ?from);
//        //todo: this can be done with exists(...) i believe
        pre : open(?fromwing, ?towing);
//        //or open ?to ?from
        pre : in(?from, ?fromwing);
        pre : in(?to, ?towing);
        pre : inWing(?actor, ?fromwing);
    }
    effects : {
        success : not(at(?actor, ?from));
        success : at(?actor, ?to);
        success : inWing(?actor, ?towing);
        success : not(inWing(?actor, ?fromwing));
    }
    op:log(info,"[goto] ?actor going to ?to");
    act:goToLocation(?to);
}

//todo: replace with derived
() = bind["Temporary bind method for a tube and prep location"] (edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location, edu.tufts.hrilab.fol.Symbol ?prep:location, edu.tufts.hrilab.fol.Symbol ?wing:area, edu.tufts.hrilab.fol.Symbol ?dir:direction, edu.tufts.hrilab.fol.Symbol ?id:id) {
    conditions : {
        pre : not(equals(?actor,station:station));
        pre obs: wingid(?tube, ?id);
        pre obs: wingid(?prep, ?id);
        pre obs: side(?tube, ?dir);
        pre obs: side(?prep, ?dir);
        pre obs: in(?tube, ?wing);
        pre obs: in(?prep, ?wing);
        pre : not(equals(?tube, ?prep));
    }
    effects : {
        success : bound(?tube, ?prep);
    }

    op:log("trace", "Logging this line so it's not processed as a PDDL Event.");
}

//TODO:brad make this take in a flag for fist/last?
() = queryEventTime(edu.tufts.hrilab.fol.Term ?fact){
  edu.tufts.hrilab.fol.Symbol !timeStamp;

  !timeStamp= act:queryLastTime(?fact);

  act:generateResponse(!timeStamp);
}

() = describeSuccess["Describes the success of a context"](edu.tufts.hrilab.fol.Symbol ?ref) {
    edu.tufts.hrilab.fol.Predicate !successPred;
    edu.tufts.hrilab.fol.Symbol !success;
    edu.tufts.hrilab.fol.Predicate !failureEffects;
    edu.tufts.hrilab.fol.Predicate !becausePredicate;
    edu.tufts.hrilab.fol.Predicate !state;
    edu.tufts.hrilab.action.justification.Justification !justification;

    op:log("info","[describeSuccess]");
      try {
        !justification = act:getSuccessFromRef(?ref);
        act:generateResponseFromString("yes");
      } catch(!failureConditions) {
        op:log("info","Caught !failureConditions");
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "testPred(?actor)");
        !becausePredicate = act:createBecausePredicate(!state, ?actor, !failureConditions);
//        !becausePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(!becausePredicate)");
        op:log("info",!becausePredicate);
//        !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(!state,!becausePredicate)");
        act:generateResponse(!becausePredicate);
      }
}

() = testQuery["Tests the source query"]() {
    //When was the last power outage in wing beta?
//    act:queryLastTime("repaired(X)");
    //What did you do to restore power to wing beta?
    //How was experiment 3 executed yesterday?
//    act:queryLastSource("repaired(X)");
    //How many successful experiments were run yesterday?
//    act:queryCount("repaired(X)");

    edu.tufts.hrilab.fol.Symbol !ref = "context_1:context";

  act:getActDesc("experiment2()"); //Describe how to run experiment 5.
  act:querySupport("repaired(X)"); //has work station 4 been repaired [replace X w refId]

    act:describeSuccess(!ref);
}

() = addNotification["Adds new notification"](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?descriptor){

    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Symbol !containerID;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Predicate !queryPred;
    edu.tufts.hrilab.fol.Predicate !actionPred;

    java.util.List !tmpDescriptors;
    !tmpDescriptors = op:newObject("java.util.ArrayList");

    edu.tufts.hrilab.fol.Symbol !speaker =?actor;
    (!speaker) = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

    (!bindings) = act:askQuestionFromString(!speaker,"When should I send it?", mod(X));
    //"every time an experiment bay breaks"
    (!queryPred) = op:get(!bindings, !x);

    (!bindings) = act:askQuestionFromString(!speaker,"What should it contain?", mod(X));
    (!actionPred) = op:get(!bindings, !x);

    while(~op:equalsValue(!actionPred,none())){
        op:add(!tmpDescriptors,!actionPred);
        (!bindings) = act:askQuestionFromString(!speaker,"anything else?", mod(X));
        op:log(info,!bindings);
        (!actionPred) = op:get(!bindings, !x);
        op:log(info,!actionPred);
    }

    act:addNotifications(!queryPred, !tmpDescriptors);

    act:submitTTSRequest(!speaker,!speaker,"okay");
}

() = testNotification[""]() {
    effects : {
        success : broken(location_52);
    }
    op:log(info,"breaking tube");
}

() = respondWithAmount[""](){
    java.lang.Long !ret;
    !ret = act:getSizeOfQueue();
    act:generateResponseFromString("!ret experiments are in the queue");
}

() = respondWithGoal[""]() {
    java.util.List !ret;
    java.lang.String !robotoneGoal;
    java.lang.String !robottwoGoal;

    !ret = act:getCurrentQueueGoals();
    !robotoneGoal = op:get(!ret, 0); //todo: strip parens?
    !robottwoGoal = op:get(!ret, 1);

    act:generateResponseFromString("Robot one is doing !robotoneGoal. Robot two is doing !robottwoGoal.");
}
