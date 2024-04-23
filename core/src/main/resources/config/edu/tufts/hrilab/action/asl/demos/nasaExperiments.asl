() = initGoalQueue[""] () {

    edu.tufts.hrilab.fol.Predicate !experimentOnePredicate;
    edu.tufts.hrilab.fol.Predicate !experimentTwoPredicate;
    edu.tufts.hrilab.fol.Predicate !experimentThreePredicate;
    edu.tufts.hrilab.fol.Predicate !experimentFourPredicate;
    edu.tufts.hrilab.fol.Predicate !experimentFivePredicate;
    edu.tufts.hrilab.fol.Symbol !robotone = "robotone:agent";

    act:initRobotLocations();

    !experimentOnePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate",   "experiment1(?actor)");
    !experimentTwoPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate",   "experiment2(?actor)");
    !experimentThreePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "experiment3(?actor)");
    !experimentFourPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate",  "experiment4(?actor)");
    !experimentFivePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate",  "experiment5(?actor)");
    act:addGoalToQueue(!experimentOnePredicate);
    act:addGoalToQueue(!experimentTwoPredicate);
    act:addGoalToQueue(!experimentThreePredicate);
    act:addGoalToQueue(!experimentFourPredicate);
    act:addGoalToQueue(!experimentFivePredicate);
}


() = initGoalQueueProblemSolving[""] () {

    edu.tufts.hrilab.fol.Predicate !experimentTwoPredicate;
    edu.tufts.hrilab.fol.Symbol !robotone = "robotone:agent";

    act:initRobotLocations();

    !experimentTwoPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "experiment2(?actor)");
    act:addGoalToQueue(!experimentTwoPredicate);
    act:checkAgentsAndSubmitExperiments();
}

() = initRobotLocations["initializes robot locations to start using the semantic location names"]() {
    java.lang.String !startLocationName = "startCenter";
    edu.tufts.hrilab.fol.Symbol !startLocationRefId;
    edu.tufts.hrilab.fol.Symbol !startArea = "start:area";
    edu.tufts.hrilab.fol.Symbol !robotone = "robotone:agent";
    edu.tufts.hrilab.fol.Symbol !robottwo = "robottwo:agent";
    edu.tufts.hrilab.fol.Predicate !robotoneAtStartPredicate;
    edu.tufts.hrilab.fol.Predicate !robottwoAtStartPredicate;
    edu.tufts.hrilab.fol.Predicate !robotoneInWingPredicate;
    edu.tufts.hrilab.fol.Predicate !robottwoInWingPredicate;

    //todo: this is the start location which is provided by the json file. generalize?
    !startLocationRefId = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "location_0", "location");
    !robotoneAtStartPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at", !robotone, !startLocationRefId);
    !robottwoAtStartPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at", !robottwo, !startLocationRefId);
    !robotoneInWingPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inWing", !robotone, !startArea);
    !robottwoInWingPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inWing", !robottwo, !startArea);
    act:assertBelief(!robotoneAtStartPredicate);
    act:assertBelief(!robottwoAtStartPredicate);
    act:assertBelief(!robotoneInWingPredicate);
    act:assertBelief(!robottwoInWingPredicate);
}

() = experiment1["a robot performs experiment 1 by moving between locations and fails if tube named alphaLeftFive is broken"]() {

    edu.tufts.hrilab.fol.Predicate !tubeFailure;
    edu.tufts.hrilab.fol.Symbol !tubeOneId = "location_1:location"; //location_1 alphaLeftFive

    op:log(info, "[experimentOne] evaluating sample at !tubeOneId");
    goal:at(?actor, !tubeOneId);
    ?actor.act:evaluateAt(!tubeOneId);

    op:log(info, "[experimentOne] completed experiment one");
}

() = experiment2["a robot performs experiment 2 by moving between locations and fails if tube named betaRightSix is broken"]() {

    edu.tufts.hrilab.fol.Symbol !tubeOneId = "location_1:location"; //location_68 betaRightSix
    edu.tufts.hrilab.fol.Symbol !tubeTwoId = "location_19:location"; //location_19 betaLeftFive

    //go to the first location and prepare the sample.
    op:log(info, "[experimentTwo] executing prepare sample at !tubeOneId");

    goal:at(?actor,!tubeOneId);
    act:prepareSampleAt(!tubeOneId); //sample preparation can fail if the tube is broken

    op:log(info, "[experimentTwo] executing perform spectroscopy at !tubeTwoId");
    goal:at(?actor,!tubeTwoId);
    act:performSpectroscopyAt(!tubeTwoId); //spectroscopy cannot fail. todo: I am avoiding a conjunctive failure justification since that is not implemented.

    op:log(info, "[experimentTwo] completed experiment two");
}

() = experiment3["a robot performs experiment 3 by mixing a sample solution, filtering it, and evaluating the sample at multiple locations"]() {


    act:goToLocation(location_42:location);
    act:checkTubePressureIsBelow(psi(14));
    act:checkTubeTemperature(degrees(16));

    act:goToLocation(location_50:location);
    act:disposeOfSample();

    op:log(info, "[experimentThree] completed experiment three");
}

() = experiment4["a robot performs experiment 4 via same procedure as experiment 3, without a filtering step"]() {

    edu.tufts.hrilab.fol.Symbol !tubeOneId = "location_50:location";

    ?actor.act:goToLocation(location_6:location);
    ?actor.act:checkTubeTemperatureIs(degrees(27));
    ?actor.act:goToLocation(location_16:location);
    ?actor.act:checkTubePressureIsBelow(psi(35));
}

() = experiment5["a robot performs experiment 5 via same procedure as experiment 3, but performs spectroscopy instead of evaluating in last step"]() {

    edu.tufts.hrilab.fol.Symbol !tubeOneId = "location_42:location"; //location_42 ??? todo: identify semantic name in current config
    edu.tufts.hrilab.fol.Symbol !tubeTwoId = "location_50:location"; //location_50 ??? todo: identify semantic name in current config

    //go to first location and perform first two steps
    op:log(info, "[experimentFive] executing mixing and filtering the sample solution.");
    goal:at(?actor, !tubeOneId);
    act:mixSampleSolution(!tubeOneId);
    act:filterSampleSolution(!tubeOneId);

    //go to second location and perform spectroscopy
    op:log(info, "[experimentFive] evaluating sample at !tubeTwoId");
    goal:at(?actor, !tubeTwoId);
    ?actor.act:performSpectroscopyAt(!tubeTwoId);

    op:log(info, "[experimentFive] completed experiment five");
}

() = experiment6["a robot performs experiment 6"]() {

    act:takeMaterialsToLocation(location_6:location);
    act:performSpectroscopyAt(location_6:location);

    op:log(info, "[experimentSix] completed experiment six");
}

() = experiment8["a robot performs experiment 8"]() {
    ?actor.act:goTo(location_56:location);
    ?actor.act:checkTubeTemperatureIs(degrees(20));
    ?actor.act:checkTubePressureIsBelow(psi(32));
    ?actor.act:correctPressureIfNeeded();
    ?actor.act:correctTemperatureIfNeeded();
    ?actor.act:recordCellCultureCount();
}

() = recordCellCultureCount[""]() {
    op:log(info,"recording cell culture count");
}

() = correctPressureIfNeeded[""]() {
    op:log(info,"Correcting pressure");
}

() = correctTemperatureIfNeeded[""]() {
    op:log(info,"Correcting temperature");
}

() = evaluateAt["wraps primitive for running experiment. if primitive notifies of failure, exits with failure and the tube it was run on."](edu.tufts.hrilab.fol.Symbol ?tube:location) {
    conditions : {
        pre : at(?actor, ?tube);
    }

    edu.tufts.hrilab.fol.Predicate !tubeFailure;

    try {
        ?actor.act:failIfTubeBroken(?tube);
    }
    catch (FAIL) {
        !tubeFailure = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "functional", ?tube);
        op:log(info, "[evaluateAt] failed with failure !tubeFailure");
        exit(FAIL, !tubeFailure);
    }
    op:log(info, "[evaluateAt] executed successfully");
}

() = prepareSampleAt["prepare sample at tube location. if tube is broken at location, exits with failure and the tube it was run on."](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location) {
    conditions : {
        pre : at(?actor, ?tube);
    }

    edu.tufts.hrilab.fol.Predicate !tubeFailure;

    try {
        ?actor.act:failIfTubeBroken(?tube);
    }
    catch (FAIL) {
        op:log(info, "[prepareSampleAt] failed");
        !tubeFailure = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "functional", ?tube);
        exit(FAIL, !tubeFailure);
    }
    op:log(info, "[prepareSampleAt] executed successfully");
}

() = performSpectroscopyAt[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location) {
    conditions : {
        pre : at(?actor, ?tube);
    }
    op:log(info, "[performSpectroscopyAt] executed successfully");
}

() = performComponentAnalysisAt[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location) {
    conditions : {
        pre : at(?actor, ?tube);
    }
    op:log(info, "[performComponentAnalysisAt] executed successfully");
}

() = failIfTubeBroken["wraps primitive checking if the given tube is broken and fails with a meaningful justification if it is"](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube:location){
    java.lang.Boolean !result;

    !result = ?actor.act:senseIfTubeBroken(?tube);

    if (op:equals(!result, True)) {
        exit(FAIL);
    }
}

() = filterSampleSolution[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube) {
    conditions : {
        pre : at(?actor, ?tube);
    }

    op:log(info, "[filterSampleSolution] executed successfully");
}

() = mixSampleSolution[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube) {
    conditions : {
        pre : at(?actor, ?tube);
    }

    op:log(info, "[mixSampleSolution] executed successfully");
}

() = takeMaterialsToLocation[""](edu.tufts.hrilab.fol.Symbol ?actor:agent, edu.tufts.hrilab.fol.Symbol ?tube) {
    goal:at(?actor, ?tube);
}

() = checkTubePressureIsBelow[""](edu.tufts.hrilab.fol.Predicate ?pressure) {
    act:generateResponseFromString("Checking pressure is below ?pressure");
    op:sleep(500);
}

() = checkTubeTemperatureIs[""](edu.tufts.hrilab.fol.Predicate ?temp) {
    act:generateResponseFromString("Checking temperature is at ?temp");
    op:sleep(500);
}

() = disposeOfSample[""]() {
    conditions : {
        pre : at(?actor, ?tube);
    }
    op:sleep(500);
    op:log(info, "[disposeOfSample] executed successfully");
}