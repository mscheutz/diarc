//The circuitBreakerScrewingDemo includes the scripts used to configure
//and run various screwing primitives (and their compiled versions)
//to fill in different holes on the large (NF32SV and small (NV30FAU) circuit breakers
//====================== Setup action scripts ======================
() = initPose["workaround for not being able to retract facts from belief init files"](){
    edu.tufts.hrilab.fol.Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);
}

() = init() {
    edu.tufts.hrilab.fol.Symbol !conveyor = "conveyor";
    edu.tufts.hrilab.fol.Symbol !workArea = "scale";
    edu.tufts.hrilab.fol.Symbol !binOne= "box a";
    edu.tufts.hrilab.fol.Symbol !binTwo= "box b";

    edu.tufts.hrilab.fol.Symbol !conveyorHeight = "40";
    //todo: update as -10??????
    edu.tufts.hrilab.fol.Symbol !workHeight = "45";
    edu.tufts.hrilab.fol.Symbol !binOneHeight= "80";
    edu.tufts.hrilab.fol.Symbol !binTwoHeight= "80";

    edu.tufts.hrilab.fol.Term !toAssert;

    edu.tufts.hrilab.fol.Symbol !descriptorPillBottle="pillBottle";
    edu.tufts.hrilab.fol.Symbol !jobNamePillBottle="pillDet";

    //add descrptor to cognex job mappings
    tsc:addDetectionType(!descriptorPillBottle,!jobNamePillBottle);

    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(pillBottle,0.085,0.09)");
    act:assertBelief(!toAssert);

    edu.tufts.hrilab.fol.Predicate !cameraHeight;
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,338)");
    act:assertBelief(!cameraHeight);

    ai.thinkingrobots.mtracs.util.MPose !pose;

    act:initPose();
    tsc:openGripper();

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 650.00f, 0.0f, 300.00f, 3.14159f, 0.0f, 3.14159f);
    tsc:goToPose(!pose);
    tsc:recordPose(!conveyor, !conveyorHeight);
    //tsc:bindToSurface(!pose, !conveyorHeight);

    op:log(info, "Setup conveyor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 150.87f, -494.98f, 186.97f, 3.14159f, 0.0f, 3.14159f);
    tsc:goToPose(!pose);
    tsc:recordPose(!workArea, !workHeight);

    op:log(info, "Setup scale");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -188.55f, -359.07f, 184.76f, 3.14159f, 0.0f, 3.14159f);
    tsc:goToPose(!pose);
    tsc:recordPose(!binOne, !binOneHeight);

    op:log(info, "Setup bin one area");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -188.55f, -542.07f, 184.76f, 3.14159f, 0.0f, 3.14159f);
    tsc:goToPose(!pose);
    tsc:recordPose(!binTwo, !binTwoHeight);

    op:log(info, "Setup bin two area");

}

//====================== Demo specific? ==============================

//define item

//TODO:brad: also need to bind the with a model id, sow e can update it later?
() = defineItem["defines new item, and asks for relevant parameters"](edu.tufts.hrilab.fol.Symbol ?item){
  edu.tufts.hrilab.fol.Symbol !jobID;
  edu.tufts.hrilab.fol.Variable !x = "X";
  java.util.Map !bindings;

  (!bindings) = act:askQuestionFromString(?actor,"which Cognex job is used to detect it", job(X));
  (!jobID) = op:get(!bindings, !x);
  tsc:addDetectionType(?item,!jobID);

  act:generateResponseFromString("okay");
}
//====================== Assembly action scripts ======================

() = getOn["gets ?object on to the surface beneath ?destination"](edu.tufts.hrilab.fol.Symbol ?object, edu.tufts.hrilab.fol.Symbol ?destination) {
   edu.tufts.hrilab.fol.Predicate !goalPred;

   (!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,?destination)");
   goal:!goalPred;
}

() = moveToCameraHeight["moves up by a camera z height as defined by an inline constant"]() {
    //Todo: Will/Mar: change these from hardcoded values and/or see if these need to be inverses or not
    java.lang.Double !dist = 0.15;
    tsc:moveZRelative(!dist);
}

() = moveToObjectHeight["moves down by a camera z height as defined by an inline constant"]() {
    //Todo: Will/Mar: change these from hardcoded values and/or see if these need to be inverses or not
    java.lang.Double !dist = -0.15;
    tsc:moveZRelative(!dist);
}

() = controlLoop(edu.tufts.hrilab.fol.Symbol ?actor) {
    edu.tufts.hrilab.fol.Predicate !actionPredicate;
    java.lang.Long !gid;
    !actionPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "sort", ?actor);
    while(true) {
        !gid = act:submitGoal(!actionPredicate);
        act:joinOnGoal(!gid);
    }
}
