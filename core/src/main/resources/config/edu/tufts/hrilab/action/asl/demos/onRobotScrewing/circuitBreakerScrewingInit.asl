//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    java.lang.Boolean !twoRobots = true;
    edu.tufts.hrilab.fol.Term !toAssert;
    edu.tufts.hrilab.fol.Symbol !robotone="robotone:agent";
    edu.tufts.hrilab.fol.Symbol !robottwo="robottwo:agent";


    edu.tufts.hrilab.fol.Symbol !descriptorM3="m3Hole";
    edu.tufts.hrilab.fol.Symbol !jobNameM3="holeM3";
    edu.tufts.hrilab.fol.Symbol !descriptorM3D="deepM3Hole";
    edu.tufts.hrilab.fol.Symbol !jobNameM3D="holeDeep";
//
    //add descriptor -> Cognex job mappings
    //This is mainly left as example code, this is handled within CognexConsultant now
    !robotone.tsc:addDetectionType(!descriptorM3,!jobNameM3);
    !robotone.tsc:addDetectionType(!descriptorM3D,!jobNameM3D);
    if (op:equals(!twoRobots, true)) {
        !robottwo.tsc:addDetectionType(!descriptorM3,!jobNameM3);
        !robottwo.tsc:addDetectionType(!descriptorM3D,!jobNameM3D);
    }

    //specify height of object to be used in calculations to determine z height
    //for running Cognex Jobs
    //objectDefinition(descriptor,zOffset,xOffset,)
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(nf32sv,0.055,0.0)");
    act:assertBelief(!toAssert);

    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!descriptorM3,0.075,0.0)");
    act:assertBelief(!toAssert);

    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(nv30fau,0.045,0.0)");
    act:assertBelief(!toAssert);

    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!descriptorM3D,0.045,0.0)");
    act:assertBelief(!toAssert);

    //screwDescent(descriptor,descent)
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "screwDescent(m3,-0.2055)");
    act:assertBelief(!toAssert);

    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "screwDescent(deepM3,-0.218)");
    act:assertBelief(!toAssert);

   !robotone.act:setupPoses();
   if (op:equals(!twoRobots, true)) {
       !robottwo.act:setupPoses();
   }

}

() = setupPoses() {
    edu.tufts.hrilab.fol.Symbol !conveyor = "conveyor";
    edu.tufts.hrilab.fol.Symbol !workArea = "work area";
    edu.tufts.hrilab.fol.Symbol !screwFeeder = "screw feeder";

    edu.tufts.hrilab.fol.Symbol !conveyorHeight = "50";
    edu.tufts.hrilab.fol.Symbol !workHeight = "-13";
    edu.tufts.hrilab.fol.Symbol !screwHeight = "86";
    edu.tufts.hrilab.fol.Symbol !default = "default";

    ai.thinkingrobots.mtracs.util.MPose !pose;

//TODO:brad: what is this for? how should we handle it?
    edu.tufts.hrilab.fol.Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);

    edu.tufts.hrilab.fol.Predicate !cameraHeight;
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,338)");
    act:assertBelief(!cameraHeight);

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 510.00f, 0.0f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    tsc:recordPose(!conveyor, !pose, !conveyorHeight);
    op:log(info, "Setup !conveyor for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -10.00f, -400.00f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    tsc:recordPose(!workArea, !pose, !workHeight);
    op:log(info, "Setup !workArea for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -217.00f, -450.56f, 200.0f, 3.14159f, 0.0f, 3.14159f);
    tsc:recordPose(!screwFeeder, !pose, !screwHeight);
    op:log(info, "Setup !screwFeeder for ?actor");

    //These are here so they are done for each robot
    tsc:openGripper();
    act:rotateToEE(!default);


}
