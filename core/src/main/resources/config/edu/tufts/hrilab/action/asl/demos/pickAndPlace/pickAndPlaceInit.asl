import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    edu.tufts.hrilab.fol.Term !toAssert;
    Symbol !robotone="robotone:agent";

    Symbol !jobNameBox = "nvDet";
    Symbol !boxDescriptor = "box";

    //add descriptor -> Cognex job mappings
    !robotone.tsc:addDetectionType(!boxDescriptor,!jobNameBox);

    //specify height of object to be used in calculations to determine z height
    //for running Cognex Jobs
    //objectDefinition(descriptor,zOffset,xOffset,)
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!boxDescriptor,0.045,0.0)");
    act:assertBelief(!toAssert);

   !robotone.act:setupPoses();
}

() = setupPoses() {
    Symbol !locationOne = "location one";
    Symbol !locationTwo= "location two";
    Symbol !locationThree = "location three";

    Symbol !conveyorHeight = "50";
    Symbol !workHeight = "-13";
    Symbol !default = "default";

    ai.thinkingrobots.mtracs.util.MPose !pose;

    //TODO:brad: what is this for? how should we handle it?
    Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);

    Predicate !cameraHeight;
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,338)");
    act:assertBelief(!cameraHeight);

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 510.00f, 0.0f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    tsc:recordPose(!locationOne, !pose, !conveyorHeight);
    op:log(info, "Setup !locationOne for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -10.00f, -400.00f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    tsc:recordPose(!locationTwo, !pose, !workHeight);
    op:log(info, "Setup !locationTwo for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -217.00f, -450.56f, 200.0f, 3.14159f, 0.0f, 3.14159f);
    tsc:recordPose(!locationThree, !pose, !workHeight);
    op:log(info, "Setup !locationThree for ?actor");

    //These are here so they are done for each robot if we have more than one
    tsc:openGripper();
    act:rotateToEE(!default);


}
