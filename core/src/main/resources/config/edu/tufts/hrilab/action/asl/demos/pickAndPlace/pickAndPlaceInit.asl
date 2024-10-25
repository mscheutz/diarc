import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;

//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    edu.tufts.hrilab.fol.Term !toAssert;
    Symbol !robotone="robotone:agent";

    //todo: there should be unique jobs for each of the block types (pete)
    Symbol !jobNameBlock = "blocksDet";
    Symbol !jobNameRed = "blocksDet";
    Symbol !jobNameGreen = "blocksDet";
    Symbol !jobNameYellow = "blocksDet";
    Symbol !jobNameOrange = "blocksDet";

    Symbol !redBlockDescriptor = "red block";
    Symbol !blueBlockDescriptor = "blue block";
    Symbol !greenBlockDescriptor = "green block";
    Symbol !yellowBlockDescriptor = "yellow block";
    Symbol !orangeBlockDescriptor = "orange block";

    Symbol !redBlock;
    Symbol !blueBlock;
    Symbol !greenBlock;
    Symbol !yellowBlock;
    Symbol !orangeBlock;

    Term !props;

    //add descriptor -> Cognex job mappings
    !robotone.tsc:addDetectionType(!redBlockDescriptor,!jobNameRed);
    !robotone.tsc:addDetectionType(!blueBlockDescriptor,!jobNameBlock);
    !robotone.tsc:addDetectionType(!greenBlockDescriptor,!jobNameGreen);
    !robotone.tsc:addDetectionType(!yellowBlockDescriptor,!jobNameYellow);
    !robotone.tsc:addDetectionType(!orangeBlockDescriptor,!jobNameOrange);

    //red block//
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!redBlockDescriptor,0.050,0.0)");
    act:assertBelief(!toAssert);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(red block(X:physobj))");
    !redBlock = act:positReference(!props);
//    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!redBlock,pose_0:pose)");
//    op:log(debug, "[init] asserting !toAssert");
//    act:assertBelief(!toAssert);

    //blue block//
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!blueBlockDescriptor,0.050,0.0)");
    act:assertBelief(!toAssert);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(blue block(X:physobj))");
//    !blueBlock = act:positReference(!props);
//    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!blueBlock,pose_1:pose)");
//    op:log(debug, "[init] asserting !toAssert");
    act:assertBelief(!toAssert);

    //green block//
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!greenBlockDescriptor,0.050,0.0)");
    act:assertBelief(!toAssert);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(green block(X:physobj))");
    !greenBlock = act:positReference(!props);
//    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!greenBlock,pose_0:pose)");
//    op:log(debug, "[init] asserting !toAssert");
//    act:assertBelief(!toAssert);

    //yellow block/
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!yellowBlockDescriptor,0.050,0.0)");
    act:assertBelief(!toAssert);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(yellow block(X:physobj))");
    !yellowBlock = act:positReference(!props);
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!yellowBlock,pose_1:pose)");
    op:log(info, "[init] asserting !toAssert");
    act:assertBelief(!toAssert);

    //orange block/
    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectDefinition(!orangeBlockDescriptor,0.050,0.0)");
    act:assertBelief(!toAssert);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(orange block(X:physobj))");
    !orangeBlock = act:positReference(!props);

    !robotone.act:setupPoses();
}

() = setupPoses() {
    Symbol !default = "default";

    ai.thinkingrobots.mtracs.util.MPose !pose;

    //TODO:brad: what is this for? how should we handle it?
    Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);

    Predicate !cameraHeight;
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,338)");
    act:assertBelief(!cameraHeight);

    act:goToCameraPose(pose_0:pose);

    //These are here so they are done for each robot if we have more than one
    tsc:openGripper();
    act:rotateToEE(!default);

}