import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;

//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    Symbol !robotone="robotone:agent";

    !robotone.tsc:loadObjects(!robotone);
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