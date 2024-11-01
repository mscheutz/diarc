import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.util.HashMap;

//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    Symbol !robotone="robotone:agent";

    !robotone.tsc:loadObjects(!robotone);
    !robotone.tsc:loadLocations(!robotone);
    !robotone.act:setupPoses();
}

() = setupPoses() {
    Symbol !default = "default";
    Variable !bindingVar = "X";
    Predicate !queryPred;
    List !bindings;
    Symbol !cameraHeight;
    HashMap !elem;

    ai.thinkingrobots.mtracs.util.MPose !pose;

    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "heightForPhotoFromRobotBase(X)");
    !bindings = act:queryBelief(!queryPred);
    !elem = op:get(!bindings, 0);
    !cameraHeight = op:get(!elem, !bindingVar);

    //TODO:brad: what is this for? how should we handle it?
    Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);

    Predicate !cameraHeightPredicate;
    !cameraHeightPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,!cameraHeight)");
    act:assertBelief(!cameraHeightPredicate);

    act:goToCameraPose(pose_0:pose);

    //These are here so they are done for each robot if we have more than one
    tsc:openGripper();
    act:rotateToEE(!default);

}