import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.util.HashMap;

() = initCratePacking(){
    Symbol !robotone="robotone:agent";
    Term !props;
    Symbol !refId;
    java.lang.Integer !crateL;
    java.lang.Integer !itemL;

    !robotone.tsc:loadLocations(!robotone);
    !robotone.act:setupPoses();

    !crateL = !robotone.tsc:getCrateLength();
    !itemL = !robotone.tsc:getPackedItemLength();
    if (~op:equals(!crateL,-1) && ~op:equals(!itemL,-1)) {
        !robotone.act:solveCratePacking3D();
    }
}

() = setupPoses() {
    Symbol !default = "default";
    Variable !bindingVar = "X";
    Predicate !queryPred;
    List !bindings;
    HashMap !elem;
    Symbol !pose0;
    java.lang.String !actorName;
    java.lang.String !pose0String;

    act:rotateToEE(!default);

    //TODO: not sure if we always want this
    !actorName = op:getName(?actor);
    !pose0String = op:invokeStaticMethod("java.lang.String","join","",!actorName,"pose_0:pose");
    !pose0 = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !pose0String);

    act:goToPoseDirect(!pose0);

    //These are here so they are done for each robot if we have more than one
    tsc:openGripper();
}
