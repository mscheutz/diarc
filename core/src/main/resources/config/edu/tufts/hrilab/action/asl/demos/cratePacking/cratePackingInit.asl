import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.util.HashMap;

() = initCratePacking(){
    Symbol !robotone="robotone:agent";

    !robotone.tsc:loadLocations(!robotone);
    !robotone.act:setupPoses();
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
