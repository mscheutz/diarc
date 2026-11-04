// The conveyorMedkitDemo script contains wrapper action scripts for conveyor
// functionality where there are poses at the beginning and end of the segment
// of the conveyor belt which is covered by one forward pulse.

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;
import java.util.List;
import java.lang.String;

() = moveConveyorForward["Moves the conveyor belt forward"]() {
//    conditions : {
//         pre:accessible(?actor,?start);
//         pre:accessible(?actor,?end);
//         pre: at(?object,?start);
//    }
//    effects:{
//        success: at(?object,?end);
//        success: not(at(?object,?start));
//    }

    Term !queryPred;
    List !bindings;
    Symbol !objectID;
    String !objectIDString;
    Variable !bindingVar;
    Variable !y;

    //todo (pete): this shouldn't hardcode the pose that we're retracting from. should really look up the property for the pose?
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","at(X,pose_0:pose)");
    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "[moveConveyorForward] queryPred: !queryPred");
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    op:log(debug, "[moveConveyorForward] bindings for objects at pose_0: !bindings");
    if (op:isEmpty(!bindings)) {
        op:log(debug, "[moveConveyorForward] no objects reported at pose_0");
    }
    !y = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "Y");
    foreach(!binding : !bindings) {
        !objectID = op:invokeMethod(!binding,get,!bindingVar);
        if (~act:isDiarcAgent(!objectID)) {
            tsc:removeCognexReferenceWithID(!objectID);
            op:log(debug, "[moveConveyorForward] removing cognex reference with ID: !objectID");
            !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(!objectID, !y)");
            tsc:retractBelief(!queryPred);
        }
    }

    tsc:conveyorForward();

    op:log("info","Conveyor finished moving forwards");
}

() = moveConveyorBackward["Moves the conveyor belt backward"]() {
//    conditions : {
//         pre:accessible(?actor,?start);
//         pre:accessible(?actor,?end);
//         pre:at(?object,?start);
//    }
//    effects:{
//        success: not(at(?object,?end));
//    }
    tsc:conveyorForward();

    op:log("info","Conveyor finished moving backwards");
}