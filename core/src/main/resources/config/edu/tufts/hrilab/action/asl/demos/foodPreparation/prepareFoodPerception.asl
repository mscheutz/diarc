import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import java.util.List;

() = perceiveitem["top level perception action which binds the refId based on the availability of that type of object at the ?actor's location"](Symbol ?refId:physobj, Symbol ?itemType:property, Symbol ?pose:pose, Symbol ?location:location) {

    conditions : {
        pre : agentAt(?actor, ?location);
        pre : propertyof(?refId, ?itemType);
        pre : observableAt(?itemType,?pose); //TODO:brad: is this in Belief or the consultant?
        pre : not(itemAt(?refId, ?pose));
        pre : reachable(?actor, ?pose, ?location);
        pre : not(beenperceived(?refId));
    }
    effects : {
        success : itemAt(?refId, ?pose);
        success : beenperceived(?refId);
    }

    act:goToCameraPose(?pose);
    act:perceiveEntityFromSymbol(?refId);

    //todo: handle failure here. ideally better than we did with multiRobotCaddy in terms of what the failure justification is and how hand-tuned the payload is. might need the new failure recovery code though
}

() = perceiveEntityFromSymbol["runs a job for a given pre-existing ?refId and binds the relevant result to that reference"](edu.tufts.hrilab.fol.Symbol ?refId:physobj) {
    java.util.List !cameraResults;
    java.lang.String !jobName;
    edu.tufts.hrilab.abb.consultant.cognex.CognexJob !job;
    edu.tufts.hrilab.abb.consultant.cognex.CognexResult !result;
    edu.tufts.hrilab.abb.consultant.cognex.CognexReference !ref;

    (!ref) = act:getCognexReferenceForID(?refId);

    (!job) = act:getCognexJobForCognexReference(!ref);
    (!jobName) = op:invokeMethod(!job, "getName");
    (!cameraResults) = act:getCameraData(!jobName);
    if (op:isEmpty(!cameraResults)) {
        op:log("warn","[perceiveEntityFromSymbol] failed to get cognex results");
    } else {
        (!result) = act:getMatchingResult(!ref,!cameraResults);
//        bind reference to the result that it matches
        act:bindCognexResult(!ref, !result, 0);
        op:log("info","[perceiveEntityFromSymbol] Bound !result to !ref");
    }
}

() = invalidateReference["todo: (pete) document"](Symbol ?refId:physobj){
    Predicate !toRetract;
    act:removeReference(?refId);
    !toRetract = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemOn(?refId, X)");
    act:retractBelief(!toRetract);
    !toRetract = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(?refId, X)");
    act:retractBelief(!toRetract);
    !toRetract = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "prepared(?refId)");
    act:retractBelief(!toRetract);
}
