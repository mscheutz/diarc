import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import java.util.List;

() = lookFor(Symbol ?refId:physobj) {
    Predicate !queryPred;

    Symbol !area;

    effects : {
        success : itemAt(?refId, !area);
        success : beenperceived(?refId);
    }

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(?actor, X)");
    !area = act:getBinding(!queryPred);

    act:perceiveEntityFromSymbol(?refId);
}

() = perceiveitem["top level perception action which binds the refId based on the availability of that type of object at the ?actor's location"](Symbol ?refId:physobj, Symbol ?itemType:property, Symbol ?area:area, Symbol ?location:location) {

    java.lang.Boolean !success;
    Predicate !failCond;

    conditions : {
        pre : agentAtLocation(?actor, ?location);
        pre : agentAtArea(?actor, ?area);
        pre : property_of(?refId, ?itemType);
        pre : observableAt(?itemType,?area);
        pre : not(itemAt(?refId, ?area));
        pre : not(beenperceived(?refId));
    }
    effects : {
        success : itemAt(?refId, ?area);
        success : beenperceived(?refId);
    }

    !success = act:perceiveEntityFromSymbol(?refId);
    if (op:equals(!success, true)) {
       op:log(debug, "[perceiveitem] succeeded");
    } else {
        !failCond = op:invokeStaticMethod("edu.hrilab.tufts.fol.Factory", "createPredicate", "not(perceive(?actor, ?itemType))");
        exit(FAIL, !failCond);
    }
}

() = doYouSee(Symbol ?refId:physobj) {
    Predicate !queryPred;
    Symbol !area;

    effects : {
        success : itemAt(?refId, !area);
        success : beenperceived(?refId);
    }

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(?actor, X)");
    !area = act:getBinding(!queryPred);


    try {
        act:perceiveEntityFromSymbol(?refId);
        act:generateResponseFromString("yes");
    } catch(FAIL_POSTCONDITIONS, !e) {
        act:generateResponseFromString("no");
    }
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
