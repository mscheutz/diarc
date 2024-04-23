import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;

() = pickup(Symbol ?item:physobj, Symbol ?pose:pose, Symbol ?equivalentPose:pose, Symbol ?location:location) {
    conditions : {
        pre : free(?actor);
        pre : overlapping(?pose, ?equivalentPose);
        pre : itemAt(?item, ?equivalentPose);
        pre : reachable(?actor, ?pose, ?location);
        pre : agentAt(?actor, ?location);
    }
    effects : {
        success : not(free(?actor));
        success : holding(?actor, ?item);
        success : not(itemAt(?item, ?equivalentPose));
    }

    op:log(info, "[pickup] going to pose ?pose and perceiving ?item.");
    act:goToCameraPose(?pose);
    act:perceiveEntityFromSymbol(?item);
    op:log(info, "[pickup] picking up ?item");
    act:pickupItem(?item);
}

() = putingredient(Symbol ?item:physobj, Symbol ?destination:physobj, Symbol ?location:location, Symbol ?pose:pose) {
    conditions : {
        pre : holding(?actor, ?item);
        pre : itemAt(?destination, ?pose);
        pre : agentAt(?actor, ?location);
        pre : reachable(?actor, ?pose, ?location);
    }
    effects : {
        success : itemOn(?item, ?destination);
        success : itemAt(?item, ?pose);
        success : not(holding(?actor, ?item));
        success : free(?actor);
    }

    act:putDownItem(?item, ?pose);
}

() = adjustpose(Symbol ?item:physobj, Symbol ?pose:pose, Symbol ?altpose:pose) {
    conditions : {
        pre : itemAt(?item, ?pose);
        pre : overlapping(?pose, ?altpose);
    }
    effects : {
        success : itemAt(?item, ?altpose);
    }

    op:log(info, "[adjustPose] adjusting pose from ?pose to ?altpose");
}

() = putitematpose(Symbol ?item:physobj, Symbol ?pose:pose, Symbol ?altpose:pose, Symbol ?location:location) {
    conditions : {
        pre : holding(?actor, ?item);
        pre : agentAt(?actor, ?location);
        pre : reachable(?actor, ?pose, ?location);
        pre : overlapping(?pose, ?altpose);
    }
    effects : {
        success : itemAt(?item, ?altpose);
        success : not(holding(?actor, ?item));
        success : free(?actor);
    }

    act:putDownItem(?item, ?pose);
}

() = gotolocation(Symbol ?actor:mobileyumi, Symbol ?current:location, Symbol ?destination:location) {
    conditions : {
        pre : agentAt(?actor, ?current);
    }
    effects : {
        success : not(agentAt(?actor, ?current));
        success : agentAt(?actor, ?destination);
    }

    act:goToLocation(?destination);
}


//"primitive" wrapper actions

//() = splitthebun(Symbol ?actor:gofa, Symbol ?refId:physobj, Symbol ?pose:pose, Symbol ?location:location) {
//
//    conditions : {
////        pre obs : propertyof(?refId, bun:property);
//        pre : agentAt(?actor, ?location);
//        pre : itemAt(?refId, ?pose);
//        pre : reachable(?pose, ?location);
//    }
//    effects : {
//        success : split(?refId);
//    }
//
//    Symbol !bottomRefId;
//    Symbol !topRefId;
//    Symbol !pose;
//    Predicate !queryPred;
//    java.util.List !bindings;
//    java.util.HashMap !binding;
//    Variable !bindingVar;
//
//    Predicate !toAssert;
//
//    op:log(info, "[splitBun] ?actor splitting bun into top and bottom");
//
//    !bottomRefId = act:createCogRefWithProperty(bottomBun, "bottomBun(X:physobj)");
//    !topRefId = act:createCogRefWithProperty(topBun, "topBun(X:physobj)");
//
//    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
//    (!queryPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(?refId,X)");
//    !bindings = act:queryBelief(!queryPred);
//    if (~op:isEmpty(!bindings)) {
//        !binding = op:get(!bindings, 0);
//        (!pose) =op:get(!binding,!bindingVar);
//    } else {
//        op:log(error, "[splitBun] bun refId ?refId not at a valid pose. cannot bind split bun halves to the appropriate pose");
//        exit(FAIL, "[splitBun] bun refId ?refIdnot at a valid pose. cannot bind split bun halves to the appropriate pose");
//    }
//
//    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(!bottomRefId, !pose)");
//    act:assertBelief(!toAssert);
//    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(!topRefId, !pose)");
//    act:assertBelief(!toAssert);
//    !toAssert = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(?refId, !pose)");
//    act:retractBelief(!toAssert);
//}

//goal wrapper actions. everything is submitted as a goal, but we need actions for action learning
() = getTo(Symbol ?item:physobj, Symbol ?destination:pose) {
    goal:itemAt(?item, ?destination);
}

() = getOn(Symbol ?item:physobj, Symbol ?destination:physobj) {
    goal:itemOn(?item, ?destination);
}

() = splitBun(Symbol ?bunRefId:physobj) {
    goal:split(?bunRefId);
}


() = getAllOn(Predicate ?itemCollection, Symbol ?destination:physobj) {
    java.util.List !args;
    Symbol !arg;

    !args = op:getArgs(?itemCollection);

    foreach(!arg : !args) {
     goal:itemOn(!arg, ?destination);
    }
}
