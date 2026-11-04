import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;

() = goto(Symbol ?location) {
    Symbol !from;
    Predicate !queryPred;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");
    !from = act:getBinding(!queryPred);
    if (op:equalsValue(!from, none)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X");
        exit(FAIL, !failCond);
    }
    //only trigger moveto if we want the conditions and effects
    if (op:equalsValue(!from, ?location)) {
        act:goToLocation(?location);
    } else {
        act:moveto(!from, ?location);
    }

    act:generateResponseFromString("I'm here");
}

() = pickup(Symbol ?object) {
    goal:holding(?actor,?object);
}

() = putdown() {
    Symbol !object;
    Predicate !queryPred;
    Predicate !failCond;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, X)");
    !object = act:getBinding(!queryPred);
    if (op:equalsValue(!object, none)) {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, X");
        exit(FAIL, !failCond);
    }
    act:putdown(!object);
}

() = putdown(Symbol ?object) {

    Predicate !queryPred;
    Symbol !location;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");
    !location = act:getBinding(!queryPred);

    //todo: check to see if there's a table binding and fail if there isn't?

    goal:at(?object, !location);
}

() = putdownat(Symbol ?object, Symbol ?location) {
    goal:at(?object,?location);
}

() = getTo(Symbol ?object, Symbol ?location) {
    goal:at(?object,?location);
}

() = deliver(Symbol ?object, Symbol ?location) {
    effects : {
        success : not(delivered(?object,?location));
    }
    goal:delivered(?object, ?location);
}

//todo: make sure this behaves well.
() = addfact(Symbol ?object, Symbol ?location) {

    Symbol !property;

    conditions : {
        pre : property_of(?object, !property);
//        pre : observable(!property);
    }
    effects : {
        success : at(?object, ?location);
        success : not(observable(!property));
    }

    op:log(info, "[addfact] asserting that ?object is at ?location and toggling off observability of other objects of type !property");

}

//() = fetch(Symbol ?type) {
//    effects : {
//        success : not(delivered(?type));
//    }
//    goal:delivered(?type);
//}




//() = deliver_to(Symbol ?type, Symbol ?location) {
//    //todo: This precondition doesn't actually do anything other than ensure that the object is there.
//    //In order to fix this, we would need to either get the object and have our goal be (at(!obj,?location))
//    //Or change delivered to accept a location as well.
////    conditions : {
////        pre : observableat(?type,?location);
////    }
//    effects : {
//        success : not(delivered(?type,?location));
//    }
//    goal:delivered(?type, ?location);
//}

(Symbol ?result) = getBinding(Predicate ?queryPred) {
    Variable !queryKey = X;
    java.util.List !bindings;
    java.util.Map !binding;

    ?result = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");

    !bindings = act:queryBelief(?queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("debug", "[getBinding] query ?queryPred returned no results");
    } else {
        !binding = op:get(!bindings, 0);
        ?result= op:get(!binding, !queryKey);
        op:log("debug", "[getBinding] query ?queryPred result: ?result");
    }
}
