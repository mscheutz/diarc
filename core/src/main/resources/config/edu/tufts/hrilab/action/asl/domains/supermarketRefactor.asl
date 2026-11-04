// Action scripts for the supermarket environment
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
//() = initializeagent[""]() {
//
//}
//
//() = exit() {
//
//}
//
() = checkout(Symbol ?register:register) {
    conditions : {
        pre infer : at(?actor, ?register);
    }
    effects : {
        success infer : paid(?actor);
    }
  op:log("info", "?actor checking out at ?register");

}
//
//(java.util.List ?return) = checkAt(edu.tufts.hrilab.fol.Predicate ?predicate) {
//    java.util.Map !bindings;
//    edu.tufts.hrilab.fol.Symbol !x;
//
//    observes : at(?object,!x);
//
//    ?return = act:objectObserver(?predicate);
//}

() = navigate(Symbol ?from:location, Symbol ?to:location, Symbol ?basket:basket) {
    conditions : {
        pre infer : at(?actor, ?from);
        pre infer : holding(?actor, ?basket);
    }
    effects : {
        success infer : at(?actor, ?to);
        success infer : not(at(?actor, ?from));
    }
  op:log("info", "Navigating from ?from to ?to");
  act:reactive_nav(?to);
}

() = getbasket(Symbol ?location:basketReturn, Symbol ?basket:basket) {
    conditions : {
        //pre infer : at(?actor, ?location);
        pre infer : at(?basket, ?location);
        pre infer : free(?actor);
    }
    effects : {
        success infer : not(free(?actor));
        success infer : holding(?actor, ?basket);
        success infer : at(?actor, ?location);
        success infer : not(at(?basket, ?location));
    }
    op:log("info", "getting ?basket from ?location");
    act:reactive_nav(?location);
    act:pickup(?location);
}

() = getitem(Symbol ?object:food, Symbol ?location:location, Symbol ?basket:basket) {
    conditions : {
        pre infer : fluent_greater(amount(?location, ?object), 1);
        pre infer : at(?actor, ?location);
    }

    effects : {
        success infer : fluent_decrease(amount(?location, ?object), 1);
        success infer : fluent_increase(amount(?basket, ?object), 1);
        success infer : not(paid(?actor));
    }
    act:pickup(?location);
    op:log("info", "putting away ?object at ?location into ?basket");
}

//() = leavebasket(Symbol ?location:location, Symbol ?basket:basket) {
//    conditions : {
//        pre infer : at(?actor, ?location);
//        pre infer : holding(?actor, ?basket);
//    }
//    effects : {
//        success infer : at(?basket, ?location);
//        success infer : free(?actor);
//        success infer : not(holding(?actor, ?basket));
//    }
//    op:log("info", "leaving ?basket at ?location");
//}

//() = pickup(Symbol ?object:food, Symbol ?location:location) {
//    conditions : {
//        pre infer : at(?actor, ?location);
//        pre infer : fluent_greater(amount(?location, ?object), 1);
//        pre infer : free(?actor);
//    }
//    effects : {
//        success infer : not(free(?actor));
//        success infer : holding(?actor, ?object);
//        success infer : fluent_decrease(amount(?location, ?object), 1);
//    }
//    op:log("info", "picking up ?object at ?location");
//
//}
//
//() = putaway(Symbol ?object:food, Symbol ?location:location) {
//    conditions : {
//        pre infer : at(?actor, ?location);
//        pre infer : holding(?actor, ?object);
//    }
//
//    effects : {
//        success infer : free(?actor);
//        success infer : not(holding(?actor, ?object));
//        success infer : fluent_increase(amount(?location, ?object), 1);
//    }
//    op:log("info", "putting away ?object at ?location");
//}