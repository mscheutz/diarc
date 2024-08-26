import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;

() = goto_wrapper(Symbol ?location) {
    goal:at(?actor,?location);
}

() = pickup_wrapper(Symbol ?object) {
    goal:holding(?actor,?object);
}

() = putdown_wrapper() {
    goal:free(?actor);
}

() = putdownat_wrapper(Symbol ?object, Symbol ?location) {
    goal:at(?object,?location);
}

() = deliver_to(Symbol ?object, Symbol ?location) {
    effects : {
        success : not(delivered(?object,?location));
    }
    goal:delivered(?object, ?location);
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
