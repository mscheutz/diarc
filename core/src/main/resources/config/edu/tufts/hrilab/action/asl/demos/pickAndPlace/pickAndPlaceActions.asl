import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

() = openGripper() {
    tsc:openGripper();
}

() = closeGripper() {
    tsc:openGripper();
}

//todo: (pete) allows the LLM target for semantics to be more sane than "getOn". should be true everywhere.
() = moveObjectTo(Symbol ?object, Symbol ?destination) {
    act:getOn(?object, ?destination);
}

() = pickUp(Symbol ?object) {
    Predicate !goalPred;

    !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, ?object)");
    goal:!goalPred;
}
