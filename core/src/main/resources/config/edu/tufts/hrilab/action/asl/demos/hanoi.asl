import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;

() = recoverDoor(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?failedGoal) {
    recovery: {
        failureReasons: {goal(?actor, open())}
    }
    effects : {
        success : not(pushdoor(?actor));
        success : pulldoor(?actor);
    }
    act:generateResponseFromString("I could not push the door. I will try pulling it.");
    op:log(info, "Recovering");

}

//(java.util.List ?return) = checkDoor(edu.tufts.hrilab.fol.Predicate ?predicate) {
//    observes: open()
//    ?return = op:newArrayList("java.util.Map");
//    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", edu.tufts.hrilab.fol.Symbol);
//    if (op:==(!goalSize, 1)) {
//      op:add(?return, !bindings);
//    }
//}


() = pull_door() {
    conditions : {
        pre : pulldoor(?actor);
    }
    effects : {
        success infer : open();
    }
    op:log(info, "Pulling door");
    act:pull_door_primitive();
}

() = push_door() {
    conditions : {
        pre : pushdoor(?actor);
    }
    effects : {
        success infer : open();
    }
    op:log(info, "Pushing door");
    act:push_door_primitive();
}