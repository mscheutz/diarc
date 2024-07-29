import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = pickUp(Symbol ?outcome) {
    Symbol !success = success;

    //These cases are actually identical now
    if (op:equals(?outcome,!success)) {
        act:pickup();
    } else {
        act:closegripper();
        act:pickup();
    }
}

() = pickupFullRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {holding(X)}
    }
    effects : {
        success : holding(?actor);
    }

    op:log("info", "[pickupFullRecovery]");
}

() = pickupPartialRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {pickUp(X,success)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[pickupPartialRecovery]");
}

() = pickupFailRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {pickUp(X,failure)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[pickupFailRecovery]");
    act:fakeAction();
}

() = pickupExitRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {pickUp(X,exit)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[pickupFailRecovery]");
    exit(EXIT_RECOVERY);
}