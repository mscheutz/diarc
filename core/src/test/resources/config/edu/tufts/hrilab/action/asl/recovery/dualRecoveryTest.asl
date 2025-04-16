import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = pickupwrapper() {
    effects : {
        success : pickUpWrapperSuccess(?actor);
    }
    act:pickup();
}

() = pickUpWrapperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {pickupwrapper(X)}
            postRecovery : postrecovery_continue
        }
        goals : {pickUpWrapperSuccess(X)}
    }

    op:log("info", "[pickupWrapperActionRecovery] Going to fail by calling nonexistent action");
    act:nonexistentAction();
}

() = pickUpWrapperGoalRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {pickUpWrapperSuccess(X)}
    }
    effects : {
        success : gripperOpen(X);
    }

    op:log("info", "[pickupWrapperGoalRecovery] Asserting necessary precondition for pickup");
}