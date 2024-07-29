import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = pickUp() {
    act:opengripper();
    act:pickupv2();
    op:log("info","[pickUp] success");
}

() = pickupv2() {
    conditions : {
        pre : gripperOpen(?actor);
    }
    effects : {
        success : holding(?actor);
    }
    op:log("info","[pickup] calling nonexistent action");
    act:fakeAction();
}

//Only present to include dummy predicate in the domain to make it so init isn't empty
() = fillerAction() {
    effects : {
        success : fillerPredicate(?actor);
    }
    op:log("info","[fillerAction]");
}

() = pickupActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : pickupv2(X)
            postRecovery : postrecovery_success
        }
    }
    effects : {
        success : holding(?actor);
    }

    op:log("info", "[pickupActionRecovery] asserting holding(?actor)");
}

() = openGripperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : opengripper(X)
            postRecovery : postrecovery_success
        }
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[openGripperActionRecovery] asserting gripperOpen(?actor)");
}
