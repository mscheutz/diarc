import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = pickUp() {
    act:opengripper();
    act:pickup();
}

() = pickUpFail() {
    act:closegripper();
    act:pickup();
}

() = openGripperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : opengripper(X)
            postRecovery : postrecovery_success
        }
        goals : {holding(?actor)}
    }
    effects : {
        success : gripperOpen(?actor);
    }
    op:log("info","[openGripperActionRecovery]");
}

() = pickUpOpengripperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : pickUp(X)
            postRecovery : postrecovery_success
        }
        excludedGoals : {pickUpFailV2(X)}
        failedActions : {opengripper(X)}
    }
    effects : {
        success : gripperOpen(X);
    }
    op:log("info","[pickupOpengripperActionRecovery]");
}

() = pickupActionRecoveryUndefinedCatching(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_retry
        }
        failedActions : {pickup(X)}
    }
    effects : {
        success : gripperOpen(X);
    }
    op:log("info","[pickupActionRecoveryUndefinedCatching]");
}

() = pickUpV2() {
    op:log("info","[pickUpV2]");
    act:openGripperV2();
    act:pickupv2();
}

() = pickupv2() {
    conditions : {
        pre : gripperOpenV2(?actor);
    }
    effects : {
        success : holdingV2(?actor);
    }

    op:log("info","[pickupv2]");
}

() = pickUpFailV2(Symbol ?test) {
    act:pickUpV2();
    act:pickUpV2();
    op:log("info", "[pickUpFailV2] success");
}

() = openGripperV2() {
    conditions : {
        pre : not(holdingV2(?actor));
    }
    effects : {
        success : gripperOpenV2(?actor);
    }
    op:log("info","[openGripperV2]");
}

() = pickupParentRetryActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : pickUpV2(X)
            postRecovery : postrecovery_parent_retry
        }
        goals : {pickUpFailV2(X,parentretry)}
    }
    effects : {
        success : not(holdingV2(X));
    }
    op:log("info","[pickupParentRetryActionRecovery]");
}


() = pickupParentSuccessActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingAction : pickUpV2(X)
            postRecovery : postrecovery_parent_success
        }
        goals : {pickUpFailV2(X,parentsuccess)}
    }
    effects : {
        success : holdingV2(X);
    }
    op:log("info","[pickupParentSuccessActionRecovery]");
}