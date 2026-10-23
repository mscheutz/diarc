import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;
import java.lang.String;
import edu.tufts.hrilab.action.recovery.PostRecoveryExecutionType;

() = pickUp() {
    act:opengripper();
    act:pickup();
}

() = pickUpFail() {
    act:closegripper();
    act:pickup();
    op:log("info","[pickUpFail] complete");
}

//TODO: example of case where we can't do this definition as nicely as we could previously - can't constrain recovery
//  for fakeAction failure specifically to opengripper (do we ever actually want this though?)
() = openGripperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            //catchingActions : {opengripper(X)}
            postRecovery : continue
        }
        failureReasons : {found(self:agent,fakeAction(self:agent))}
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
            //catchingActions : {pickUp(X)}
            catchingActions : {opengripper(X)}
            postRecovery : continue
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
            postRecovery : retry
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
            catchingActions : {pickUpV2(X)}
            postRecovery : retry
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
            catchingActions : {pickUpV2(X)}
            postRecovery : continue
        }
        goals : {pickUpFailV2(X,parentsuccess)}
    }
    effects : {
        success : holdingV2(X);
    }
    op:log("info","[pickupParentSuccessActionRecovery]");
}

(PostRecoveryExecutionType ?postRecoveryType) = overridePostRecoveryPolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            //catchingActions : {pickUpV2(X)}
            catchingActions : {openGripperV2(X)}
            postRecovery : retry
        }
        goals : {pickUpFailV2(X,override)}
    }
    op:log("info","[overridePostRecoveryPolicy]");
    ?postRecoveryType = op:invokeStaticMethod("edu.tufts.hrilab.action.recovery.PostRecoveryExecutionType", "fromString", "CONTINUE");
}

() = multipleAttemptsRecoveryPolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            //catchingActions : {pickUpV2(X)}
            postRecovery : retry
        }
        failureReasons : {not(holdingV2(self:agent))}
        goals : {pickUpFailV2(X,multiple)}
        maximumAttempts : 3
    }
    op:log("info","[multipleAttemptsRecoveryPolicy]");
}

() = cyclicalActionRecoveryTestAction() {
    conditions : {
        pre : gripperOpen(?actor);
        pre : fillerPredicate(?actor);
    }
    effects : {
        success : not(gripperOpen(?actor));
    }
    op:log("info","[cyclicalActionRecoveryTestAction] in");
    op:log("info","[cyclicalActionRecoveryTestAction] out");
}

() = cyclicalActionRecoveryTestActionRecoveryPolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : retry
        }
        goals : {cyclicalActionRecoveryTestAction(X),executeCyclicalTestActionTwice(X)}
        failureReasons : {gripperOpen(X)}
    }
    effects : {
        success: gripperOpen(?actor);
        success: not(fillerPredicate(?actor));
    }
    op:log("info","[cyclicalActionRecoveryTestActionRecoveryPolicy]");
}

() = cyclicalActionRecoveryTestActionRecoveryPolicyTwo(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : retry
        }
        goals : {cyclicalActionRecoveryTestAction(X)}
        failureReasons : {fillerPredicate(X)}
    }
    effects : {
        success: not(gripperOpen(?actor));
        success: fillerPredicate(?actor);
    }
    op:log("info","[cyclicalActionRecoveryTestActionRecoveryPolicyTwo]");
}

() = executeCyclicalTestActionTwice() {
    op:log("info","[executeCyclicalTestActionTwice] entering first instance");
    act:cyclicalActionRecoveryTestAction();
    op:log("info","[executeCyclicalTestActionTwice] entering second instance");
    act:cyclicalActionRecoveryTestAction();
}

() = cyclicalActionRecoveryTestActionRecoveryPolicyTwoSucceed(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : retry
        }
        goals : {executeCyclicalTestActionTwice(X)}
        failureReasons : {fillerPredicate(X)}
    }
    effects : {
        success: fillerPredicate(?actor);
    }
    op:log("info","[cyclicalActionRecoveryTestActionRecoveryPolicyTwoSucceed]");
}