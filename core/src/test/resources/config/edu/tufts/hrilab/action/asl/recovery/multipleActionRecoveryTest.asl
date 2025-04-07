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
            catchingActions : {pickupv2(X)}
            postRecovery : postrecovery_continue
        }
        failureReasons : {found(self:agent,fakeAction(self:agent))}
    }
    effects : {
        success : holding(?actor);
    }

    op:log("info", "[pickupActionRecovery] asserting holding(?actor)");
}

() = openGripperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {opengripper(X)}
            postRecovery : postrecovery_continue
        }
        failureReasons : {found(self:agent,fakeAction(self:agent))}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[openGripperActionRecovery] asserting gripperOpen(?actor)");
}

() = closeGripperThrice() {
    act:closeGripper();
    act:closeGripper();
    act:closeGripper();
}

() = closeGripper() {
    conditions : {
        pre : gripperOpen(?actor);
    }
    effects : {
        success : not(gripperOpen(?actor));
    }
    op:log("info","[closeGripper]");
}

() = closeGripperActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            //catchingActions : {closeGripperThrice(X)}
            catchingActions : {closeGripper(X)}
            postRecovery : postrecovery_retry
        }
        excludedGoals : {testActionSequence(X,Y)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[closeGripperActionRecovery] asserting gripperOpen(?actor)");
}

() = testActionSequence(Symbol ?ignored) {
    op:log("info","[testActionSequence] filler log 1");
    op:log("info","[testActionSequence] filler log 2");
    act:closeGripper();
    op:log("info","[testActionSequence] filler log 3");
    act:closeGripper();
}

() = testActionSequenceActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {testActionSequence(X,once)}
            postRecovery : postrecovery_retry
        }
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[testActionSequenceActionRecovery] asserting gripperOpen(?actor)");
}

() = testActionSequenceTwoAttemptsActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {testActionSequence(X,twice)}
            postRecovery : postrecovery_retry
        }
        maximumAttempts: 2
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[testActionSequenceActionRecovery] asserting gripperOpen(?actor)");
}


() = actionone() {
    conditions: {
        pre : not(blockingCondition(?actor));
    }
    effects : {
        success: achievedGoal(?actor);
    }

    op:log("info", "[actionone]");
    act:failAction();
}

() = failAction() {
    conditions : {
        pre : impossibleToSatisfy(?actor);
    }

    op:log("info","[failAction]");
}

() = failActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {achievedGoal(X)}
        failedActions : {actionone(X)}
    }
    effects : {
        success : blockingCondition(?actor);
    }

    op:log("info","[failActionRecovery]");
}

() = unblockcondition() {
    effects : {
        success : not(blockingCondition(?actor));
    }

    op:log("info","[unblockCondition]");
}
