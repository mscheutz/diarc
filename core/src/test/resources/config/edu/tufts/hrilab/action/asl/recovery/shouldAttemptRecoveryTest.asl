import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.action.justification.Justification;
import java.util.Map;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

() = emptyActionFail() {
    conditions : {
        pre : impossibleCondition();
    }
}

() = actionOne(Symbol ?outcome, Predicate ?justifications = null) {
    effects : {
        success : stateOne(?actor);
    }

    Symbol !success = success;

    if (op:equals(?outcome,!success)) {
        op:log("info", "[actionOne] success");
    } else {
        if (op:isNull(?justifications)) {
            op:log("info", "[actionOne] fail with no justifications");
            exit(FAIL);
        } else {
            op:log("info", "[actionOne] fail with ?justifications");
            exit(FAIL, ?justifications);
        }
    }
}

() = actionTwo() {
    conditions : {
        pre : actionTwoPre();
    }
    effects : {
        success : stateTwo(?actor);
    }

    op:log("info", "[actionTwo]");
}

() = actionTwoRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {stateTwo(X)}
    }
    effects : {
        success : stateTwo(?actor);
    }

    op:log("info", "[actionTwoRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}


() = actionThree() {
    conditions : {
        pre : actionThreePre();
    }
    effects : {
        success : stateThree(?actor);
    }

    op:log("info", "[actionThree]");
}

() = actionThreeRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        excludedGoals : {actionThree(X)}
        failedActions : {actionThree(X)}
    }
    effects : {
        success : stateThree(?actor);
    }

    op:log("info", "[actionThreeRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionFour() {
    conditions : {
        pre : actionFourPre();
    }
    effects : {
        success : stateFour(?actor);
    }

    op:log("info", "[actionFour]");
}

() = actionFourRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        failureReasons : {actionFourPre()}
    }
    effects : {
        success : stateFour(?actor);
    }

    op:log("info", "[actionFourRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionFive() {
    conditions : {
        overall : actionFiveOverall();
    }
    effects : {
        success : stateFive(?actor);
    }

    op:log("info", "[actionFive]");
}

() = actionFiveRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionStatuses : {FAIL_OVERALLCONDITIONS}
    }
    effects : {
        success : stateFive(?actor);
    }

    op:log("info", "[actionFiveRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionSix() {
    conditions : {
        pre : actionSixPre();
    }
    effects : {
        success : stateSix(?actor);
    }

    op:log("info", "[actionSix]");
}

() = actionSixRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        failedActions : {actionSix(X)}
        excludedFailureReasons : {actionSixPre()}
    }
    effects : {
        success : stateSix(?actor);
    }

    op:log("info", "[actionSixRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionSeven() {
    effects : {
        success : stateSeven(?actor);
    }

    op:log("info", "[actionSeven]");
    act:fakeAction();
}

() = actionSevenRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        excludedFailedActions : {actionSeven(X)}
        actionStatuses : {FAIL_NOTFOUND}
    }
    effects : {
        success : stateSeven(?actor);
    }

    op:log("info", "[actionSevenRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionEight() {
    effects : {
        success : stateEight(?actor);
    }

    op:log("info", "[actionEight]");
    act:emptyActionFail();
}

() = actionEightRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {fakeAction(), actionEight(X)}
            postRecovery : postrecovery_success
        }
        goals : {stateEight(X)}
    }
    effects : {
        success : stateEight(?actor);
    }

    op:log("info", "[actionEightRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionNine() {
    effects : {
        success : stateNine(?actor);
    }

    op:log("info", "[actionNine]");
    act:emptyActionFail();
}

() = actionNineRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {fakeAction(X)}
            postRecovery : postrecovery_success
        }
        goals : {stateNine(X)}
    }
    effects : {
        success : stateNine(?actor);
    }

    op:log("info", "[actionNineRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionTen() {
    effects : {
        success : stateTen(?actor);
    }

    op:log("info", "[actionTen]");
    act:emptyActionFail();
}

() = actionTenRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            catchingActions : {actionTen(X)}
            postRecovery : postrecovery_success
        }
        excludedGoals : {stateTen(X)}
    }
    effects : {
        success : stateTen(?actor);
    }

    op:log("info", "[actionTenRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionEleven() {
    effects : {
        success : stateEleven(?actor);
    }

    op:log("info", "[actionEleven]");
    act:emptyActionFail();
}

() = actionElevenRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_success
        }
        goals : {stateEleven(X)}
        failedActions : {emptyActionFail(X)}
    }
    effects : {
        success : stateEleven(?actor);
    }

    op:log("info", "[actionElevenRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionTwelve() {
    effects : {
        success : stateTwelve(?actor);
    }

    op:log("info", "[actionTwelve]");
    act:emptyActionFail();
}

() = actionTwelveRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {stateTwelve(X)}
    }
    effects : {
        success : stateTwelve(?actor);
    }

    op:log("info", "[actionTwelveRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = betterActionTwelveRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {stateTwelve(X)}
        failedActions : {emptyActionFail(X)}
    }
    effects : {
        success : stateTwelve(?actor);
    }

    op:log("info", "[actionTwelveRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = actionThirteen() {
    effects : {
        success : stateThirteen(?actor);
    }

    op:log("info", "[actionThirteen]");
    act:emptyActionFail();
}

() = actionThirteenRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_success
        }
    }
    effects : {
        success : stateThirteen(?actor);
    }

    op:log("info", "[actionThirteenRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = stateFourteenRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {stateFourteen(X)}
    }
    effects : {
        success : stateFourteen(?actor);
    }

    op:log("info", "[stateFourteenRecovery] ?failedActionPredicates, ?failureReasons, ?failedGoal");
}

() = testActionRetry() {
    conditions : {
        pre : testActionPre(?actor);
    }
    effects : {
        success : testActionPost(?actor);
    }
    op:log("info","[testActionRetry] in");
    op:log("info","[testActionRetry] out");
}

() = testActionSuccess() {
    conditions : {
        pre : testActionPre(?actor);
    }
    effects : {
        success : testActionPost(?actor);
    }
    op:log("info","[testActionSuccess] in");
    op:log("info","[testActionSuccess] out");
}

() = testActionRetryRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_retry
        }
        failedActions : {testActionRetry(X)}
    }
    effects : {
        success : testActionPre(?actor);
    }
    op:log("info","[testActionRetryRecovery]");
}

() = testActionSuccessRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_success
        }
        failedActions : {testActionSuccess(X)}
    }
    effects : {
        success : testActionPost(?actor);
    }
    op:log("info","[testActionSuccessRecovery]");
}