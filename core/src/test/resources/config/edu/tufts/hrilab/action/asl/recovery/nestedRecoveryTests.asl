import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = pickUp(Symbol ?test) {
    act:pickup();
}

() = failAction() {
    conditions : {
        pre : failActionPre(?actor);
    }
    op:log("info","[failAction] success");
}

() = pickupNestedRecoverySuccess(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            //catchingActions : {pickUp(X,success)}
            catchingActions : {pickup(X)}
            postRecovery : postrecovery_retry
        }
        //failedActions : {pickup(X)}
        goals : {pickUp(X,success)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    act:failAction();
    op:log("info", "[pickupPartialRecovery] asserting gripperOpen(?actor)");
}

() = failActionRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_retry
        }
        failedActions : {failAction(X)}
        actionStatuses : {FAIL_PRECONDITIONS}
    }
    effects : {
        success : failActionPre(?actor);
    }
    op:log("info","[failActionRecovery] asserting preconditions required for failAction");
}

() = pickupNestedRecoveryFail(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            postRecovery : postrecovery_retry
        }
        failedActions : {pickup(X)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    act:pickup();
}

() = pickUpV2(Symbol ?type) {
    op:log("info","in");
    act:pickupv2();
    op:log("info","out");
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

//TODO - does catchingAction still mean what we want it to? I think it's the way we discussed, but
//  then how do you constrain catching the failures only to a specific parent context? Right now,
//  saying pickUpV2 as the catching action makes you redo pickUpV2 from the parent. How about if
//  you want to retry pickupv2 in the context of only pickUpV2?
() = pickupNestedRecoveryFaulty(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        actionRecovery : {
            //catchingActions : {pickUpV2(X,Y)}
            postRecovery : postrecovery_retry
        }
        goals : {pickUpV2(?actor,child)}
        failedActions : {pickupv2(X)}
    }

    op:log("info","[pickupNestedRecoveryFaulty] not actually recovering");
}

//() = pickupNestedParentRecoveryFaulty(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
//    recovery : {
//        actionRecovery : {
//            catchingActions : {pickUpV2(X,Y)}
//            postRecovery : postrecovery_parent_retry
//        }
//        goals : {pickUpV2(?actor,parent)}
//        failedActions : {pickupv2(X)}
//    }
//
//    op:log("info","[pickupNestedRecoveryFaulty] not actually recovering");
//}