import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = pickupPartialRecovery(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
    recovery : {
        goals : {holding(?actor)}
    }
    effects : {
        success : gripperOpen(?actor);
    }

    op:log("info", "[pickupPartialRecovery] asserting gripperOpen(?actor)");
}

() = openGripperActionRecoveryFail(List ?failedActionPredicates, List ?failureReasons, Predicate ?failedGoal) {
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
    op:log("info","[openGripperActionRecoveryFail] intentionally calling failed action again");
    act:opengripper();
}