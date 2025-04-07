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
            //catchingActions : {opengripper(X)}
            postRecovery : postrecovery_continue
        }
        failureReasons : {found(self:agent,fakeAction(self:agent))}
        goals : {holding(?actor)}
    }
    effects : {
        success : gripperOpen(?actor);
    }
    op:log("info","[openGripperActionRecoveryFail] intentionally calling failed action again");
    act:opengripper();
}