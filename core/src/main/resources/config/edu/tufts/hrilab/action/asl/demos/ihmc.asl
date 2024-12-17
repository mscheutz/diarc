import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
() = recoverDoor(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?failedGoal) {
    recovery: {
        failureReasons: {at(X,Y)}
        actionStatuses: {FAIL_POSTCONDITIONS}
    }
    op:log(info, "[failed_at]");
}