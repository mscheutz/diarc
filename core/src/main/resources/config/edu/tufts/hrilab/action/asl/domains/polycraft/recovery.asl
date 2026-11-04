import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = failedPostConditionsPolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_POSTCONDITIONS}
  }

  op:log("debug", "executing failedPostConditionsPolicy");
  act:executeFailedPostConditionsPolicy(?failedActionPredicates, ?failureReasons, ?goal);
}

() = failedPreConditionsPolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_PRECONDITIONS}
  }

  op:log("debug", "executing failedPreConditionsPolicy");
  act:executeFailedPreConditionsPolicy(?failedActionPredicates, ?failureReasons, ?goal);
}

() = failedReturnValuePolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_RETURNVALUE}
  }

  op:log("debug", "executing failedReturnValuePolicy");
  act:executeFailedReturnValuePolicy(?failedActionPredicates, ?failureReasons, ?goal);
}

() = failedToPlanPolicy(List ?failedActionPredicates, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_NOTFOUND}
  }

  op:log("debug", "executing failedToPlanPolicy");
  act:executeFailedToPlanPolicy(?failedActionPredicates, ?failureReasons, ?goal);
}