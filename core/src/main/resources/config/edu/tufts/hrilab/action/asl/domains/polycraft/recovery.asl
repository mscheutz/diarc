import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

() = failedPostConditionsPolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_POSTCONDITIONS}
  }

  op:log("debug", "executing failedPostConditionsPolicy");
  act:executeFailedPostConditionsPolicy(?failedActionPredicate, ?failureReasons, ?goal);
}

() = failedPreConditionsPolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_PRECONDITIONS}
  }

  op:log("debug", "executing failedPreConditionsPolicy");
  act:executeFailedPreConditionsPolicy(?failedActionPredicate, ?failureReasons, ?goal);
}

() = failedReturnValuePolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_RETURNVALUE}
  }

  op:log("debug", "executing failedReturnValuePolicy");
  act:executeFailedReturnValuePolicy(?failedActionPredicate, ?failureReasons, ?goal);
}

() = failedToPlanPolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {

  recovery: {
    actionStatuses: {FAIL_NOTFOUND}
  }

  op:log("debug", "executing failedToPlanPolicy");
  act:executeFailedToPlanPolicy(?failedActionPredicate, ?failureReasons, ?goal);
}