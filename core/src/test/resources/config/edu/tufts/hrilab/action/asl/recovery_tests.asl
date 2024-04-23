() = policy1(Predicate ?brokenActionPredicate, List<Predicate> ?failureReasons, Predicate ?goal) {

  recovery: {
    goals: {grasping(X,Y,Z), holding(X,Y)}
//    excludedGoal:
//    failedActions:
//    excludedFailedActions:
//    failureReasons:
//    excludedFailureReasons:
    actionStatuses: {FAIL_POSTCONDITIONS, FAIL_PRECONDITIONS}
  }

  op:log("info", "executing recovery policy");
}
