import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

() = runTest() {
  op:log("info", "runTest");
  Predicate !actionPredicate;
  !actionPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionName(fetch,mug)");
  act:firstAction(!actionPredicate);
}

() = firstAction(Predicate ?actionPredicate) {
  op:log("info", "firstAction");
  Symbol !localActor;
  !localActor = op:invokeMethod(?actionPredicate, "get", 0);
  !localActor.act:secondAction();
}

() = secondAction() {
  op:log("info", "secondAction");
  if (~op:equalsValue(?actor, fetch)) {
    op:log("error", "Action failed because actor should be fetch, not ?actor");
    exit(FAIL);
  } else {
    op:log("info", "Actor is correctly assigned: ?actor");
  }
}