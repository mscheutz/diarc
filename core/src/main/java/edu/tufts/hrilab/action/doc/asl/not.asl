if (~op:lt(?var, 10)) {
  op:log("info", "?var is NOT lower than 10");
}

if (~act:isMoving()) {
  !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(propertyOf(?actor,moving))");
  exit(FAIL, !failCond);
}