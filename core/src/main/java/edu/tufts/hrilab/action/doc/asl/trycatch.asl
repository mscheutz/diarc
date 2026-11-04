try {
  act:walkForward();
} catch(FAIL_PRECONDITIONS, !error) {
  op:log("info", "Cannot walk forward: !error");
  exit(FAIL_PRECONDITIONS, !error);
} catch(FAIL_OVERALLCONDITIONS, !error) {
  act:stopMoving();
  op:log("info", "Cannot keep walking: !error");
  exit(FAIL_OVERALLCONDITIONS, !error);
} finally {
  op:log("info", "In finally block.");
}