import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.action.justification.Justification;

() = runPassTest1() {
  tsc:tsc1();
  ?actor.tsc:tsc1();

  Symbol !arg = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "one");
  tsc:tsc2(!arg);
  ?actor.tsc:tsc2(!arg);

  tsc:tsc3(?actor, !arg);
  ?actor.tsc:tsc3(?actor, !arg);

  tsc:tsc4();
  Justification !justification = tsc:tsc4();
  op:log("info", "Justification returned: !justification");
}

() = runFailTest1() {
  tsc:notExist();
}

() = runFailTest2() {
  Symbol !arg = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "one");
  tsc:notExist(!arg);
}

// test failure requires registering TSCTestHelper as a non "self" diarc agent
() = runFailTest3() {
  ?actor.tsc:tsc1();
}

// test passing requires registering TSCTestHelper as a "agent:hugo" diarc agent
() = runPassTest2() {
  hugo.tsc:tsc1();
}

() = runBooleanTest() {
  if (~tsc:tsc_bool_true()) {
    exit(FAIL);
  }

  if (tsc:tsc_bool_false()) {
    exit(FAIL);
  }

  if (~tsc:tsc_justification_true()) {
    exit(FAIL);
  }

  if (tsc:tsc_justification_false()) {
    exit(FAIL);
  }
}
