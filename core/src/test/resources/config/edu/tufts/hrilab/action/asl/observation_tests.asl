() = observationTest() {
    edu.tufts.hrilab.fol.Predicate !predicate;
    edu.tufts.hrilab.fol.Symbol !x;

    // TODO: should this be allowed?
    // (!predicate) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "touching(!x)");
    // op:!predicate

    if (obs:touching(mug)) {
      op:log("info", "touching mug observed");
    } else {
      op:log("info", "touching mug NOT observed");
      exit(FAIL);
    }

    if (obs:touching(apple)) {
      op:log("info", "touching apple observed");
      exit(FAIL);
    } else {
      op:log("info", "touching apple NOT observed");
    }

    if (obs:touching(!x)) {
      op:log("info", "touching !x observed");
    } else {
      op:log("info", "touching !x NOT observed");
      exit(FAIL);
    }
}

(java.util.List ?return) = observer(edu.tufts.hrilab.fol.Predicate ?predicate) {
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Symbol !arg;
    edu.tufts.hrilab.fol.Symbol !mug = mug;

    observes : touching(!arg);

    ?return = op:newArrayList("java.util.Map");
    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    if (op:invokeStaticMethod("edu.tufts.hrilab.action.util.Utilities", "isScriptVariable", !arg)) {
      // observed touching(mug)
      op:put(!bindings, !arg, mug);
      op:add(?return, !bindings);
    } elseif (op:equals(!arg, !mug)) {
      // requested observation made:  touching(mug)
      op:add(?return, !bindings);
    } else {
      // requested observation not made. don't add bindings to return list
    }

    op:log("info", "Observing ?predicate -> ?return");
}
