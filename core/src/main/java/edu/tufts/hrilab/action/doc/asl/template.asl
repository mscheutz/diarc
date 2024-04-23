import edu.tufts.hrilab.fol.Symbol;

(java.lang.String var2 = "def") = actionName["description"](Symbol ?var1) {
    benefit = 4.0;
    cost = 3.0;
    minurg = 0.6;
    maxurg = 0.7;
    timeout = 0;

    conditions : {
      pre : precond();
      or : {
        pre false : notObservable();
        pre true : observable();
        pre : default();
      }
      overall : overall();
    }
    effects : {
      always : alwayseffect();
      always : alwayssideeffect();
      success : successeffect();
      success true : successobservable();
      failure : failureeffect();
      nonperf : nonperformance();
    }

    locks : motionLock;

    if (op:gt(1, 0)) {
        ?actor.act:action(arg1, arg2, 3.1415);
    }
}