import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

() = testLuca() {
    Term !tmpTerm;
    Term !ruleHead;
    java.util.List !ruleBody;

    op:log("info", "Setting up beliefs...");

    // This is ?actor. He's human.
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "creature(?actor,human)");
    act:assertBelief(!tmpTerm);

    // For ?actor, believing is being.
    !ruleBody = op:newObject("java.util.ArrayList");
    !ruleHead = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "self(A,B)");
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "belief(?actor,self(A,B))");
    op:invokeMethod(!ruleBody, "add", !tmpTerm);
    act:assertRule(!ruleHead, !ruleBody);
    op:invokeMethod(!ruleBody, "clear");

    // When threshold is crossed, ?actor believes he's done.
    !ruleHead = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "belief(?actor,self(?actor,done))");
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "threshold_crossed(?actor)");
    op:invokeMethod(!ruleBody, "add", !tmpTerm);
    act:assertRule(!ruleHead, !ruleBody);
    op:invokeMethod(!ruleBody, "clear");

    // Taking 10 steps crosses threshold
    !ruleHead = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "threshold_crossed(X)");
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "steps(X,10)");
    op:invokeMethod(!ruleBody, "add", !tmpTerm);
    act:assertRule(!ruleHead, !ruleBody);
    op:invokeMethod(!ruleBody, "clear");

    // Agents can celebrate when they're done!
    !ruleHead = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "can(X,celebrate)");
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "self(X,done)");
    op:invokeMethod(!ruleBody, "add", !tmpTerm);
    act:assertRule(!ruleHead, !ruleBody);
    op:invokeMethod(!ruleBody, "clear");

    try {
      act:celebrate();
    } catch(fail_preconditions, !predicates) {
      op:log("info", "Of course, ?actor refuses: !predicates");
      !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "notcelebrated(?actor)");
      if (act:querySupport(!tmpTerm)) {
        op:log("info", "The beliefs have been updated accordingly!");
      } else {
        op:log("error", "The beliefs have NOT been updated. That's not good!");
        exit(FAIL);
      }
    }

    try {
      act:step(10);
      !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "threshold_crossed(?actor)");
      if (act:querySupport(!tmpTerm)) {
        op:log("info", "?actor crossed the threshold.");
        !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "belief(?actor,self(?actor,done))");
        if (act:querySupport(!tmpTerm)) {
          op:log("info", "?actor believes he's done.");
        }
      } else {
        op:log("error", "?actor does not believe he's done after 10 steps");
        exit(FAIL);
      }
      act:celebrate();
    } catch(fail_preconditions, !predicates) {
      op:log("info", "Surprisingly, ?actor refuses. He says: !predicates");
    } catch() {
      op:log("error", "Unexpected error. Not good!");
      exit(FAIL);
    }

    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "celebrated(?actor)");
    if (~act:querySupport(!tmpTerm)) {
      op:log("error", "The beliefs have NOT been updated. That's not good!");
      exit(FAIL);
    }

    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,location0)");
    act:assertBelief(!tmpTerm);
    act:moveTo(locationA);
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,locationA)");
    if (act:querySupport(!tmpTerm)) {
      op:log("info", "1) ?actor is at A!");
    } else {
      op:log("error", "1) ?actor is NOT at A! Thats an error!!");
      exit(FAIL);
    }

    act:moveTo(locationB);
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,locationA)");
    if (act:querySupport(!tmpTerm)) {
      op:log("error", "---> ?actor IS STILL AT A!");
      exit(FAIL);
    }

    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,locationB)");
    if (act:querySupport(!tmpTerm)) {
      op:log("info", "2) ?actor is at B. All is well.");
    } else {
      op:log("error", "2) ?actor is NOT at B! Thats an error!!");
      exit(FAIL);
    }

    op:log("info", "Let's clean up the mess we made here...");
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    act:retractBelief(!tmpTerm);
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "steps(?actor,10)");
    act:retractBelief(!tmpTerm);
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "celebrated(?actor)");
    act:retractBelief(!tmpTerm);
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "notcelebrated(?actor)");
    act:retractBelief(!tmpTerm);
    !tmpTerm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,locationB)");
    act:retractBelief(!tmpTerm);
}

() = step(int ?nbr = 0) {
    int !i = 0;

    conditions : {
      pre : creature(?actor,human);
    }
    effects : {
      success : steps(?actor,?nbr);
    }

    while (op:lt(!i, ?nbr)) {
      !i = op:++(!i);
      op:log("info", "step !i");
    }
}

() = celebrate() {
    conditions : {
      pre : can(?actor,celebrate);
    }
    effects : {
      success : celebrated(?actor);
      failure : notcelebrated(?actor);
    }

    op:log("info", "?actor celebrates!");
}

() = countInBackground(int ?i, int ?target) {
    effects : {
      success : countedTo(?target);
    }

    op:+(0, 0, ?i);
    while (op:lt(?i, ?target)) {
      ?i = op:++(?i);
      op:log("info", "Counting: ?i");
      op:sleep(100);
    }
    op:log("info", "I just counted to ?i!");
}

() = moveTo(Symbol ?destination) {
    Symbol !start;

    conditions : {
      or : {
        pre : at(?actor,!start);
        pre : not(at(?actor,!start));
      }
    }
    effects : {
      success : at(?actor,?destination);
      success : not(at(?actor,!start));
    }

    op:log("info", "?actor moved from !start to ?destination");
}

() = beliefTest() {
    Predicate !state;

    !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "countedTo(3)");
    while (true) {
      act:submitSimGoal(!state);
    }
}

() = pickUpTest(Symbol ?object) {
    conditions : {
      pre obs : touching(?object);
    }

    op:log("info", "I am touching: ?object");
}

(java.util.List ?return) = observer(Predicate ?predicate) {
    java.util.Map !bindings;
    Symbol !x;

    observes : touching(!x);

    ?return = op:newArrayList("java.util.Map");
    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    if (op:isVariable(!x)) {
      op:put(!bindings, !x, mug);
    }
    op:add(?return, !bindings);
    op:log("info", "Observing ?predicate -> ?return");
}
