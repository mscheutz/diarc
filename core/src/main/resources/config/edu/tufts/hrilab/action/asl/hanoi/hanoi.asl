import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;
import java.lang.String;

() = reach_pick(Symbol ?from:stackable, Symbol ?to:stackable) {
    conditions : {
        pre : clear(?to);
        pre : free(?actor);
        pre : over(?actor, ?from);
    }
    effects : {
        success : over(?actor, ?to);
        success : not(over(?actor, ?from));
    }
    op:log(debug,"reach_pick");
    act:callPolicy("reach_pick(?from,?to)");
}

() = reach_drop(Symbol ?disc:disc, Symbol ?from:stackable, Symbol ?to:stackable) {
    conditions : {
        pre : grasped(?disc);
        pre : clear(?to);
        pre : over(?actor, ?from);
    }
    effects : {
        success : over(?actor, ?to);
        success : not(over(?actor, ?from));
    }
    op:log(debug,"reach_drop");
    act:callPolicy("reach_drop(?disc,?from,?to)");
}

() = pick(Symbol ?disc:disc, Symbol ?from:stackable) {
    conditions : {
        pre : clear(?disc);
        pre : over(?actor, ?disc);
        pre : on(?disc, ?from);
        pre : free(?actor);
    }
    effects : {
        success : grasped(?disc);
        success : not(clear(?disc));
        success : not(on(?disc, ?from));
        success : not(free(?actor));
        success : not(over(?actor, ?disc));
        success : over(?actor, ?from);
        success : clear(?from);
    }
    op:log(debug,"pick");
    act:callPolicy("pick(?disc,?from)");
}


() = drop(Symbol ?disc:disc, Symbol ?to:stackable) {
    conditions : {
        pre : over(?actor, ?to);
        pre : grasped(?disc);
        pre : clear(?to);
        pre : smaller(?to, ?disc);
    }
    effects : {
        success : not(grasped(?disc));
        success : not(clear(?to));
        success : clear(?disc);
        success : on(?disc, ?to);
        success : free(?actor);
    }
    op:log(debug,"drop");
    act:callPolicy("drop(?disc,?to)");
}

() = rlPolicyFailedPolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {
  recovery: {
    failureReasons : {failedPolicy(?action)}
  }

  op:log("debug", "executing RLPolicyFailedPolicy");
  act:learnPolicy("?failedActionPredicate");
}