import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

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
    op:log(info,"reach_pick");
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
    act:callPolicy("reach_drop(?disc,?from,?to)");

    op:log(info,"reach_drop");
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
    act:callPolicy("pick(?disc,?from)");

    op:log(info,"pick");
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
    act:callPolicy("drop(?disc,?to)");

    op:log(info,"drop");
    //act:failureTest();
}

() = RLPolicyFailedPolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {

  recovery: {
    failedActions : {drop(self:agent, ?disc:disc, ?to:stackable)}
  }

  op:log("debug", "executing RLPolicyFailedPolicy");

  act:update_rl();
}