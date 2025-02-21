import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.Map;
import java.util.List;

() = scheduleGoal() {

    Map !bindings;
    Variable !x = "X";
    Variable !y = "Y";
    Predicate !goalPredicate;
    Predicate !scheduledGoal;
    Predicate !confirmation;
    Symbol !interval;
    Symbol !time;
    Symbol !addl;
    Symbol !intervalUnits;
    Symbol !timeSymbol = "none";

    !bindings = act:askQuestionFromString(?actor, "What would you like to do?", val(X));
    !goalPredicate = op:get(!bindings, !x);
    op:log(debug, "[scheduleGoal] goal !goalPredicate");

    //!bindings = act:askQuestionFromString(?actor, "When should I perform this goal?", time(X));
    //!time = op:get(!bindings, !x);
    //op:log(debug, "[scheduleGoal] time !time");

    !bindings = act:askQuestionFromString(?actor, "When should this task be scheduled for?", val(X,Y));
    !time = op:get(!bindings, !x);
    !addl = op:get(!bindings, !y);
    if (~op:equalsValue(!addl, pm())) {
      if (~op:equalsValue(!addl, am())) {
        !time = act:parseTime(!time, !addl);
        !timeSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "notNone"); //this is a bad way to do this!
      }
    }
    op:log(debug, "[scheduleGoal] time/units: !time/!addl");

    !bindings = act:askQuestionFromString(?actor, "Should this task be repeated?", val(X));
    !confirmation = op:get(!bindings, !x);

    if (op:equalsValue(!confirmation, yes())) {
      !bindings = act:askQuestionFromString(?actor, "How often?", val(X,Y));
      !interval = op:get(!bindings, !x);
      !intervalUnits = op:get(!bindings, !y);
      if (op:equalsValue(!timeSymbol, "none")) {
          !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval, !intervalUnits)), !time, !addl)");
      } else  {
          !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, scheduled(?actor, !goalPredicate, !interval, !intervalUnits)), !time)");
      }
    } else {
      if (op:equalsValue(!timeSymbol, "none")) {
          !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, !goalPredicate, !time, !addl)");
      } else {
          !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, !goalPredicate, !time)");
      }
    }
    op:log(info, "[scheduleGoal] scheduledGoal: !scheduledGoal");

    act:submitGoal(!scheduledGoal);

    act:generateResponse(?actor, "added(!scheduledGoal)", direct);
}

() = forEach(Predicate ?collection, Symbol ?actionName) {
    Predicate !goalPred;
    List !args;
    Symbol !arg;

    !args = op:invokeMethod(?collection, "getArgs");
    foreach (!arg : !args) {
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?actionName(?actor, !arg)");
        act:submitGoal(!goalPred);
    }
}