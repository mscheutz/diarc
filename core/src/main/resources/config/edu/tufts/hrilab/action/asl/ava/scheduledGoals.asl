import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.Map;

() = scheduleGoal() {

    Map !bindings;
    Variable !x = "X";
    Variable !y = "Y";
    Predicate !goalPredicate;
    Predicate !scheduledGoal;
    Predicate !confirmation;
    Symbol !time;
    Symbol !units;

    !bindings = act:askQuestionFromString(?actor, "What would you like to do?", val(X));
    !goalPredicate = op:get(!bindings, !x);
    op:log(debug, "[scheduleGoal] goal !goalPredicate");

    //!bindings = act:askQuestionFromString(?actor, "When should I perform this goal?", time(X));
    //!time = op:get(!bindings, !x);
    //op:log(debug, "[scheduleGoal] time !time");

    !bindings = act:askQuestionFromString(?actor, "How long should I wait before performing this goal?", val(X,Y));
    !time = op:get(!bindings, !x);
    !units = op:get(!bindings, !y);
    op:log(debug, "[scheduleGoal] time/units: !time/!units");

    !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, !goalPredicate, !time, !units)");
    op:log(debug, "[scheduleGoal] goal to submit: !scheduledGoal");

    !bindings = act:askQuestionFromString(?actor, "Should this task be repeated?", val(X));
    !confirmation = op:get(!bindings, !x);
    if (op:equalsValue(!confirmation, yes())) {
      !bindings = act:askQuestionFromString(?actor, "How often?", val(X,Y));
      !time = op:get(!bindings, !x);
      !units = op:get(!bindings, !y);
      !scheduledGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "scheduled(?actor, persistent(?actor, !scheduledGoal), !time, !units)");
      op:log(debug, "[scheduleGoal] scheduledGoal: !scheduledGoal");
    }

    act:submitGoal(!scheduledGoal);
}