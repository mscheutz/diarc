import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.Map;
import java.util.List;
import java.lang.String;

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

() = modifyScheduledGoal() {
    List !currentGoals;

    !currentGoals = act:getCurrentGoals();
    op:log(debug, "[modifyScheduledGoal] current goals: !currentGoals");

    //todo: remove this hack. should query for actual goals and actual semantics
    act:sayText("There is one goal scheduled");
    act:sayText("Scheduled goal #1 is:");
    act:sayText("Perform lab patrol with wifi evaluation at 12:00 p.m.");

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

() = definePatrolPath(Symbol ?name) {
    Map !bindings;
    Variable !x = "X";
    Predicate !locationId;
    String !tmpString;
    Symbol !tmp;
    Predicate !location = "none()";
    List !locations;
    Predicate !collectionDefinition;
    Predicate !locationsPredicate;

    !locations = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    !bindings = act:askQuestionFromString(?actor,"What location is on the path?", val(X));
    !locationId = op:get(!bindings, !x);
    !tmpString = op:invokeMethod(!locationId, "getName");
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !tmpString);
    op:log(debug, "location: !tmp");
    op:add(!locations, !tmp);

    !bindings = act:askQuestionFromString(?actor,"Is there another location on the path?", val(X));
    !locationId = op:get(!bindings, !x);
    !tmpString = op:invokeMethod(!locationId, "getName");
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !tmpString);
    op:log(debug, "location: !tmp");
    while(~op:equalsValue(!locationId,"none()")){
        op:add(!locations, !tmp);
        !bindings = act:askQuestionFromString(?actor,"Is there another location on the path?", val(X));
        !locationId = op:get(!bindings, !x);
        op:log(debug, "location: !locationId");
    }
    !locationsPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and", !locations);
    !collectionDefinition = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?name, !locationsPredicate)");
    op:log(debug, "[definePatrolPath] collection definition: !collectionDefinition");
    act:assertBelief(!collectionDefinition);

}

() = patrol(Symbol ?collectionName, Symbol ?actionName) {

  Predicate !queryPred;
  Variable !x = "X";
  Predicate !locations;
  Map !elem;
  List !bindings;

  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "collection(?collectionName,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     !locations = op:get(!elem,!x);
     act:forEach(!locations, ?actionName);
  } else {
    exit(FAIL, known(patrol, ?collectionName));
  }
}

() = wifiEvaluation(Symbol ?locationId) {
    act:goToLocation(?locationId);
    op:log(info, "[wifiEvaluation] evaluating wifi quality at ?locationId");
    //todo: actual call to do something useful. log to somewhere?
}