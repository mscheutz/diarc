import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Boolean;
import java.lang.Long;
import java.lang.Integer;
import java.lang.String;
import java.util.List;
import java.util.Map;

() = startDemo() {

    Predicate !goalPredicate;
    Predicate !toAssert;
    Long !gid;

    !goalPredicate = op:createPredicate("scheduled(demoAction(ava), start(1, seconds), period(1, hours))");
    !gid = act:submitGoal(!goalPredicate);
    !toAssert = op:createPredicate("demoGoalId(!gid)");
    act:assertBelief(!toAssert);
}

() = stopDemo() {
    List !bindings;
    Map !binding;
    Predicate !query;
    Variable !x = X;
    Predicate !result;
    String !resultString;
    Long !gid;

    !query = op:createPredicate("demoGoalId(!x)");
    !bindings = act:queryBelief(!query);
    if (op:isEmpty(!bindings)) {
        op:log(warn, "[stopDemo] scheduled task query returned no results");
    } else {
        !binding= op:get(!bindings, 0);
        !result= op:get(!binding, !x);
        !resultString = op:getName(!result);
        op:log(debug, "[stopDemo] result: !result name: !resultString");
        !gid = op:parseLong(!resultString);
        op:log(debug, "[stopDemo] gid: !result");
        tsc:cancelGoal(!gid);
        op:log(info, "[stopDemo] successfully stopped the demo");
    }
    act:retractBelief(!query);
}

() = demoAction() {
    String !matchString = "We are";
    act:goToLocation(location_4:location);
    act:showVideoMatching(!matchString);
    act:goHome();
}