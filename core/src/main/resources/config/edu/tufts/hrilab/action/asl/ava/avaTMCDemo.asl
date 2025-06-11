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

    !goalPredicate = op:createPredicate("scheduled(showVideo(ava, location_4:location), start(3, minutes), period(1, hours))");
    !gid = act:submitGoal(!goalPredicate);
    !toAssert = op:createPredicate("demoGoalId(!gid)");
    act:assertBelief(!toAssert);
    !goalPredicate = op:createPredicate("scheduled(doALap(ava), start(1, seconds), period(30, minutes))");
    !gid = act:submitGoal(!goalPredicate);
    !toAssert = op:createPredicate("demoGoalId(!gid)");
    act:assertBelief(!toAssert);
}

() = doALap() {
    act:goToLocation(location_6:location);
    act:showVideo();
    act:goToLocation(location_1:location);
    act:goToLocation(location_2:location);
    act:showVideo();
    act:goToLocation(location_0:location);
}

() = stopDemo() {
    List !bindings;
    Predicate !query;
    Variable !x = X;
    Predicate !result;
    String !resultString;
    Long !gid;
    Map !binding;

    !query = op:createPredicate("demoGoalId(!x)");
    !bindings = act:queryBelief(!query);
    if (op:isEmpty(!bindings)) {
        op:log(warn, "[stopDemo] scheduled task query returned no results");
    } else {
        foreach (!binding : !bindings) {
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
    act:goHome();
}