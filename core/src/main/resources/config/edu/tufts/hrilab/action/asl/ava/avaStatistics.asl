import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.Map;
import java.util.List;
import java.lang.String;
import java.lang.Long;
import java.lang.Integer;
import java.lang.Double;
import java.lang.Boolean;

// TODO: Revisit statistic selection UX: consider typed response form (e.g. statisticType(X))
// or alternative to list+option(N) for "which statistic?" — see docs/AVA_STATISTICS.md.

() = discussRobotPerformance() {
    Map !bindings;
    Variable !x = "X";
    List !statIds;
    Integer !numStats;
    Integer !idx;
    Integer !displayNum;
    Integer !optionNumber;
    Integer !selectedIndex;
    Symbol !response;
    java.lang.String !numberString;
    java.lang.String !statId;
    java.lang.String !displayName;
    java.lang.String !startDateStr;
    java.lang.String !endDateStr;
    Long !startMs;
    Long !endMs;
    Double !value;
    Predicate !reportPred;
    java.util.List !responseFormList;
    Predicate !responseForm;

    !statIds = tsc:getAvailableStatisticIds();
    if (op:isEmpty(!statIds)) {
        act:generateResponseFromString("There are no tracked statistics configured");
        exit(FAIL,not(exist(trackedStatistics)));
    }
    !numStats = op:invokeMethod(!statIds,"size");
    act:generateResponseFromString("The tracked statistics are");
    !displayNum = op:newObject("java.lang.Integer", "0");
    for (!idx = 0; !idx lt !numStats; !idx ++) {
        !displayNum = op:++(!displayNum);
        !statId = op:get(!statIds,!idx);
        !displayName = tsc:getStatisticDisplayName(!statId);
        act:generateResponseFromString("Option !displayNum: !displayName");
    }

    act:suspendWaitForResponse(option(X),?actor,false);
    !bindings = act:askQuestionFromString("What information would you like to know about? Say the option number.", option(X));
    act:resumeWaitForResponse(option(X),?actor);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !response = op:get(!bindings,!x);
    !numberString = op:getName(!response);
    !optionNumber = op:newObject("java.lang.Integer", !numberString);
    !selectedIndex = op:--(!optionNumber);
    !statIds = tsc:getAvailableStatisticIds();
    !numStats = op:invokeMethod(!statIds,"size");
    if (op:lt(!selectedIndex,0) || op:gte(!selectedIndex,!numStats)) {
        act:generateResponseFromString("That option was not valid");
        exit(FAIL,not(know(?actor,whichStatistic)));
    }
    !statId = op:get(!statIds,!selectedIndex);

    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("date(X)");
    op:add(!responseFormList,!responseForm);

    !bindings = act:askQuestionFromString("When is the start of the analysis period?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !response = op:get(!bindings,!x);
    !startDateStr = op:getName(!response);

    !bindings = act:askQuestionFromString("When is the end of the analysis period?", !responseFormList);
    if (op:invokeMethod(!bindings,"isEmpty")) {
        exit(FAIL,not(wasAnswered(myQuestion)));
    }
    !response = op:get(!bindings,!x);
    !endDateStr = op:getName(!response);

    !startMs = tsc:parseDateToStartMs(!startDateStr);
    !endMs = tsc:parseDateToEndMs(!endDateStr);
    !value = tsc:queryStatistics(!statId, !startMs, !endMs);

    !reportPred = tsc:createStatisticsReportPredicate(!statId, !startDateStr, !endDateStr, !value);
    act:generateResponse(!reportPred);
}
