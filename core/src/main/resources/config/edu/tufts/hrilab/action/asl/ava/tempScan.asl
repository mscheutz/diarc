import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

() = measureTemperatureAtCurrentLocation() {
    Integer !temp = 0;
    Variable !x;

    effects : {
        success : not(temperature(!currentLocation, !x));
        success : temperature(!currentLocation, !temp);
    }

    Symbol !currentLocation;
    !currentLocation = act:getCurrentLocation();
    !temp = act:measureTemperatureDegrees();
    op:log(info, "[measureTemperatureAtCurrentLocation] !currentLocation temp !temp");
}

() = measureTemperatureAt(Symbol ?location) {
    Integer !temp = 0;
    Variable !x;

    effects : {
        success : not(temperature(?location, !x));
        success : temperature(?location, !temp);
    }

    act:goToLocation(?location);
    !temp = act:measureTemperatureDegrees();
    op:log(info, "[measureTemperatureAt] ?location temp !temp");
}

() = measureTemperatureAtLocations(Predicate ?locations) {
    List !locList;
    Symbol !location;

    !locList = op:invokeMethod(?locations, "getArgs");
    foreach(!location : !locList) {
        act:measureTemperatureAt(!location);
    }
}

() = measureTemperatureAtCurrentLocationAndReportAt(Symbol ?reportLocation) {
    act:measureTemperatureAtCurrentLocation();
    act:goToLocation(?reportLocation);
    act:generateTemperatureSummaryString();
}

() = measureTemperatureAtLocationsAndReportAt(Predicate ?locations, Symbol ?reportLocation) {
    act:measureTemperatureAtLocations(?locations);
    act:goToLocation(?reportLocation);
    act:generateTemperatureSummaryString();
}

() = reportTemperatureAtLocation(Symbol ?location) {
    String !output;
    !output = act:generateTemperatureAtLocationString(?location);
    act:generateResponseFromString(!output);
}

() = getPreviouslyMeasuredTemperatureAtLocationString(Symbol ?location) {
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    java.lang.Long !priority;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier;
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    edu.tufts.hrilab.slug.common.Utterance !response;
    //Predicate !response;
    !output = act:generateTemperatureAtLocationString(?location);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        act:handleParsedUtterance(!response);
    //    !priorityTier = act:getPriorityTierForGoal(!response);
    //    !priority = act:getPriorityForGoal(!response);
    //    !goalID = act:submitGoal(!response,!execType,!priorityTier,!priority,!user);
    //    act:joinOnGoal(!goalID);
    //    !output = op:getName(!response);
    //    if (op:equals(!output,"generateResponseFromString")) {
    //      act:listenPostCommunicateUtterance(!user);
    //    }
    }
}

() = reportPreviouslyMeasuredTemperatureOfLocations(Predicate ?locations) {
    List !locList;
    Symbol !location;
    String !output;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");

    !locList = op:invokeMethod(?locations, "getArgs");
    foreach(!location : !locList) {
        !output = act:generateTemperatureAtLocationString(!location);
        op:invokeMethod(!sb,"append",!output);
        op:invokeMethod(!sb,"append",".");
    }
    !output = op:invokeMethod(!sb,"toString");
    act:generateResponseFromString(!output);
}

() = getPreviouslyMeasuredTemperatureAtLocationsString(Predicate ?locations) {
    List !locList;
    Symbol !location;
    String !output;
    edu.tufts.hrilab.slug.common.Utterance !response;
    //Predicate !response;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");

    !locList = op:invokeMethod(?locations, "getArgs");
    foreach(!location : !locList) {
        !output = act:generateTemperatureAtLocationString(!location);
        op:invokeMethod(!sb,"append",!output);
        op:invokeMethod(!sb,"append",".");
    }
    !output = op:invokeMethod(!sb,"toString");

    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    java.lang.Long !priority;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier;
    Symbol !user = this:getOriginalSubmitter();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        act:handleParsedUtterance(!response);
    //    !priorityTier = act:getPriorityTierForGoal(!response);
    //    !priority = act:getPriorityForGoal(!response);
    //    !goalID = act:submitGoal(!response,!execType,!priorityTier,!priority,!user);
    //    act:joinOnGoal(!goalID);
    //    !output = op:getName(!response);
    //    if (op:equals(!output,"generateResponseFromString")) {
    //      act:listenPostCommunicateUtterance(!user);
    //    }
    }
}

() = reportPreviouslyMeasuredTemperatureOfCurrentLocation() {
    act:measureTemperatureAtCurrentLocation();

    Symbol !currentLocation;
    String !output;
    !currentLocation = act:getCurrentLocation();
    !output = act:generateTemperatureAtLocationString(!location);
    act:generateResponseFromString(!output);
}

() = getPreviouslyMeasuredTemperatureAtCurrentLocationString() {
    act:measureTemperatureAtCurrentLocation();

    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    java.lang.Long !priority;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier;
    Symbol !currentLocation;
    String !output;
    edu.tufts.hrilab.slug.common.Utterance !response;
    //Predicate !response;
    !currentLocation = act:getCurrentLocation();
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateTemperatureAtLocationString(!location);
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        act:handleParsedUtterance(!response);
    //    !priorityTier = act:getPriorityTierForGoal(!response);
    //    !priority = act:getPriorityForGoal(!response);
    //    !goalID = act:submitGoal(!response,!execType,!priorityTier,!priority,!user);
    //    act:joinOnGoal(!goalID);
    //    !output = op:getName(!response);
    //    if (op:equals(!output,"generateResponseFromString")) {
    //      act:listenPostCommunicateUtterance(!user);
    //    }
    }
}