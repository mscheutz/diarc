import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

() = measureWifiAtCurrentLocation() {
    Integer !rssi = 0;
    Variable !x;

    effects : {
        success : not(wifiStrength(!currentLocation, !x));
        success : wifiStrength(!currentLocation, !rssi);
    }

    //TODO: understand interaction with "unknown" may be weird
    Symbol !currentLocation;
    !currentLocation = act:getCurrentLocation();
    !rssi = act:measureWifiRssi();
    op:log(info, "[measureWifi] !currentLocation rssi !rssi");
}

() = measureWifiAt(Symbol ?location) {
    Integer !rssi = 0;
    Variable !x;

    effects : {
        success : not(wifiStrength(?location, !x));
        success : wifiStrength(?location, !rssi);
    }

    act:goToLocation(?location);
    !rssi = act:measureWifiRssi();
    op:log(info, "[measureWifiAt] ?location rssi !rssi");
}

() = measureWifiAtLocations(Predicate ?locations) {
    List !locList;
    Symbol !location;

    !locList = op:invokeMethod(?locations, "getArgs");
    foreach(!location : !locList) {
        act:measureWifiAt(!location);
    }
}

() = measureWifiAtCurrentLocationAndReportAt(Symbol ?reportLocation) {
    act:measureWifiAtCurrentLocation();
    act:goToLocation(?reportLocation);
    act:generateWifiSummaryString(?reportLocation);
}

() = measureWifiAtLocationsAndReportAt(Predicate ?locations, Symbol ?reportLocation) {
    act:measureWifiAtLocations(?locations);
    act:goToLocation(?reportLocation);
    act:generateWifiSummaryString(?reportLocation);
}

() = reportWifiAtLocation(Symbol ?location) {
    String !output;
    !output = act:generateWifiAtLocationString(?location);
    act:generateResponseFromString(!output);
}

() = getPreviouslyMeasuredWifiAtLocationString(Symbol ?location) {
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    java.lang.Long !priority;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier;
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    edu.tufts.hrilab.slug.common.Utterance !response;
    //Predicate !response;
    !output = act:generateWifiAtLocationString(?location);
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

() = reportPreviouslyMeasuredWifiOfLocations(Predicate ?locations) {
    List !locList;
    Symbol !location;
    String !output;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");

    !locList = op:invokeMethod(?locations, "getArgs");
    foreach(!location : !locList) {
        !output = act:generateWifiAtLocationString(!location);
        op:invokeMethod(!sb,"append",!output);
        op:invokeMethod(!sb,"append",".");
    }
    !output = op:invokeMethod(!sb,"toString");
    act:generateResponseFromString(!output);
}

() = getPreviouslyMeasuredWifiAtLocationsString(Predicate ?locations) {
    List !locList;
    Symbol !location;
    String !output;
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    edu.tufts.hrilab.slug.common.Utterance !response;
    //Predicate !response;

    !locList = op:invokeMethod(?locations, "getArgs");
    foreach(!location : !locList) {
        !output = act:generateWifiAtLocationString(!location);
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

() = reportPreviouslyMeasuredWifiOfCurrentLocation() {
    act:measureWifiAtCurrentLocation();

    Symbol !currentLocation;
    String !output;
    !currentLocation = act:getCurrentLocation();
    !output = act:generateWifiAtLocationString(!location);
    act:generateResponseFromString(!output);
}

() = getPreviouslyMeasuredWifiAtCurrentLocationString() {
    act:measureWifiAtCurrentLocation();
    edu.tufts.hrilab.slug.common.Utterance !response;
    //Predicate !response;

    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    java.lang.Long !priority;
    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier;
    Symbol !currentLocation;
    String !output;
    !currentLocation = act:getCurrentLocation();
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateWifiAtLocationString(!location);
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
