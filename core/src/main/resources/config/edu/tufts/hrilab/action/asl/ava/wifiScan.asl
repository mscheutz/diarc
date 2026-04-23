import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

() = measureWifi() {
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

() = measureWifiAtLocationsAndReportAt(Predicate ?locations, Symbol ?reportLocation) {
    act:measureWifiAtLocations(?locations);
    act:goToLocation(?reportLocation);
    act:generateWifiSummaryString();
}

() = getWifiAtLocationString(Symbol ?location) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateWifiAtLocationString(?location);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}

() = getWifiAtLocationsString(Predicate ?locations) {
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

    Symbol !user = this:getOriginalSubmitter();
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}

() = getWifiAtCurrentLocationString() {
    act:measureWifi();

    Symbol !currentLocation;
    String !output;
    !currentLocation = act:getCurrentLocation();
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateWifiAtLocationString(!location);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
