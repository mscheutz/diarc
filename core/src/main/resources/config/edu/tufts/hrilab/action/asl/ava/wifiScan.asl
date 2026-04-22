import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

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
