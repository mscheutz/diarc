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

() = measureWifiAtCurrentLocationAndReport() {
    String !reportString;
    Symbol !currentLocation;

    !currentLocation = act:getCurrentLocation();
    act:measureWifiAtCurrentLocation();
    !reportString = act:generateWifiAtLocationString(!currentLocation);
    act:generateResponseFromString(!reportString);
}

() = measureWifiAtCurrentLocationAndReportAt(Symbol ?reportLocation) {
    String !reportString;
    Symbol !currentLocation;

    !currentLocation = act:getCurrentLocation();
    act:measureWifiAtCurrentLocation();
    act:goToLocation(?reportLocation);
    !reportString = act:generateWifiAtLocationString(!currentLocation);
    act:generateResponseFromString(!reportString);
}

() = measureWifiAtLocationsAndReportAt(Predicate ?locations, Symbol ?reportLocation) {
    String !reportString;
    List !locationList;

    !locationList = op:invokeMethod(?locations,"getArgs");
    act:measureWifiAtLocations(?locations);
    act:goToLocation(?reportLocation);
    !reportString = act:generateWifiAtLocationsString(!locationList);
    act:generateResponseFromString(!reportString);
}

() = reportPreviouslyMeasuredWifiStrengthsOfLocations(Predicate ?locations) {
    List !locList;
    String !output;

    !locList = op:invokeMethod(?locations, "getArgs");
    !output = act:generateWifiAtLocationsString(!locList);
    act:generateResponseFromString(!output);
}

() = reportPreviouslyMeasuredWifiOfCurrentLocation() {
    act:measureWifiAtCurrentLocation();

    Symbol !currentLocation;
    String !output;
    !currentLocation = act:getCurrentLocation();
    !output = act:generateWifiAtLocationString(!currentLocation);
    act:generateResponseFromString(!output);
}