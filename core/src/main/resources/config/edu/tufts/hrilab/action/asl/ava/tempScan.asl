import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.util.List;
import java.lang.String;
import java.lang.Integer;

() = measureTemperature() {
    Integer !temp = 0;
    Variable !x;

    effects : {
        success : not(temperature(!currentLocation, !x));
        success : temperature(!currentLocation, !temp);
    }

    Symbol !currentLocation;
    !currentLocation = act:getCurrentLocation();
    !temp = act:measureTemperatureDegrees();
    op:log(info, "[measureTemperature] !currentLocation temp !temp");
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

() = measureTemperatureAtLocationsAndReportAt(Predicate ?locations, Symbol ?reportLocation) {
    act:measureTemperatureAtLocations(?locations);
    act:goToLocation(?reportLocation);
    act:generateTemperatureSummaryString();
}

() = getTemperatureAtLocationString(Symbol ?location) {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    !output = act:generateTemperatureAtLocationString(?location);
    String !contextId = this:getCommand();
    !output = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:invokeMethod(!output,"isEmpty")) {
        act:generateResponseFromString(!output);
    }
}
