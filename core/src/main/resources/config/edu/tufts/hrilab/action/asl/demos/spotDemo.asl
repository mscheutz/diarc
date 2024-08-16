import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;

//todo: this doesn't actually work, since it will happily localize to the wrong place? if we sit down etc it assumes we're closest to the dock location where the tour started i think.
() = failedLocalization(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {
  Symbol !defaultLocation = location_0:location;

  recovery: {
    failedActions: {init(self:agent)}
    actionStatuses: {FAIL_RETURNVALUE}
  }

  act:setInitialLocation(!defaultLocation);
  op:log(warn, "[failedLocalization] location initialized to !defaultLocation after failing to localize");
}


() = init[""]() {
    Boolean !result;

    act:claim();
    act:powerOn();

    !result = act:hasBeenLocalized();

    if (op:equals(!result, true)) {
        op:log(debug, "[init] no need to localize again");
    } else {
        act:localize();
    }
    op:log(debug, "[init] set up spot");
}

() = map() {
    Symbol !location;
    Integer !numLocations;
    !numLocations = tsc:getNumberOfKnownLocations();
    for (!i=0; !i lt !numLocations; !i ++) {
        !location = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "location_!i:location");
        act:goToLocationUnsafe(!location);
        act:bindLocationToCurrentPose(!location);
        op:log(debug, "[map] mapped !location");
    }
    tsc:writeReferencesToFile("/home/pete/dev/diarc/core/src/main/resources/config/edu/tufts/hrilab/spot/spotLocationRefs.json");
    act:localize();
}

() = wander() {
    Symbol !location;
    Integer !numLocations;
    !numLocations = tsc:getNumberOfKnownLocations();
    for (!i=0; !i lt !numLocations; !i ++) {
        !location = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "location_!i:location");
        act:goToLocation(!location);
        op:log(debug, "[wander] at location !location");
    }
    op:log(info, "[wander] done with tour. at location !location");
}

() = shutDown() {
    java.lang.Integer !dockId = 521;

    act:dock(!dockId);
    act:powerOff();
}

() = reset() {
    act:goToLocationUnsafe(location_1:location);
    act:shutDown();
    act:undock();
}

() = test() {
    act:map();
    act:wander();
}