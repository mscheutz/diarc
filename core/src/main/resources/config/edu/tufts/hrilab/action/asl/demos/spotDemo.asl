import edu.tufts.hrilab.fol.Symbol;
import java.lang.String;
import java.lang.Integer;

() = init[""]() {
    act:claim();
    act:powerOn();
    act:setInitialLocation(location_0:location);
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