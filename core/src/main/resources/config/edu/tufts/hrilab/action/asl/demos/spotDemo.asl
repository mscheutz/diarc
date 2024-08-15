() = init[""]() {
    act:claim();
    act:powerOn();
    act:stand();
    act:setInitialLocation("location_0:location");
}

() = shutDown() {
    java.lang.Integer !dockId = 521;

    act:dock(!dockId);
    act:powerOff();
}