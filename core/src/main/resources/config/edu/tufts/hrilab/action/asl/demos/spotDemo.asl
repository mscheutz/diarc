() = init[""]() {
    act:claim();
    act:powerOn();
    act:stand();
}

() = shutDown() {
    java.lang.Integer !dockId = 521;

    act:dock(!dockId);
    act:powerOff();
}