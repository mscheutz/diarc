import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

() = boardElevator["demo actionscript for boarding elevator"]() {
    Symbol !downbutton;
    Symbol !thirdfloorbutton;
    Symbol !openDoor;

    !downbutton = op:newObject("edu.tufts.hrilab.fol.Symbol", "physobj_8:physobj");
    !thirdfloorbutton = op:newObject("edu.tufts.hrilab.fol.Symbol", "physobj_3:physobj");

    // go to outside panel
    op:log("info", ">> goToNearestElevator");
    act:goToNearestElevator();

    // press button
    op:log("info", ">> findAndPressButton");motes/origin/Unity_Broken_Welder
  remotes/origin/Unity_Dean
  remotes/origin/Unity_Dean_WebGL

    act:findAndPressButton(!downbutton);

    // wait for door to open
    op:log("info", ">> waitForOpenDoor");
    !openDoor = act:waitForOpenDoor();

    // board elevator through open door
    op:log("info", ">> boardElevator");
    act:boardElevator(!openDoor);

    // need to enter elevator first, before pressing button
    // op:log("info", ">> findAndPressButton");
    // act:findAndPressButton(!thirdfloorbutton);

    // wait to get on target floor
    //op:log("info", ">> waitForFloor");
    //act:waitForFloor(3);

    // switch floor maps
    //op:log("info", ">> setCurrentFloor");
    //act:setCurrentFloor(3);

    // wait for door to open
    // op:log("info", ">> waitForOpenDoor");
    // !openDoor = act:waitForOpenDoor();

    // exit elevator through open door
    // op:log("info", ">> exitElevator");
    // act:exitElevator(!openDoor);

    op:log("info", ">> done with elevator sequence");
}

/**
 * Simple action, with built in NLG response, for asking the robot to take you somewhere.
 */
() = take(Symbol ?follower, Symbol ?location) {
  act:goToLocation(?location);

  Predicate !response;
  !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "is(this, ?location)");
  act:generateResponse(?follower, !response, direct);
}