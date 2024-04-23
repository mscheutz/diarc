() = patrolTheLab["?actor patrols"]() {
    op:log("debug", "In patrolTheLab script for PR2.");
    act:clean(roomOne);
    act:clean(roomTwo);
    act:clean(roomThree);
}

() = clean["?actor cleans room ?room"](edu.tufts.hrilab.fol.Symbol ?room) {
    edu.tufts.hrilab.fol.Symbol !Someone;
    conditions : {
        pre : not(isIn(!Someone,?room));
        overall : not(isIn(!Someone,?room));
    }
    op:log("debug", "In patrol script for PR2.");
    act:moveTo(?room, true);
    act:clean();
}

