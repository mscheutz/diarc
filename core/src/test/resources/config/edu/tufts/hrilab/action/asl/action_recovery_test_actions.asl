() = pickUp(edu.tufts.hrilab.fol.Symbol ?obj) {
    conditions : {
      pre: sees(?actor, ?obj);
    }
    op:log("info", "Executing pickUp action.");
}

() = findObject(edu.tufts.hrilab.fol.Symbol ?obj) {
    effects : {
      success: sees(?actor, ?obj);
    }
    op:log("info", "Executing findObject action.");
    //exit(FAIL, "forced(failure)");
}
