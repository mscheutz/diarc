() = screwIn(edu.tufts.hrilab.fol.Symbol ?actor, edu.tufts.hrilab.fol.Symbol ?var_1, edu.tufts.hrilab.fol.Symbol ?var_0) {
    ?actor.act:perceiveEntity(?var_0);
    ?actor.act:mountScrew(?var_1);
    ?actor.act:alignWith(?var_0);
    ?actor.act:runScrewdriverJob(?var_1);
}