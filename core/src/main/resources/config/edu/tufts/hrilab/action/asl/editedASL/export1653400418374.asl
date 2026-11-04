() = screwIn(edu.tufts.hrilab.fol.Symbol ?actor, edu.tufts.hrilab.fol.Symbol ?var_0, edu.tufts.hrilab.fol.Symbol ?var_1) {
    ?actor.act:alignWith(?actor, ?var_1);
    ?actor.act:runScrewdriverJob(?actor, ?var_0);
}
