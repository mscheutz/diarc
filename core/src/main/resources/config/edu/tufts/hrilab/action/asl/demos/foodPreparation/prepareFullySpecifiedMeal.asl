import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.List;
import java.util.ArrayList;

() = prepareHamburgerFrenchFriesCoke["prepares a fully specified order of hamburger, french fries, and a coke"]() {

    Term !props;

    op:log(debug, "positing reference for hamburger");
    Symbol !hamburgerRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(hamburger(X:physobj))");
    !hamburgerRefId = act:positReference(!props);

    op:log(debug, "positing reference for french fries");
    Symbol !frenchFriesRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(frenchfries(X:physobj))");
    !frenchFriesRefId = act:positReference(!props);

    op:log(debug, "positing reference for coke");
    Symbol !cokeRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(coke(X:physobj))");
    !cokeRefId = act:positReference(!props);

    op:log(debug, "positing reference for tray");
    Symbol !trayRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tray(X:physobj))");
    !trayRefId = act:positReference(!props);

//    goal:and(prepared(!hamburgerRefId), itemOn(!hamburgerRefId, !trayRefId), prepared(!frenchFriesRefId), itemOn(!frenchFriesRefId,!trayRefId), prepared(!cokeRefId), itemOn(!cokeRefId, !trayRefId));
    act:prepareitemfrenchfries(!frenchFriesRefId, !trayRefId);
    goal:itemAt(!trayRefId, pose_6:pose);

    op:log(info, "[prepareHamburgerFrenchFriesCoke] all items delivered to location_4:location");
}


() = preparePizza() {

    Term !props;

    op:log(debug, "positing reference for pizza");
    Symbol !pizzaRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(pizzaItem(X:physobj))");
    !pizzaRefId = act:positReference(!props);

    op:log(debug, "positing reference for coke");
    Symbol !cokeRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(cokeItem(X:physobj))");
    !cokeRefId = act:positReference(!props);

    op:log(debug, "positing reference for tray");
    Symbol !trayRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tray(X:physobj))");
    !trayRefId = act:positReference(!props);

    List !toppingTypes;
    !toppingTypes = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    op:add(!toppingTypes, pepperoni);

    act:prepareitempizza(!pizzaRefId, !toppingTypes, !trayRefId);
    act:preparedrink(!cokeRefId, !trayRefId);
    goal:itemAt(!trayRefId, pose_6:pose);

    op:log(info, "[preparePizza] all items delivered to location_4:location");
}

() = prepareTaco() {

    Term !props;

    op:log(debug, "positing reference for taco");
    Symbol !tacoRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tacoItem(X:physobj))");
    !tacoRefId = act:positReference(!props);

    op:log(debug, "positing reference for tray");
    Symbol !trayRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tray(X:physobj))");
    !trayRefId = act:positReference(!props);

    act:prepareitemtaco(!tacoRefId, !trayRefId);
    goal:itemAt(!trayRefId, pose_6:pose);

    op:log(info, "[prepareTaco] all items delivered to location_4:location");
}

() = testHamburgerPrepare() {
    act:init();

    act:prepareHamburgerFrenchFriesCoke();
}

() = testPizzaPrepare() {
    act:init();

    act:preparePizza();
}

() = testTacoPrepare() {
    act:init();

    act:prepareTaco();
}

() = test() {
    act:init();

    Symbol !trayRefId;
    Term !props;

    op:log(debug, "positing reference for tray");
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tray(X:physobj))");
    !trayRefId = act:positReference(!props);


//    !goalState = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "prepare(?actor,frenchfries,!trayRefId)");
//    op:log(debug,"[prepare ] goal state: !goalState");
//    act:submitGoal(!goalState);
    act:prepare(frenchfries,!trayRefId);
}
