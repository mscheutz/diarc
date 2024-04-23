import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Integer;
import java.util.List;
import java.util.ArrayList;

/////////////// Assumptions ///////////////
//
// (no)---------------------------------------------------------------------------------------------all ingredients at  : location_0
//            table 1  : location_1
//            table 2  : location_2
//            table 3  : location_3
//           delivery  : location_4
//           cooktop   : location_5
//           deep frier : ?????
//
///////////////////////////////////////////


///////////////    Notes    ///////////////
//
//  1. We're going to need to figure out what we're doing with "containers" and "areas" during cooking.
//      - e.g. "get the beef on the frier pan". Is this the same thing as "get the bread on table 1"?
//      - This is an issue for domain complexity, etc. For now I'm assuming that we're just working with "locations"
//  2. Where are we assuming these "frier pan"/"oven" locations are bound to? How does that relate to the 2 stationary robots?
//  3. How are we going to restrict actions to only being for certain robots?
//      - capabilities (types?)
//      - location     (preconditions?)
//
///////////////////////////////////////////


() = prepareitemhamburger["instructed 'prepare item' script for hamburger"](Symbol ?refId:physobj, Symbol ?trayRefId:physobj) {

    Symbol !gofa = "gofa:gofa";

//todo: should we have preconditions all the way down validating the property of the refId that's getting passed in? really a domain validity sanity check, rather than something we're using right now for failure recovery, etc. but it could be useful?

    conditions : {
        pre : propertyof(?refId, hamburger:property);
        pre : propertyof(?trayRefId, tray:property);
    }
    effects : {
        success : prepared(?refId);
        success : itemOn(?refId, ?trayRefId);
    }

    Term !props;

    Symbol !bunRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(bun(X:physobj))");
    !bunRefId = act:positReference(!props);
    Symbol !bunBottomRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(bottomBun(X:physobj))");
    !bunBottomRefId = act:positReference(!props);
    Symbol !bunTopRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(topBun(X:physobj))");
    !bunTopRefId = act:positReference(!props);
    Symbol !pattyRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(beefPatty(X:physobj))");
    !pattyRefId = act:positReference(!props);
    Symbol !lettuceRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(lettuce(X:physobj))");
    !lettuceRefId = act:positReference(!props);
    Symbol !tomatoRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tomato(X:physobj))");
    !tomatoRefId = act:positReference(!props);
    Symbol !ketchupRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(ketchup(X:physobj))");
    !ketchupRefId = act:positReference(!props);
    Symbol !mustardRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(mustard(X:physobj))");
    !mustardRefId = act:positReference(!props);

    Predicate !tmp;

    act:getTo(?trayRefId, pose_3:pose);

    act:splitBun(!bunRefId);
    act:getOn(!bunBottomRefId, ?trayRefId);

    act:getOn(!lettuceRefId, !bunBottomRefId);

    act:getOn(!tomatoRefId, !bunBottomRefId);

    act:getTo(!pattyRefId, pose_0:pose);
    !gofa.act:cook(!pattyRefId, 2, 30);

    act:getOn(!pattyRefId, !tomatoRefId);

    act:getOn(!mustardRefId, !pattyRefId);

    act:getOn(!ketchupRefId, !pattyRefId);

    act:getOn(!bunTopRefId, !pattyRefId);

    //invalidate all of the references for the ingredients. they have been wrapped in the burger.
    act:invalidateReference(!bunRefId);
    act:invalidateReference(!bunBottomRefId);
    act:invalidateReference(!bunTopRefId);
    act:invalidateReference(!pattyRefId);
    act:invalidateReference(!lettuceRefId);
    act:invalidateReference(!tomatoRefId);
    act:invalidateReference(!ketchupRefId);
    act:invalidateReference(!mustardRefId);
}

() = prepareitemfrenchfries(Symbol ?refId:physobj, Symbol ?trayRefId:physobj) {

    Symbol !gofa = "gofa:gofa";

    conditions : {
    //todo: should this be configurable?
        pre : propertyof(?refId, frenchFries:property);
        pre : propertyof(?trayRefId, tray:property);
    }
    effects : {
        success : prepared(?refId);
        success : itemOn(?refId, ?trayRefId);
    }

    Term !props;

    Symbol !boxRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(friesBox(X:physobj))");
    !boxRefId = act:positReference(!props);
    Symbol !potatoesRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(potatoes(X:physobj))");
    !friesRefId = act:positReference(!props);

    act:getAt(?trayRefId, pose_3:pose);

    act:getOn(!boxRefId, ?trayRefId);

    act:getAt(!potatoesRefId, pose_1:pose);
    !gofa.act:fry(!friesRefId, 90);
    act:getOn(!friesRefId, !boxRefId);

    act:invalidateReference(!boxRefId);
    act:invalidateReference(!friesRefId);
}

//prepare a type of item: drink
//() = preparedrink(Symbol ?drinkRefId:physobj, Symbol ?trayRefId:physobj) {
//    conditions : {
//        pre : propertyof(?trayRefId, tray:property);
//        pre : propertyof(?drinkRefId, coke:property);
//    }
//    effects : {
//        success : prepared(?drinkRefId);
//        success : itemOn(?drinkRefId, ?trayRefId);
//    }
//
//    act:getDrinkFromCooler(?drinkRefId);
//}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  PIZZA                                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////

() = prepareitempizza(Symbol ?refId:physobj, List ?toppingTypes, Symbol ?trayRefId:physobj) {
    conditions : {
        pre : propertyof(?refId, pizza:property);
        pre : propertyof(?trayRefId, tray:property);
    }
    effects : {
        success : prepared(?refId);
        success : itemOn(?refId, ?trayRefId);
    }

    //positing references
    Term !props;
    Symbol !toppingRef;

    Symbol !crustRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(crust(X:physobj))");
    !crustRefId = act:positReference(!props);
    Symbol !sauceRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tomatoSauce(X:physobj))");
    !sauceRefId= act:positReference(!props);
    Symbol !cheeseRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(cheese(X:physobj))");
    !cheeseRefId = act:positReference(!props);

    List !toppingRefs;
    !toppingRefs = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    foreach(!toppingType : ?toppingTypes) {
    //todo: how do we make sure that !toppingType is a valid type that we can find a location for in belief?
        !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(!toppingType(X:physobj))");
        !toppingRef = act:positReference(!props);
        op:add(!toppingRefs, !toppingRef);
    }

    //actually preparing
    //todo: dough types, "extend", and different locations for different types of dough
    op:log(debug, "[prepareitempizza] putting the crust !crustRefId on the tray ?trayRefId");
    act:getOn(!crustRefId, ?trayRefId);
    op:log(debug, "[prepareitempizza] putting the sauce !sauceRefId on the crust !crustRefId");
    act:getOn(!sauceRefId, !crustRefId);
    op:log(debug, "[prepareitempizza] putting the cheese !cheeseRefId on the crust !crustRefId");
    act:getOn(!cheeseRefId, !crustRefId);

    foreach(!toppingRef : !toppingRefs) {
    op:log(debug, "[prepareitempizza] putting the topping !toppingRef on the crust !crustRefId");
        act:getOn(!toppingRef, !crustRefId);
    }

    //destroying references

    act:invalidateReference(!crustRefId);
    act:invalidateReference(!sauceRefId);
    act:invalidateReference(!cheeseRefId);
    foreach(!toppingRef : !toppingRefs) {
        act:invalidateReference(!toppingRef);
    }
}

//todo: this feels like it should be parameterized by the meat and salsa type (closer to "pizza"? or "drink"?)
() = prepareitemtaco["instructed 'prepare item' script for (pork) taco with (salsa verde)"](Symbol ?refId:physobj, Symbol ?trayRefId:physobj) {

    Symbol !gofa = "gofa:gofa";

    conditions : {
        pre : propertyof(?refId, taco:property);
        pre : propertyof(?trayRefId, tray:property);
    }
    effects : {
        success : prepared(?refId);
        success : itemOn(?refId, ?trayRefId);
    }

    Term !props;

    Symbol !tortillaRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(tortilla(X:physobj))");
    !tortillaRefId = act:positReference(!props);
    Symbol !meatRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(pork(X:physobj))");
    !meatRefId = act:positReference(!props);
    Symbol !salsaRefId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(salsaVerde(X:physobj))");
    !salsaRefId = act:positReference(!props);

    Predicate !tmp;

    act:getOn(!tortillaRefId, ?trayRefId);

    op:log(debug, "[prepareTacoItem] preparing taco mea");
    act:getTo(!meatRefId, pose_0:pose);
    op:log(warn, "[prepareTacoItem] cooking the meat");
    !gofa.act:cookmeat(!meatRefId, 2);
    op:log(debug, "[prepareTacoItem] putting the meat on the tortilla");
    act:getOn(!meatRefId, !tortillaRefId);

    op:log(debug, "[prepareTacoItem] preparing the salsa");
    act:getOn(!salsaRefId, !tortillaRefId);

    //invalidate all of the references for the ingredients. they have been wrapped in the burger.
    op:log(debug, "[prepareTacoItem] invalidating references for ingredients used in assembling hamburger ?refId");
    act:invalidateReference(!tortillaRefId);
    act:invalidateReference(!meatRefId);
    act:invalidateReference(!salsaRefId);
}
