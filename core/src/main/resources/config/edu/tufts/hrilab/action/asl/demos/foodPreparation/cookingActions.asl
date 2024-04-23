import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Integer;
import java.lang.String;
import java.lang.Boolean;

() = cook(Symbol ?refId:physobj, Term ?seconds) {

    Symbol !cooktop; //= X:pose;
    Symbol !location; //= Y:location;

    conditions : {
        pre : itemAt(?refId, !cooktop);
        pre : reachable(?actor, !cooktop, !location);
//        pre obs : propertyof(!cooktop,cooktop:property);
        pre : agentAt(?actor, !location);
    }
    effects : {
        success : cooked(?refId);
    }

    op:log(debug, "[cookpatty] ?actor cooking patty ?refId at !cooktop, !location");
    act:cookItem(?refId, ?seconds);
    //act:cookForDuration(?refId, ?flips, ?seconds);
}

() = drizzle(Symbol ?refId:physobj) {

    String !sauceTwoPoseName = "sauceAreaTwo"; //= X:pose;
    String !sauceOnePoseName = "sauceAreaOne"; //= X:pose;
    String !servingAreaPoseName = "servingAreaLeft"; //= X:pose;
    String !carryPoseName = "carry"; //= X:pose;

    if (?actor.act:isAioli(?refId)) {
        ?actor.act:goToPoseName(!sauceTwoPoseName);
        ?actor.act:perceiveEntityFromSymbol(?refId);
        ?actor.act:pickupItem(?refId);
        ?actor.act:goToCameraPose(leftpose_5:leftpose);
        ?actor.act:pourSauce();
        ?actor.act:goToPoseName(!carryPoseName);
        ?actor.act:putDownItem(?refId, leftpose_3:leftpose);
    } else {
        ?actor.act:goToPoseName(!sauceOnePoseName);
        ?actor.act:perceiveEntityFromSymbol(?refId);
        ?actor.act:pickupItem(?refId);
        ?actor.act:goToCameraPose(leftpose_5:leftpose);
        ?actor.act:pourSauce();
        ?actor.act:goToPoseName(!carryPoseName);
        ?actor.act:putDownItem(?refId, leftpose_2:leftpose);
    }
}

() = fry(Symbol ?refId:physobj, Term ?time) {
    Symbol !frier;
    Symbol !location;

    conditions : {
        pre : agentAt(?actor, !location);
        pre : itemAt(?refId, !frier);
        pre : reachable(?actor, !frier, !location);
//        pre obs : propertyof(?refId, beefPatty:property);
    }
    effects : {
        success : fried(?refId);
    }

    act:fryForDuration(?refId, ?time); //assuming fryer is at table 1? todo: pete
}

//() = getDrinkFromCooler(Symbol ?refId:physobj) {
////todo: where are drinks located?
//    op:log(info, "getting drink from cooler");
//}

() = saute(Symbol ?refId:physobj, Term ?seconds) {

    Symbol !cooktop; //= X:pose;
    Symbol !location; //= Y:location;

    conditions : {
        pre : itemAt(?refId, !cooktop);
        pre : reachable(?actor, !cooktop, !location);
//        pre obs : propertyof(!cooktop,cooktop:property);
        pre : agentAt(?actor, !location);
    }
    effects : {
        success : cooked(?refId);
    }

    op:log(debug, "[saute] ?actor saute ?refId at !cooktop, !location");
    act:sauteItem(?refId, ?seconds);
}
