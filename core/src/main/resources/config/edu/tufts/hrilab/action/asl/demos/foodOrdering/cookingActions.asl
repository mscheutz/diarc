import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.Integer;
import java.lang.String;
import java.lang.Boolean;

() = cook(Symbol ?refId:physobj, Term ?seconds, Symbol ?area:area, Symbol ?location:location, Symbol ?cooktopProperty:property) {

    conditions : {
        pre : itemAt(?refId, ?area);
        pre : agentAtLocation(?actor, ?location);
        pre : property_of(?area, ?cooktopProperty);
    }
    effects : {
        success : cooked(?refId);
    }

    act:cookItem(?refId, ?seconds);
}

() = cook(Symbol ?refId:physobj, Term ?seconds) {
    Predicate !queryPred;

    Symbol !agentLocation;
    Symbol !itemAtArea;
    Symbol !cooktopProperty;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(?actor, X)");
    !agentLocation = act:getBinding(!queryPred);
    op:log(debug, "[cook] agent location: !agentLocation");

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(?refId, X)");
    !itemAtArea = act:getBinding(!queryPred);
    op:log(debug, "[cook] item at area : !itemAtArea");

    !cooktopProperty = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "\"cooktop\"");

    act:generateResponseFromString("cooking");
    act:cook(?refId, ?seconds, !itemAtArea, !agentLocation, !cooktopProperty);

}

//todo: make this action use the new pipeline for going between locations. use "above", etc. "drizzled" should be an outcome on (sauce, item) pair.
() = drizzle(Symbol ?refId:physobj) {

    String !carryPoseName = "carry"; //= X:pose;

    ?actor.act:goToPoseName("\"camera pose sauce area\"");
    ?actor.act:perceiveEntityFromSymbol(?refId);
    ?actor.act:pickupItem(?refId);
    ?actor.act:gotosafepose();
    ?actor.act:goToPoseName("\"camera pose sauce pour\"");
    ?actor.act:pourSauce();
    ?actor.act:gotosafepose();
    ?actor.act:putDownItem(?refId, leftArmpose_7:leftArmpose);
    ?actor.act:gotosafepose();
}

() = saute(Symbol ?refId:physobj, Term ?seconds, Symbol ?area:area, Symbol ?location:location, Symbol ?hotplateProperty:property) {

    conditions : {
        pre : itemAt(?refId, ?area);
        pre : agentAtLocation(?actor, ?location);
        pre : property_of(?area, ?hotplateProperty);
    }
    effects : {
        success : cooked(?refId);
    }

    act:generateResponseFromString("sauteing");
    act:sauteItem(?refId, ?seconds);
}

() = saute(Symbol ?refId:physobj, Term ?seconds) {
    Predicate !queryPred;

    Symbol !agentLocation;
    Symbol !itemAtArea;
    Symbol !hotplateProperty;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(?actor, X)");
    !agentLocation = act:getBinding(!queryPred);
    op:log(debug, "[saute] agent location: !agentLocation");

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "itemAt(?refId, X)");
    !itemAtArea = act:getBinding(!queryPred);
    op:log(debug, "[saute] item at area : !itemAtArea");

    !hotplateProperty = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "\"hot plate\"");

    act:saute(?refId, ?seconds, !itemAtArea, !agentLocation, !hotplateProperty);

}
