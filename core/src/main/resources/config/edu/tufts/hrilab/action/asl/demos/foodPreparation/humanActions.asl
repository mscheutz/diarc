import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

() = getingredient["Equivalent action to squashing (optional gotolocation)->perceiveitem->pickup->gotolocation->putitematpose"](Symbol ?actor:mobileyumi,
                                 //Symbol ?current:location, Symbol ?destination:location,
                                 Symbol ?current:location,

                                 Symbol ?refId:physobj, Symbol ?itemType:property, Symbol ?perceptionPose:pose, Symbol ?perceptionLocation:location,

                                 //Symbol ?item:physobj, Symbol ?pickupPose:pose, Symbol ?equivalentPose:pose, Symbol ?pickupLocation:location,
                                 Symbol ?pickupPose:pose, Symbol ?equivalentPose:pose, Symbol ?pickupLocation:location,

                                 //Symbol ?item:physobj, Symbol ?pose:pose, Symbol ?location:location) {
                                 Symbol ?putdownPose:pose, Symbol ?putdownLocation:location) {

    conditions : {
        pre : agentAt(?actor, ?current);

        //pre : agentAt(?actor, ?perceptionLocation);
        pre : propertyof(?refId, ?itemType);
        pre : observableAt(?itemType,?perceptionPose); //TODO:brad: is this in Belief or the consultant?
        pre : not(itemAt(?refId, ?perceptionPose));
        pre : reachable(?actor, ?perceptionPose, ?perceptionLocation);
        pre : not(beenperceived(?refId));

        pre : free(?actor);
        pre : overlapping(?pickupPose, ?equivalentPose);
        //pre : itemAt(?item, ?equivalentPose);
        pre : reachable(?actor, ?pickupPose, ?pickupLocation);
        //pre : agentAt(?actor, ?pickupLocation);

        //pre : holding(?actor, ?item);
        //pre : agentAt(?actor, ?location);
        pre : reachable(?actor, ?putdownPose, ?putdownLocation);
    }
    effects : {
        success : not(agentAt(?actor, ?current));
        //success : agentAt(?actor, ?destination);
        success : agentAt(?actor, ?putdownLocation);

        //success : itemAt(?refId, ?pose);
        success : beenperceived(?refId);

        //success : not(free(?actor));
        //success : holding(?actor, ?item);
        //success : not(itemAt(?item, ?equivalentPose));

        //success : itemAt(?item, ?pose);
        success : itemAt(?refId, ?putdownPose);
        //success : not(holding(?actor, ?item));
        success : not(holding(?actor, ?refId));
        //success : free(?actor);
    }

    java.util.Map !bindings;
    Variable !x = "X";
    Predicate !response;

    java.lang.String !questionString;

    !questionString = act:getQuestionFromRefs(?itemType, ?perceptionPose, ?putdownPose);

    !bindings = act:askQuestionFromString(?actor, !questionString, got(X));
    !response = op:get(!bindings, !x);

    //act:goToLocation(?perceptionLocation);
//    act:perceiveEntityFromSymbol(?refId);
    //act:pickupItem(?refId);
    //act:goToLocation(?putdownLocation);
    //act:putDownItem(?refId);

}