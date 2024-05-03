import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

() = getingredient(Symbol ?actor:mobileManipulator, Symbol ?refId:physobj, Symbol ?itemType:property,
                   Symbol ?perceptionArea:area, Symbol ?perceptionLocation:location,
                   Symbol ?putdownLocation:location, Symbol ?putdownArea:area) {

    conditions : {
        pre : reachable(?actor, ?perceptionArea, ?perceptionLocation);
        pre : property_of(?refId, ?itemType);
        pre : observableAt(?itemType,?perceptionArea);
        pre : not(beenperceived(?refId));
        pre : free(?actor);
        pre : reachable(?actor, ?putdownArea, ?putdownLocation);
    }
    effects : {
        success : beenperceived(?refId);
        success : itemAt(?refId, ?putdownArea);
    }

    java.util.Map !bindings;
    Variable !x = "X";
    Predicate !response;

    java.lang.String !questionString;

    !questionString = act:getQuestionFromRefs(?itemType, ?perceptionArea, ?putdownArea);

    !bindings = act:askQuestionFromString(?actor, !questionString, got(X));
    !response = op:get(!bindings, !x);
}