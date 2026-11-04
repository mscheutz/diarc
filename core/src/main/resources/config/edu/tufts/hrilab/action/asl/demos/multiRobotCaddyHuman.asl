import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.String;

() = putitematlocation["a human assistant puts ?objectRef at ?area"](Symbol ?actor:human, Symbol ?objectRef:physobj, Symbol ?propertyType:property, Symbol ?area:area) {
    conditions : {
        pre : property_of(?objectRef,?propertyType);
        pre : not(objectAt(?objectRef, ?area));
        pre : not(isContainer(?objectRef));
    }
    effects : {
        success obs : objectAt(?objectRef, ?area);
        success infer : fluent_increase(amount(?area,?propertyType), 1);
    }

    // TODO: how to know the diarc agent
    Symbol !diarcAgent = "self:agent";
    !diarcAgent.act:askForHelp(?objectRef, ?propertyType, ?area);

    op:log("info", "[putItemAtLocation] ?actor successfully put ?propertyType at ?area");
}
