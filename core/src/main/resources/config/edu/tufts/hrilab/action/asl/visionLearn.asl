import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.String;
import java.util.Map;
import java.util.List;

() = defineVisualSearch() {
    Variable !x = "X";
    Symbol !objectName = op:createSymbol("none");
    Predicate !responsePred;
    Symbol !shape;
    Predicate !descriptor;
    List !description = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    Predicate !learningPred;

    Map !bindings = act:askQuestionFromString(user, "How many parts does the object have?", val(X));
    Symbol !numParts = op:get(!bindings, !x);  //
    op:log("info", "Visual search has !numParts components.");
    !descriptor = op:createPredicate("components(!numParts)");
    op:invokeMethod(!description, "add", !descriptor);

    // collect information about each component
    String !numPartsStr = op:invokeMethod(!numParts, "getName");
    int !numPartsInt = op:invokeStaticMethod("java.lang.Integer", "valueOf", !numPartsStr);
    for (!i=1; !i leq !numPartsInt; !i ++) {
      !bindings = act:askQuestionFromString(user, "Is there shape information for part !i?", val(X));
      !responsePred = op:get(!bindings, !x);
      if (~op:equalsValue(!responsePred,no())) {
        !shape = op:get(!bindings, !x);
        !descriptor = op:createPredicate("shape(!i,!shape)");
        op:invokeMethod(!description, "add", !descriptor);
      }
    }

    // optionally get a name for the object
    !bindings = act:askQuestionFromString(user, "Does the object have a name?", val(X));
    !responsePred = op:get(!bindings, !x);
    if (~op:equalsValue(!responsePred,no())) {
      !objectName = op:get(!bindings, !x);
    }

    // create final learning predicate of form: modelOf(description, name)
    Predicate !descriptionConj = op:createPredicate("and", !description);
    !learningPred =  op:createPredicate("modelOf", !descriptionConj, !objectName);

    // start model training
    act:generateResponseFromString("Ok. Starting to train the model now. I'll inform you when training is complete.");
    java.util.List !learningBindings = op:newArrayList("java.util.Map");
    tsc:learn(!learningPred, !learningBindings);

    // notify user that training is done
    act:generateResponseFromString("Training is complete.");

}
