import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

() = defineVisualSearch() {
    Variable !x = "X";
    Symbol !objectName = op:createSymbol("none");

    java.util.Map !bindings = act:askQuestionFromString(user, "How many parts does the object have?", val(X));
    Symbol !numParts = op:get(!bindings, !x);  //
    op:log("info", "Visual search has !numParts components.");

//    // collect information about each component
//    !numParts = op:invokeMethod(!partsPred, "get", 0);
//    for (!i=0; !i lt !numParts; !i ++) {
//      !bindings = act:askQuestionFromString("Is there shape information for part !i?", val(X));
//      Predicate !responsePred = op:get(!bindings, !x);
//      if (~op:equalsValue(!responsePred,no())) {
//        !objectName = op:get(!bindings, !x);
//      }
//    }
//
//    // optionally get a name for the object
//    !bindings = act:askQuestionFromString("Does the object have a name?", val(X));
//    Predicate !responsePred = op:get(!bindings, !x);
//    if (~op:equalsValue(!responsePred,no())) {
//      !objectName = op:get(!bindings, !x);
//    }

    // start model training
    act:generateResponseFromString("Ok. Starting to train the model now. I'll inform you when training is complete.");
    Predicate !learningPred =  op:createPredicate("modelOf(a,b)");
    java.util.List !learningBindings = op:newArrayList("java.util.Map");
    tsc:learn(!learningPred, !learningBindings);

    // notify user that training is done
    act:generateResponseFromString("Training is complete.");

}