import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

() = lookForObjects() {

  boolean !result = act:searchLocation(location_2:location);
  if (op:equals(!result, false)) {
    act:generateResponseFromString("Searching for other parts");
    !result = act:searchLocation(location_3:location);
  }
  if (op:equals(!result, false)) {
    act:generateResponseFromString("Searching for other parts");
    !result = act:searchLocation(location_4:location);
  }

  // no more parts to explore
  if (op:equals(!result, false)) {
    act:generateResponseFromString("I do not see any other parts, return to assembly area");
    act:goToLocation(location_1:location);
  }

  op:log("info", "Exiting lookForObjects action");
}

(boolean ?result) = searchLocation(Symbol ?locRef) {
  Variable !x = op:createVariable("X");
  Predicate !obsPred = op:createPredicate("see(?actor,!x)");
  java.util.Map !objBinding;
  Symbol !objectRef;

  act:goToLocation(?locRef);
  act:goToPose(detection);
  act:openGripper(arm);

  java.util.List !bindings = obs:!obsPred;
  op:log("info", "Got bindings from observation: !bindings");
  act:generateResponse(person, !obsPred, !bindings, direct);

  java.util.Map !responseBindings = act:askQuestionFromString(person, "Do you want me to bring it to you?", val(X));
  Predicate !response = op:get(!responseBindings, !x);
  if (op:equalsValue(!response, yes())) {
    !objBinding = op:get(!bindings, 0);
    !objectRef = op:get(!objBinding, !x);
    act:bringObjectToPerson(!objectRef);
    ?result = op:newObject("java.lang.Boolean", true);
  } else {
    ?result = op:newObject("java.lang.Boolean", false);
  }
}

() = bringObjectToPerson(Symbol ?objectRef) {
  act:pickUp(?objectRef);
  act:goToLocation(location_1:location);
  act:askQuestionFromString(person, "Say ready when you are ready to grab the object.", val(X));
  act:openGripper(arm);
}