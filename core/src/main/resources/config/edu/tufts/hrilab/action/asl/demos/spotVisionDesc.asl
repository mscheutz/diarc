import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

() = lookForObjects() {

  boolean !result = act:searchLocation(location_2:location);
//  if (!result) {
//    act:generateResponse(person,"Searching for other parts");
//    !result = act:searchLocation(location_3:location);
//  }
//  if (!result) {
//    act:generateResponse(person,"Searching for other parts");
//    !result = act:searchLocation(location_4:location);
//  }
//
//  // no more parts to explore
//  if (!result) {
//    act:generateResponse(person,"I do not see any other parts, return to assembly area");
//    act:goToLocation(location_1:location);
//  }
}

(boolean ?result) = searchLocation(Symbol ?locRef) {
  Variable !x = op:createVariable("X");
  Predicate !obsPred = op:createPredicate("sees(?actor,!x)");

  act:goToLocation(?locRef);
  act:goToPose(detection);
  act:openGripper(arm);

  obs:!obsPred; // EAK: does this return the return value?
  //java.util.List !bindings = act:observeScene(!obsPred);
  act:generateResponse(person, !obsPred, !bindings, direct);

  java.util.Map !responseBindings = act:askQuestionFromString(person, "Do you want me to bring it to you?", val(X));
  Predicate !response = op:get(!responseBindings, !x);
  if (op:equalsValue(!response, yes())) {
    act:bringObjectToPerson();
    ?result = op:newObject("java.lang.Boolean", true);
  } else {
    ?result = op:newObject("java.lang.Boolean", false);
  }
}

() = bringObjectToPerson(Symbol ?objectRef) {
  act:pickUp(?objectRef);
  act:goToLocation(location_1:location);
  act:askQuestionFromString(person, "Say okay when you are ready to grab the object.", val(X));
  act:openGripper(arm);
}