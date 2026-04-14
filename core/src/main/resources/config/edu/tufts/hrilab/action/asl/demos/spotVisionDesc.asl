import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

() = lookForObjects() {

  try {
    act:undock();
   } catch () {
     // ignore failure if robot is already undocked
   }

  boolean !result = act:searchLocation(location_2:location);
  if (op:equals(!result, false)) {
    !result = act:searchLocation(location_3:location);
  }
  if (op:equals(!result, false)) {
    !result = act:searchLocation(location_4:location);
  }

  // no more parts to explore
  if (op:equals(!result, false)) {
    act:generateResponseFromString("I do not see any other parts, returning to assembly area");
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

  op:sleep(1000); // wait for image from spot to catch up to new pose

  java.util.List !bindings = obs:!obsPred;
  if (op:isNull(!bindings)) {
    op:log("info", "Null bindings from observation: !obsPred");
    ?result = op:newObject("java.lang.Boolean", false);
    act:goToPose(stow);
    act:closeGripper(arm);
    return;
  } elseif (op:isEmpty(!bindings)) {
    op:log("info", "Empty bindings from observation: !obsPred");
    ?result = op:newObject("java.lang.Boolean", false);
    act:goToPose(stow);
    act:closeGripper(arm);
    return;
  }

  op:log("info", "Got bindings from observation: !bindings");
  act:generateResponse(person, !obsPred, !bindings, direct);

  java.util.Map !responseBindings = act:askQuestionFromString(person, "Do you want me to bring it to you?", val(X));
  Predicate !response = op:get(!responseBindings, !x);
  if (op:equalsValue(!response, yes())) {
    act:generateResponseFromString("okay");
    !objBinding = op:get(!bindings, 0);
    !objectRef = op:get(!objBinding, !x);
    act:bringObjectToPerson(!objectRef);
    ?result = op:newObject("java.lang.Boolean", true);
  } else {
    act:generateResponseFromString("Searching for other parts");
    act:goToPose(stow);
    act:closeGripper(arm);
    ?result = op:newObject("java.lang.Boolean", false);
  }
}

() = bringObjectToPerson(Symbol ?objectRef) {
  act:pickUp(?objectRef);
  act:goToPose(stow);
  act:goToLocation(location_1:location);

  act:goToPose(carry);
  act:askQuestionFromString(person, "Say ready when you are ready to grab the object.", val(X));
  op:sleep(1000);
  act:openGripper(arm);
  op:sleep(1000);
  act:closeGripper(arm);
  act:goToPose(stow);
}