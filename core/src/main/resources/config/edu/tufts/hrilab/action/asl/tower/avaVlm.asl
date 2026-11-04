import java.lang.String;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;

() = countPeople () {
  String !count;
  String !noPeopleMessage = "I do not see any people";
  String !personMessage = "I see one person";
  String !peopleMessage;

  !count = act:vlmCountPeople();

  if (op:invokeMethod(!count, "equals", "0")) {
    act:generateResponseFromString(!noPeopleMessage);
  } elseif (op:invokeMethod(!count, "equals", "1")) {
    act:generateResponseFromString(!personMessage);
  } else {
    !peopleMessage = op:newObject("java.lang.String", "I see !count people");
    act:generateResponseFromString(!peopleMessage);
  }
}

() = personInFront () {
  java.lang.Boolean !isPerson;
  String !noPersonMessage = "I do not see anyone in front of me";
  String !personMessage = "I see someone in front of me";

  !isPerson = act:vlmPersonInFront();

  if (op:==(!isPerson, true)) {
    act:generateResponseFromString(!personMessage);
  } else {
    act:generateResponseFromString(!noPersonMessage);
  }
}

() = describe () {
  String !description = act:vlmDescribe();
  act:generateResponseFromString(!description);
}

(String ?count) = vlmCountPeople() {
  op:log("info", "vlmCountPeople()");
  String !message = "You are a robot dedicated to counting the number of people in an image. It is essential that you only respond with a single integer value representing the number of people in this image.";
  Prompt !prompt = op:newObject("edu.tufts.hrilab.llm.Prompt", !message);
  Completion !res = act:visionCompletion(!prompt);
  ?count = op:invokeMethod(!res, "getText");
  op:log("info", "VLM responded: ?count");
}

(java.lang.Boolean ?isPerson) = vlmPersonInFront() {
  op:log("info", "vlmPersonInFront()");
  String !message = "You are a robot dedicated to detecting people in proximity to the camera. Please respond either yes or no to the following question: is there one or more people in this image facing the camera?";
  Prompt !prompt = op:newObject("edu.tufts.hrilab.llm.Prompt", !message);
  Completion !res = act:visionCompletion(!prompt);
  String !text = op:invokeMethod(!res, "getText");
  String !lower = op:invokeMethod(!text, "toLowerCase");
  op:log("info", "VLM responded: !text");
  if (op:invokeMethod(!lower, "contains", "yes")) {
    ?isPerson = op:newObject("java.lang.Boolean", true);
  } else {
    ?isPerson = op:newObject("java.lang.Boolean", false);
  }
}

(String ?description) = vlmDescribe () {
  op:log("info", "vlmDescribe()");
  String !message = "You are a robot dedicated to describing what you see. Please describe this image as accurately and matter-of-factly as you can. Be brief but descriptive. Use the first person as if you are seeing what is in the image with your own eyes.";
  Prompt !prompt = op:newObject("edu.tufts.hrilab.llm.Prompt", !message);
  Completion !res = act:visionCompletion(!prompt);
  ?description = op:invokeMethod(!res, "getText");
  op:log("info", "VLM responded: ?description");
}