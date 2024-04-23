import java.lang.String;
import java.lang.Boolean;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;

//Dummy actions

() = goTo (Symbol ?location) {
	op:log("info", "goTo(?location)");
}

() = goToPose(Symbol ?pose) {
	op:log("info", "goToPose(?pose)");
}

() = raise(Symbol ?arm) {
	op:log("info", "raise(?arm)");
}

() = lower(Symbol ?arm) {
	op:log("info", "lower(?arm)");
}

() = pickUp(Symbol ?object, Symbol ?arm) {
	op:log("info", "pickUp(?object, ?arm)");
}

() = placeOn(Symbol ?object, Symbol ?location, Symbol ?arm) {
	op:log("info", "placeOn(?object, ?location, ?arm)");
}

() = releaseObject(Symbol ?object, Symbol ?arm) {
	op:log("info", "releaseObject(?object, ?arm)");
}

() = moveObject(Symbol ?object, Symbol ?arm) {
	op:log("info", "moveObject(?object, ?arm)");
}

() = speak(Symbol ?message) {
	op:log("info", "speak(?message)");
}

//main

() = llmAction() {
  op:log("info", "llmAction()");

	String !additionalContext = "There is a basement three floors below where you currently are. It is accessible by staircase on the north end of the hall and an elevator on the south end.";
	String !instruction = "Go to the elevator.";
	Prompt !actionSystem;
  Chat !actionChat;

  !actionSystem = op:invokeStaticMethod("edu.tufts.hrilab.llm.Prompts", "getPrompt", "actionExample");
  !actionChat = op:newObject("edu.tufts.hrilab.llm.Chat", !actionSystem);

	op:invokeMethod(!actionChat, "addContext", !additionalContext);
	op:invokeMethod(!actionChat, "addUserMessage", !instruction);

	op:log("info", "Human: !instruction");

	Completion !response = act:chatCompletion(!actionChat);
	String !text = op:invokeMethod(!response, "getText");
	Predicate !goal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !text);

	op:log("info", "Robot: !text");

	String !basement = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "basement:location");
	String !elevator = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "elevator:location");
	String !stairs = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "stairs:location");
	String !arm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "arm:appendage");
	String !cup = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "cup:object");

	goal:!goal;
}