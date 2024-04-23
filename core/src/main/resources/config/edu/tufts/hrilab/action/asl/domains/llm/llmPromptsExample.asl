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

//["Prompts example"]
() = promptsExample() {
	Boolean !updateTokens = true;

	Prompt !dialogSystem = op:invokeStaticMethod("edu.tufts.hrilab.llm.Prompts", "getPrompt", "dialogExample");

  Chat !interactionChat = op:newObject("edu.tufts.hrilab.llm.Chat");
	Chat !dialogChat = op:newObject("edu.tufts.hrilab.llm.Chat", !dialogSystem);

	Completion !response;

	String !additionalContext = "There is a basement three floors below where you currently are. It is accessible by staircase on the north end of the hall and an elevator on the south end.";

	String !messageOne = "How do I get to the basement?";
	String !messageTwo = "I can't use the stairs.";
	String !messageThree = "Thank you for your help.";
	String !messageFour = "Goodbye.";
	String !text;

	op:invokeMethod(!dialogChat, "addContext", !additionalContext);

	//message 1
	op:log("info", "Human: !messageOne");
	op:invokeMethod(!dialogChat, "addUserMessage", !messageOne);
	
	!response = act:chatCompletion(!dialogChat);
	!text = op:invokeMethod(!response, "getText");
	op:log("info", "Robot: !text");
	op:invokeMethod(!dialogChat, "addCompletion", !response, !updateTokens);
	op:log("info", "!dialogChat");
	op:invokeMethod(!interactionChat, "setMessages", !dialogChat);
	act:evaluateEmotion(!interactionChat);

	//message 2
	op:log("info", "Human: !messageTwo");
	op:invokeMethod(!dialogChat, "addUserMessage", !messageTwo);
	
	!response = act:chatCompletion(!dialogChat);
	!text = op:invokeMethod(!response, "getText");
	op:log("info", "Robot: !text");
	op:invokeMethod(!dialogChat, "addCompletion", !response, !updateTokens);

	op:invokeMethod(!interactionChat, "setMessages", !dialogChat);
	act:evaluateEmotion(!interactionChat);

	//message 3
	op:log("info", "Human: !messageThree");
	op:invokeMethod(!dialogChat, "addUserMessage", !messageThree);
		
	!response = act:chatCompletion(!dialogChat);
	!text = op:invokeMethod(!response, "getText");
	op:log("info", "Robot: !text");
	op:invokeMethod(!dialogChat, "addCompletion", !response, !updateTokens);

	op:invokeMethod(!interactionChat, "setMessages", !dialogChat);
	act:evaluateEmotion(!interactionChat);

	//message 4
	op:log("info", "Human: !messageFour");
	op:invokeMethod(!dialogChat, "addUserMessage", !messageFour);
		
	!response = act:chatCompletion(!dialogChat);
	!text = op:invokeMethod(!response, "getText");
	op:log("info", "Robot: !text");
	op:invokeMethod(!dialogChat, "addCompletion", !response, !updateTokens);

	op:invokeMethod(!interactionChat, "setMessages", !dialogChat);
	act:evaluateEmotion(!interactionChat);
}

() = evaluateEmotion (Chat ?interactionChat) {
  op:log("info", "---> Evaluating Emotion < ---");
	Prompt !emotionSystem = op:invokeStaticMethod("edu.tufts.hrilab.llm.Prompts", "getPrompt", "emotionExample");
	Chat !emotionChat = op:newObject("edu.tufts.hrilab.llm.Chat", !emotionSystem);
	String !emotionDescription = op:invokeMethod(?interactionChat, "toPromptString");
  Completion !emotionResponse;
  String !text;

	op:invokeMethod(!emotionChat, "addUserMessage", !emotionDescription);

	!emotionResponse = act:chatCompletion(!emotionChat);
	!text = op:invokeMethod(!emotionResponse, "getText");
	op:log("info", "!text");
}