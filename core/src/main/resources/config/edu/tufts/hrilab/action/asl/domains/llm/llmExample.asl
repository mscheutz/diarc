import java.lang.String;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;


//["LLM Example"]
() = llmExample() {
  op:log("info", "llmExample()");
  String !prompt = "Create a javascript function that takes a string and returns it in reverse character order.";
  //String !model = "text-davinci-003";
  Completion !res = act:chatCompletion(!prompt);
  String !text = op:invokeMethod(!res, "getText");
  java.util.List !codeList = op:invokeMethod(!res, "getCode");
  String !code;

  if (~op:isEmpty(!codeList)) {
    !code = op:invokeMethod(!codeList, "get", 0);
    op:log("info", "Got code");
    op:log("info", !code);
  } else {
    op:log("info", "Did not get code");
    op:log("info", !text);
  }
}

() = llmExampleLlama() {
  op:log("info", "llmExampleLlama()");
  String !message = "[Human] Hello, how are you? [Robot] I'm fine, how are you? [Human] Very well, thank you. Can you fetch a screw? [Robot]";
  Prompt !prompt = op:newObject("edu.tufts.hrilab.llm.Prompt", !message);
  Completion !res = act:completion(!prompt);
  String !text = op:invokeMethod(!res, "getText");

  op:log("info", "Prompt:");
  op:log("info", !prompt);
  op:log("info", "Response:");
  op:log("info", !text);
}

() = llmExampleLlamaChat() {
  op:log("info", "llmExampleLlamaChat()");
  String !prompt = "Create a javascript function that takes a string and returns it in reverse character order.";
  Completion !res = act:chatCompletion(!prompt);
  String !text = op:invokeMethod(!res, "getText");
  java.util.List !codeList = op:invokeMethod(!res, "getCode");
  String !code;

  if (~op:isEmpty(!codeList)) {
    !code = op:invokeMethod(!codeList, "get", 0);
    op:log("info", "Got code");
    op:log("info", !code);
  } else {
    op:log("info", "Did not get code");
    op:log("info", !text);
  }
}

() = llmExampleCodeParse() {
  op:log("info", "llmExampleCodeParse()");
  String !text = "This is an example with a ```code block```.";
  java.util.List !codeList = op:invokeStaticMethod("edu.tufts.hrilab.llm.Completion", "extractCodeFromText", !text);
  String !code;

  if (~op:isEmpty(!codeList)) {
    !code = op:invokeMethod(!codeList, "get", 0);
    op:log("info", "Got code");
    op:log("info", !code);
  } else {
    op:log("info", "Did not get code");
    op:log("info", !text);
  }
}

() = initializeSpaceStation () {
  op:log("info", "initializeSpaceStation()");
  String !message = "Robot, please go to alpha";
  Prompt !prompt = op:invokeStaticMethod("edu.tufts.hrilab.llm.Prompts", "getActionPrompt");
  //runtime error here
  Chat !chat = op:newObject("edu.tufts.hrilab.llm.Chat", !prompt);
  Completion !response;

  op:invokeMethod(!chat, "addUserMessage", !message);
  !response = act:chatCompletion(!chat);
  act:toGoal(!response);
}