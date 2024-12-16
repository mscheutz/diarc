/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station.llm;

import java.util.regex.Pattern;
import java.util.regex.Matcher;
import java.util.List;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.slug.parsing.llm.AlternateResponse;

public class UnitySpaceStationLLM extends DiarcComponent {
  Pattern agentPattern = Pattern.compile("agent:");
  Matcher agentMatcher = agentPattern.matcher("");

  @TRADEService
  public void LLMListenSpaceStationFinetune(String message) {
    log.warn("Deprecated for new NLU pipeline");
    /*
    Prompt prompt = Prompts.getPrompt("spaceStationFinetunedLlama");
    Chat chat = new Chat(prompt);
    Object oResponse = null;
    String jsonString = "";
    LlamaSpaceStationResponse lssRes = null;
    String predicateString = "";
    String dialogGoalString = "";
    String actor = getAgent();

    chat.addUserMessage(message);

    try {
      oResponse = TRADE.callThe("chatCompletion", chat);
    } catch (TRADEException e) {
      log.error("Error performing chat completion", e);
    }

    Completion response = (Completion) oResponse;
    jsonString = response.toString();

    if (!jsonString.contains("}")) {
      jsonString += "}";
    }

    try {
      lssRes = LlamaSpaceStationResponse.fromString(jsonString);
    } catch (Exception e) {
      log.error("Error parsing response: ", e);
    }

    if (lssRes != null) {
      predicateString = lssRes.toPredicate(actor);
      dialogGoalString = lssRes.toDialogGoal(actor, predicateString);
      log.info(dialogGoalString);
      Predicate goal = Factory.createPredicate(dialogGoalString);

      try {
        TRADE.callThe("submitGoal", goal);
      } catch (TRADEException e) {
        log.error("Error submitting goal.", e);
      }
    }
    */
  }

  @TRADEService
  public AlternateResponse spaceStationLLMParser(String utterance) {
    Prompt prompt = Prompts.getPrompt("spaceStationFinetunedLlama");
    Chat chat = new Chat(prompt);
    Completion oResponse = null;
    String jsonString = "";
    LlamaSpaceStationResponse lssRes = null;

    chat.addUserMessage(utterance);

    try {
      oResponse = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
    } catch (TRADEException e) {
      log.error("Error performing chat completion", e);
    }

    Completion response = oResponse;
    jsonString = response.toString();

    if (!jsonString.contains("}")) {
      jsonString += "}";
    }
    log.debug("LLM response object: " + jsonString);
    try {
      lssRes = LlamaSpaceStationResponse.fromString(jsonString);
    } catch (Exception e) {
      log.error("Error parsing response: ", e);
    }

    return new AlternateResponse(lssRes.toParserResponse(), lssRes.toAddressee());
  }

  private String getAgent() {
    List<String> groups = getMyGroups();
    for (String group : groups) {
      agentMatcher.reset(group);
      if (agentMatcher.find()) {
        return group.split(":")[1];
      }
    }
    return "robot1";
  }

  @TRADEService
  @Action
  public void stylizeUnitySpeak(Symbol original) {
    Prompt prompt = Prompts.getPrompt("spaceStationSpeechStyle");
    Symbol model = Factory.createSymbol("gpt-3.5-turbo");
    Symbol actor = Factory.createSymbol("robot1");
    Chat chat = new Chat(prompt);
    Completion oResponse = null;

    chat.addUserMessage(original.toString());

    try {
      oResponse = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Symbol.class, Chat.class)).call(Completion.class, model, chat);
    } catch (TRADEException e) {
      log.error("Error getting a completion.", e);
    }

    Completion response = oResponse;
    String strResponse = response.toString().trim();
    Symbol statement = Factory.createSymbol(strResponse);

    log.info("Response: " + strResponse);

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(Symbol.class, String.class)).call(void.class, actor, statement);
    } catch (TRADEException e) {
      log.error("Error calling sayText.", e);
    }
  }

}