/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.LLMServiceProvider;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.*;

public abstract class BasicLLMServiceProvider extends LLMServiceProvider {

  /**
   * Generate a Chat object to call LLM's chatCompletion service with for the generation task. Practically entails
   * generating a system and user message for an NLG request of the supplied utterance.
   * @param utterance outgoing utterance to perform NLG for
   * @return Fully updated Chat object to pass to an LLM's chatCompletion service for NLG
   */
  protected abstract Chat generateNLGChat(Utterance utterance);
  /**
   * Generate a Chat object to call LLM's chatCompletion service with for the parsing task. Practically entails
   * creating a new or updating an existing Chat object to contain the proper system, user, and assistant messages to
   * come up with a proper parse of the supplied utterance.
   * @param utterance incoming utterance to be parsed
   * @return Fully updated Chat object to pass to an LLM's chatCompletion service for parsing
   */
  protected abstract Chat generateNLUChat(Utterance utterance);

  /**
   * Postprocessing of LLM's chatCompletion return value for the NLU task.
   * @return The fully populated Utterance to return from parseUtterance
   */
  protected abstract Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse);
  /**
   * Postprocessing of LLM's chatCompletion return value for the NLG task.
   * @return The fully populated Utterance to return convertSemanticsToText.
   */
  protected abstract Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse);

  public boolean inExtendedInteraction(Symbol user, Symbol agentTeam) {
    return false;
  }

  @TRADEService
  public Utterance parseUtteranceLLMService(Utterance utterance) {
    return llmChatCompletionHelper(utterance, true);
  }

  @TRADEService
  public Utterance convertSemanticsToTextLLMService(Utterance u) {
    return llmChatCompletionHelper(u, false);
  }

  protected Utterance llmChatCompletionHelper(Utterance utterance, boolean parsing) {
    //No extended interactions, generate chat from scratch each time
    Chat chat;
    if (parsing) {
      chat = generateNLUChat(utterance);
    } else {
      chat = generateNLGChat(utterance);
    }

    // Call LLM to get Completion
    Completion response = callChatEndpoint(chat);

    if (response == null ||response.getText() == null) {
      log.warn("[llmChatCompletionHelper] have null response from LLM, returning original utterance");
      return utterance;
    }

    // Call abstract method to transform Completion to Utterance (implementation dependent on prompt)
    if (parsing) {
      return parseNLUCompletionResponse(utterance, response);
    } else {
      return parseNLGCompletionResponse(utterance, response);
    }
  }
}
