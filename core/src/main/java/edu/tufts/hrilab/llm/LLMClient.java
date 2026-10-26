/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import edu.tufts.hrilab.llm.message.Message;

import java.util.List;

public interface LLMClient {
  void setEndpoint(String endpoint);

  void setModel(String model);

  void setStopWords(String[] stopWords);

  void setTemperature(float temperature);

//  void setToolEnabled(boolean supported);

  Completion completion(String userPrompt);

  Completion completion(String userPrompt, String model);


  Completion completion(String userPrompt, String model, String systemPrompt);

  Completion chatCompletion(String userPrompt);

  Completion chatCompletion(String userPrompt, String model);

  Completion chatCompletion(String userPrompt, String model, String systemPrompt);

  Completion chatCompletion(List<Message> messages);

  Completion chatCompletion(Chat chat);

  Completion chatCompletion(Chat chat, String model);
}
