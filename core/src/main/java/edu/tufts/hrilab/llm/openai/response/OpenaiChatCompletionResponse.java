/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

public class OpenaiChatCompletionResponse {
  public String id;
  public long created;
  public String object;
  public String model;
  public OpenaiChatCompletionChoice[] choices;
  public OpenaiUsage usage;

  public String toString () {
    if (choices.length > 0) {
      return choices[0].message.content;
    } else {
      return null;
    }
  }
}