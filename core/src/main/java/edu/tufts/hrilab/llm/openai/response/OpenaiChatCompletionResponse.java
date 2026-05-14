/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

public class OpenaiChatCompletionResponse {
  public String id = null;
  public long created = -1;
  public String object = null;
  public String model = null;
  public OpenaiChatCompletionChoice[] choices = null;
  public OpenaiUsage usage = null;

  public String toString () {
    if (choices.length > 0) {
      return choices[0].message.content.toString();
    } else {
      return null;
    }
  }
}