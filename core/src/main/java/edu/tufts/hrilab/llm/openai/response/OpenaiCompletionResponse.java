/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

public class OpenaiCompletionResponse {
  public String id;
  public long created;
  public String object;
  public String model;
  public OpenaiCompletionChoice[] choices;
  public OpenaiUsage usage;
}