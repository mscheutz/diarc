/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.request;

public class OpenaiCompletionRequestBody {
  public String model;
  public String prompt;
  public float temperature;
  public int max_tokens;

  public OpenaiCompletionRequestBody () {

  }

  public OpenaiCompletionRequestBody (String m, String p) {
    model = m;
    prompt = p;
  }
}