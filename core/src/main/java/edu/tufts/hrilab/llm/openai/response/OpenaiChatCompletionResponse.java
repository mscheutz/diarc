/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

import java.util.Arrays;

public class OpenaiChatCompletionResponse {
  public String id = null;
  public long created = -1;
  public String object = null;
  public String model = null;
  public OpenaiChatCompletionChoice[] choices = null;
  public OpenaiUsage usage = null;

  @Override
  public String toString() {
    return "OpenaiChatCompletionResponse{" +
            "id='" + id + '\'' +
            ", created=" + created +
            ", object='" + object + '\'' +
            ", model='" + model + '\'' +
            ", choices=" + Arrays.toString(choices) +
            ", usage=" + usage +
            '}';
  }
}