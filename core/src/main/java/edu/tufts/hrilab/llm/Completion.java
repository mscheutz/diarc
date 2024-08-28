/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.UUID;
import java.util.ArrayList;
import java.util.List;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tufts.hrilab.llm.hf.response.LlamaHFChatCompletionResponse;
import edu.tufts.hrilab.llm.hf.response.T5BaseCompletionResponse;

import edu.tufts.hrilab.llm.openai.response.*;
import edu.tufts.hrilab.llm.llama.response.*;

public class Completion {
  public String text;
  public String id;
  public long created;
  public String service;
  public String model;
  public String type;
  public int inputTokens;
  public int outputTokens;

  public Completion (OpenaiCompletionResponse response) {
    text = response.choices[0].text;
    id = response.id;
    created = response.created;
    service = "openai";
    model = response.model;
    type = "completion";
    inputTokens = response.usage.prompt_tokens;
    outputTokens = response.usage.completion_tokens;
  }

  public Completion (OpenaiChatCompletionResponse response) {
    if (response.choices.length>0) {
      text = response.choices[0].message.content;
    } else {
      text = "none";
    }
    id = response.id;
    created = response.created;
    service = "openai";
    model = response.model;
    type = "chat";
    inputTokens = response.usage.prompt_tokens;
    outputTokens = response.usage.completion_tokens;
  }

  public Completion (LlamaCompletionResponse response) {
    UUID uuid = UUID.randomUUID();
    text = response.content;
    id = uuid.toString();
    created = System.currentTimeMillis() / 1000;
    service = "llama";
    model = response.model;
    type = "completion";
    inputTokens = response.tokens_evaluated;
    outputTokens = response.tokens_predicted;
  }

  public Completion (LlamaHFChatCompletionResponse response) {
    UUID uuid = UUID.randomUUID();
    text = response.generated_text;
    id = uuid.toString();
    created = System.currentTimeMillis() / 1000;
    service = "llamahf";
    type = "chat";
  }

  public Completion (T5BaseCompletionResponse response) {
    UUID uuid = UUID.randomUUID();
    text = response.translation_text;
    id = uuid.toString();
    created = System.currentTimeMillis() / 1000;
    service = "t5hf";
    type = "";
  }

  public String getText () {
    return text;
  }

  public String toString () {
    return text;
  }

  public String getId () {
    return id;
  }

  public String getModel () {
    return model;
  }

  public String getService () {
    return service;
  }

  public String getType () {
    return type;
  }

  public long getCreated () {
    return created;
  }

  public int getInputTokens () {
    return inputTokens;
  }

  public int getOutputTokens () {
    return outputTokens;
  }

  public List<String> getCode () {
    return extractCodeFromText(text);
  }

  public boolean hasCode () {
    List<String> code = getCode();
    return !code.isEmpty();
  }

  /**
   * Returns a string array of code blocks within a string
   * that are contained within markdown triple tick blocks.
   * 
   * @param text Response text to extract code from
   * @return Array of code strings
   */
  public static List<String> extractCodeFromText (String text) {
    Pattern pattern = Pattern.compile("```([\\s\\S]*?)```");
    Matcher matcher = pattern.matcher(text);
    List<String> codeBlocks = new ArrayList<>();
    while (matcher.find()) {
      String match = matcher.group(1);
      codeBlocks.add(match);
    }
    return codeBlocks;
  }
}
