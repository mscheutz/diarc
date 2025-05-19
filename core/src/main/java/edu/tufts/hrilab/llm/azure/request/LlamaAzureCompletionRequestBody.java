/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.azure.request;

import java.util.HashMap;
import java.util.Map;

import edu.tufts.hrilab.llm.LlamaHFChat;

public class LlamaAzureCompletionRequestBody {
  public String messages;
  public Map<String,Object> params;

  private void init() {
    params = new HashMap<>();
    params.put("temperature", 0.6f);
    params.put("top_p", 0.9f);
    params.put("top_k", 50);
    params.put("repetition_penalty", 1.0f);
    params.put("max_new_tokens", 100);
    params.put("return_full_text", false);
    params.put("max_time", 15);
  }

  public LlamaAzureCompletionRequestBody () {
    init();
  }

  public LlamaAzureCompletionRequestBody (String p) {
    init();
    messages = p;
  }

  public LlamaAzureCompletionRequestBody (LlamaHFChat chat) {
    init();
    messages = chat.toPromptString();
  }

  public void setTemperature(float temperature) {
    params.put("temperature", temperature);
  }

  public void setTop_k(int top_k) {
    params.put("top_k", top_k);
  }

  public void setTop_p(float top_p) {
    params.put("top_p", top_p);
  }

  public void setRepetition_penalty(float repetition_penalty) {
    params.put("repetition_penalty ",repetition_penalty);
  }

  public void setReturn_full_text(boolean return_full_text) {
    params.put("return_full_text", return_full_text);
  }

  public void setNum_return_sequences(int num_return_sequences) {
    params.put("num_return_sequences", num_return_sequences);
  }

  public void setMax_time(float max_time) {
    params.put("max_time", max_time);
  }

  public void setMax_new_tokens(int max_new_tokens) {
    params.put("max_new_tokens", max_new_tokens);
  }
}

