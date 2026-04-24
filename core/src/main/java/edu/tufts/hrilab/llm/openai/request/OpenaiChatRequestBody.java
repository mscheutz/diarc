/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.request;

import java.util.List;
import java.util.Map;

import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;

public class OpenaiChatRequestBody {
  public String model;
  public List<Message> messages;
  public List<ToolDefinition> tools = null;
  public Double temperature = null;
  public Double top_p = null;
  public Integer top_k = null;
  public Double min_p = null;
  public Double repetition_penalty = null;
  public Integer max_tokens;
  public Double frequency_penalty;
  public Double presence_penalty;
  public Boolean logprobs;
  public Integer top_logprobs;
  public Boolean stream;
  public String[] stop;
  public Integer seed;
  public String tool_choice;
  public String tool_prompt;
  public Map<String, Object> chat_template_kwargs = null;

  public OpenaiChatRequestBody () {

  }

  public OpenaiChatRequestBody (String m, List<Message> msgs) {
    model = m;
    messages = msgs;
  }

  public OpenaiChatRequestBody (String m, List<Message> msgs, List<ToolDefinition> ts) {
    model = m;
    messages = msgs;
    tools = ts;
  }
}