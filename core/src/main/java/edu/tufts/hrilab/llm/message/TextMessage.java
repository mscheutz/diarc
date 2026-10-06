package edu.tufts.hrilab.llm.message;

import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;

public class TextMessage extends Message<String> {

  public TextMessage(String role, String content) {
    this.role = role;
    this.content = content;
  }

  public TextMessage (String role, String text, String tool_call_id) {
    this.role = role;
    content = text;
    this.tool_call_id = tool_call_id;
  }

  public TextMessage (String role, String text, ToolCall[] tool_calls) {
    this.role = role;
    content = text;
    this.tool_calls = tool_calls;
  }
}
