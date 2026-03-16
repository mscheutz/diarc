package edu.tufts.hrilab.llm.gson.tool.response;

public class ToolCall {
  public String id;
  public String type;
  public ToolFunction function;

  @Override
  public String toString() {
    return function.name + "(" + function.arguments + ")";
  }
}
