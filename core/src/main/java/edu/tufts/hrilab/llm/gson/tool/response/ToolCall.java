package edu.tufts.hrilab.llm.gson.tool.response;

public class ToolCall {
  public String id;
  public String type;
  public ToolCallFunction function;

  //Streaming chunk only?
  public int index;

  @Override
  public String toString() {
    return function.name + "(" + function.arguments + ")";
  }
}
