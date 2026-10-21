package edu.tufts.hrilab.llm.gson.tool;

import edu.tufts.hrilab.llm.openai.ToolParameterHelper;

import java.util.List;

public class ToolDefinition {
  public String type = "function";
  public ToolFunction function;

  public ToolDefinition(ToolFunction toolFunction) {
    this.function = toolFunction;
  }
}
