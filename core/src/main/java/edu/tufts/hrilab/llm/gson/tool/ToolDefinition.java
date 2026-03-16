package edu.tufts.hrilab.llm.gson.tool;

import edu.tufts.hrilab.llm.openai.ToolParameterHelper;

import java.util.List;

public class ToolDefinition {
  public String type = "function";
  public ToolFunction function;

  public ToolDefinition(String name, String description, ToolParameters parameters) {
    this.function = new ToolFunction(name, description, parameters);
  }

  public ToolDefinition(String name, String description, List<ToolParameterHelper> parameters) {
    this.function = new ToolFunction(name, description, parameters);
  }
}
