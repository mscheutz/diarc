package edu.tufts.hrilab.llm.gson.tool;

import edu.tufts.hrilab.llm.openai.ToolParameterHelper;

import java.util.List;

public class ToolFunction {
  public String name;
  public String description;
  public ToolParameters parameters;

  public ToolFunction(String name, String description, ToolParameters parameters) {
    this.name = name;
    this.description = description;
    this.parameters = parameters;
  }

  public ToolFunction(String name, String description, List<ToolParameterHelper> parameters) {
    this.name = name;
    this.description = description;
    this.parameters = new ToolParameters(parameters);
  }
}
