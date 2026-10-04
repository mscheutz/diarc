package edu.tufts.hrilab.llm.gson.tool;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.LinkedHashMap;

public class ToolFunction {
  protected static Logger log = LoggerFactory.getLogger(ToolFunction.class);

  public String name;
  public String description;
  public ToolParameterV2 parameters;

  public ToolFunction(String name, String description) {
    this.name = name;
    this.description = description;
    parameters = new ToolParameterV2("object",null, new LinkedHashMap<>(), new String[0]);
  }

  public ToolFunction(String name, String description, LinkedHashMap<String,ToolParameterV2> parameters) {
    this.name = name;
    this.description = description;
    this.parameters = new ToolParameterV2("object",null,parameters, new String[0]);
  }

  public ToolFunction(String name, String description, LinkedHashMap<String,ToolParameterV2> parameters, String[] required) {
    this.name = name;
    this.description = description;
    this.parameters = new ToolParameterV2("object",null,parameters, required);
  }
}
