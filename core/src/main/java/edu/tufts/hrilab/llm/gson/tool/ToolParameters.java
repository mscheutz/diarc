package edu.tufts.hrilab.llm.gson.tool;

import com.google.gson.internal.LinkedTreeMap;
import edu.tufts.hrilab.llm.openai.ToolParameterHelper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

public class ToolParameters {
  String type = "object";
  LinkedHashMap<String, ToolParameter> properties;
  String[] required;

  public ToolParameters(LinkedHashMap<String, ToolParameter> properties) {
    this.properties = properties;
  }

  public ToolParameters(LinkedHashMap<String, ToolParameter> properties, String[] required) {
    this.properties = properties;
    this.required = required;
  }

  public ToolParameters(List<ToolParameterHelper> params) {
    List<String> req_params = new ArrayList<>();
    properties = new LinkedHashMap<>();
    for (ToolParameterHelper parameter : params) {
      properties.put(parameter.getName(), new ToolParameter(parameter.getType(), parameter.getPossible_values(), parameter.getDescription(), parameter.getProcessAsTool(), parameter.getToolChatTemplateContextId()));
      if (parameter.isRequired()) {
        req_params.add(parameter.getName());
      }
    }
    if (!req_params.isEmpty()) {
      required = req_params.toArray(new String[0]);
    }
  }

  public String getType() {
    return type;
  }

  public LinkedHashMap<String, ToolParameter> getProperties() {
    return new LinkedHashMap<>(properties);
  }

  public List<String> getRequired() {
    return required == null ? new ArrayList<>() : Arrays.stream(required).toList();
  }
}
