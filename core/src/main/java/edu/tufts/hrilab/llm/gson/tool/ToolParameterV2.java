package edu.tufts.hrilab.llm.gson.tool;

import com.google.gson.annotations.SerializedName;
import edu.tufts.hrilab.llm.openai.ToolParameterHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;

public class ToolParameterV2 {
  protected static Logger log = LoggerFactory.getLogger(ToolParameterV2.class);

  // Shared
  String type;
  String description;

  // Object type only
  LinkedHashMap<String, ToolParameterV2> properties;
  String[] required;
  transient boolean processAsTool = false;
  // Nested Tool only
  transient String toolChatTemplateContextId = "";

  // Non-Object type only
  transient String[] possible_values;
  @SerializedName("enum")
  String[] dummy = new String[0];
  transient boolean advertisePossibleValues = false;

  //// Object type constructors
  // Arbitrary Tool call argument
  public ToolParameterV2(String type, String description, String toolChatTemplateContextId) {
    if (!(type.equals("object") || type.equals("array"))) {
      log.error("[ToolParameterV2] incorrect constructor used for supplied type. toolChatTemplateContextId may only be supplied for an argument representing an (array of) arbitrary tool call(s).");
      this.type = "object";
    } else {
      this.type = type;
    }
    setDescription(description);
    if (toolChatTemplateContextId != null && !toolChatTemplateContextId.isEmpty()) {
      this.toolChatTemplateContextId = toolChatTemplateContextId;
      processAsTool = true;
    }
  }

  // Defined Object argument
  public ToolParameterV2(String type, String description, LinkedHashMap<String,ToolParameterV2> properties, String[] required) {
    if (!(type.equals("object") || type.equals("array"))) {
      log.error("[ToolParameterV2] incorrect constructor used for supplied type. Properties may only be supplied for an object or array of objects");
      this.type = "object";
    } else {
      this.type = type;
    }
    setDescription(description);
    this.properties = properties;
    this.required = required;
  }

  //// Non-Object type constructors
  public ToolParameterV2(String type, String description) {
    if ((type.equals("object"))) {
      log.error("[ToolParameterV2] incorrect constructor used for supplied type. object parameter types must have a properties map or toolChattemplateContextId supplied.");
      this.type = "string";
    } else {
      //TODO: explicitly check for valid primitive type || array?
      this.type = type;
    }
    setDescription(description);
  }

  public ToolParameterV2(String type, String description, String[] possible_values, boolean advertisePossibleValues) {
    if ((type.equals("object"))) {
      log.error("[ToolParameterV2] incorrect constructor used for supplied type. object parameter types must have a properties map or toolChattemplateContextId supplied.");
      this.type = "string";
    } else {
      //TODO: explicitly check for valid primitive type || array?
      this.type = type;
    }
    setDescription(description);
    this.possible_values = possible_values;
    this.advertisePossibleValues = advertisePossibleValues;
    if (advertisePossibleValues) {
      dummy = possible_values;
    }
  }

  private void setDescription(String description) {
    if (description != null && !description.isEmpty()) {
      this.description = description;
    }
  }

  public String getType() {
    return type == null ? "" : type;
  }

  public String getDescription() {
    return description == null ? "" : description;
  }

  public List<String> getPossible_values() {
    return possible_values == null ? new ArrayList<>() : List.of(possible_values);
  }

  public boolean getProcessAsTool() {
    return processAsTool;
  }

  public String getToolChatTemplateContextId() {
    return toolChatTemplateContextId;
  }

  public LinkedHashMap<String, ToolParameterV2> getProperties() {
    return properties == null ? new LinkedHashMap<>() : new LinkedHashMap<>(properties);
  }

  public List<String> getRequired() {
    return required == null ? new ArrayList<>() : Arrays.stream(required).toList();
  }

  public void setPossible_values(List<String> possibleValues) {
    possible_values = possibleValues.toArray(new String[0]);
    if (advertisePossibleValues) {
      dummy = possibleValues.toArray(new String[0]);
    }
  }
}
