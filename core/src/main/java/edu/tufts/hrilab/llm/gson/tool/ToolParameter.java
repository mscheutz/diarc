package edu.tufts.hrilab.llm.gson.tool;

import com.google.gson.annotations.SerializedName;

import java.util.List;

public class ToolParameter {
  String type;
  //@SerializedName("enum")
  //String[] possible_values;
  String description;

  //TODO: Should ideally be externally configurable whether this is included in the exposed tool definition per-entry
  //  or not. Want to still hold this information for postprocessing, but want to advertise constraints separately.
  transient String[] possible_values;
  @SerializedName("enum")
  String[] dummy = new String[0];

  //ServiceProvider only <- split json and java reps?
  transient boolean processAsTool = false;
  transient String toolChatTemplateContextId = "";

  public ToolParameter(String type, List<String> possible_values, String description) {
    this.type = type;
    this.possible_values = possible_values.toArray(new String[0]);
    this.description = description;
  }

  public ToolParameter(String type, List<String> possible_values, String description, boolean processAsTool, String toolChatTemplateContextId) {
    this.type = type;
    this.possible_values = possible_values.toArray(new String[0]);
    this.description = description;
    this.processAsTool = processAsTool;
    this.toolChatTemplateContextId = toolChatTemplateContextId;
  }

  public String getType() {
    return type;
  }

  public List<String> getPossible_values() {
    return List.of(possible_values);
  }

  public String getDescription() {
    return description;
  }

  public boolean getProcessAsTool() {
    return processAsTool;
  }

  public String getToolChatTemplateContextId() {
    return toolChatTemplateContextId;
  }

  public void setPossible_values(List<String> possibleValues) {
    possible_values = possibleValues.toArray(new String[0]);
  }
}
