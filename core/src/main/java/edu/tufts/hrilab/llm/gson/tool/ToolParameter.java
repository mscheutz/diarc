package edu.tufts.hrilab.llm.gson.tool;

import com.google.gson.annotations.SerializedName;

import java.util.List;

public class ToolParameter {
  String type;
  @SerializedName("enum")
  String[] possible_values;
  String description;

  public ToolParameter(String type, List<String> possible_values, String description) {
    this.type = type;
    this.possible_values = possible_values.toArray(new String[0]);
    this.description = description;
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
}
