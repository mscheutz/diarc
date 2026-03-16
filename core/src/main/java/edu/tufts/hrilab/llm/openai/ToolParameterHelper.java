package edu.tufts.hrilab.llm.openai;

import java.util.List;

public class ToolParameterHelper {
  String name;
  String type;
  List<String> possible_values;
  String description;
  boolean required;

  public ToolParameterHelper(String name, String type, List<String> possible_values, String description, boolean required) {
    this.name = name;
    this.type = type;
    this.possible_values = possible_values;
    this.description = description;
    this.required = required;
  }


  public boolean isRequired() {
    return required;
  }

  @Override
  public String toString() {
    return String.format(
            "\"%s\": {" +
                    "\"type\": \"%s\"," +
                    "%s" +
                    "\"description\": '%s'," +
                    "},",
            name,type,(possible_values.isEmpty() ? "" : "\"enum\": ["+String.join(",",possible_values)+"],"), description);
  }

  public String getName() {
    return name;
  }

  public String getType() {
    return type;
  }

  public List<String> getPossible_values() {
    return possible_values;
  }

  public String getDescription() {
    return description;
  }
}
