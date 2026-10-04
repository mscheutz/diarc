package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ChatTemplates {
  Prompt systemMessageTemplate;
  List<ToolDefinition> toolDefinitions;
  HashMap<String, Tool> tools;

  //TODO: This should look very different. Currently this way because we cannot save location on the fly and therefore
  //        ToolDefinitions won't change in this domain. In general, possible_values entries can change over time and
  //        therefore ToolDefinitions need to be updated in response to relevant consultant updates.
  //      Ideal solution is to keep tools as templates similar to the system prompt, and apply consultant ref lists
  //        as prompt variables on submission.
  //      Issue with ^ currently is that tool template conversion is on the server side, so we cannot do that in the same way
  public ChatTemplates(Prompt systemPrompt, List<Tool> tools) {
    systemMessageTemplate = systemPrompt;
    toolDefinitions = new ArrayList<>();
    this.tools = new HashMap<>();
    for (Tool tool : tools) {
      toolDefinitions.add(tool.toolDefinition);
      this.tools.put(tool.toolDefinition.function.name, tool);
    }
  }
}