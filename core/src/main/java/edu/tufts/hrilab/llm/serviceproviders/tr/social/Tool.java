package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.slug.common.UtteranceType;

public class Tool {
  ToolDefinition toolDefinition;
  UtteranceType utteranceType;

  public Tool(ToolDefinition toolDefinition, UtteranceType utteranceType) {
    this.toolDefinition = toolDefinition;
    this.utteranceType = utteranceType;
  }
}
