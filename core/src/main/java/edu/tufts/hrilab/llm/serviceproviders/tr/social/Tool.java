package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.slug.common.UtteranceType;

public class Tool {
  public enum PostCallBehavior {
    CLEAR_CONTEXT,
    CLEAR_HISTORY,
    KEEP_HISTORY
  }

  ToolDefinition toolDefinition;
  UtteranceType utteranceType;
  // Originally intended change for focused contexts - whether this tool being called should terminate the current
  //  conversation context. Originally any INSTRUCT target did terminate and any ITK target did not. Want to allow
  //  the ability to have extended
  PostCallBehavior postCallBehavior = PostCallBehavior.CLEAR_CONTEXT;



  public Tool(ToolDefinition toolDefinition, UtteranceType utteranceType) {
    this.toolDefinition = toolDefinition;
    this.utteranceType = utteranceType;
  }

  public Tool(ToolDefinition toolDefinition, UtteranceType utteranceType, PostCallBehavior postCallBehavior) {
    this.toolDefinition = toolDefinition;
    this.utteranceType = utteranceType;
    this.postCallBehavior = postCallBehavior;
  }
}
