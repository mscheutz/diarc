/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import edu.tufts.hrilab.llm.Message;

import java.util.List;

public class SocialChatHistoryData {

  public String utteranceText;
  public String systemPrompt;
  public List<Message> chatHistory;


  public void setChatHistory(List<Message> chatHistory) {
    this.chatHistory = chatHistory;
  }

  public void setUtteranceText(String utteranceText) {
    this.utteranceText = utteranceText;
  }

  public void setSystemPrompt(String systemPrompt) {
    this.systemPrompt = systemPrompt;
  }
}
