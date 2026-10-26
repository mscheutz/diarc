/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import edu.tufts.hrilab.llm.message.Message;

import java.util.ArrayList;
import java.util.List;

public class SocialChatHistories {
  private String testName;
  public List<SocialChatHistoryData> utterances;

  /**
   * To keep track of what UtteranceData is currently being added to.
   */
  private int addIndex = -1;
  /**
   * To keep track of which tesUtteranceDatalet results to return next.
   */
  private int getterIndex = 0;

  public SocialChatHistories(String testName) {
    this.testName = testName;
    this.utterances = new ArrayList<>();
  }

  public void startNextUtterance() {
    utterances.add(new SocialChatHistoryData());
    addIndex = utterances.size() - 1;
  }
  public void addSystemPrompt(String systemPrompt) {
    this.utterances.get(addIndex).setSystemPrompt(systemPrompt);
  }
  public void addUtteranceText(String utteranceText) {
    this.utterances.get(addIndex).setUtteranceText(utteranceText);
  }
  public void addChatHistory(List<Message> chatHistory) {
    this.utterances.get(addIndex).setChatHistory(chatHistory);
  }
}
