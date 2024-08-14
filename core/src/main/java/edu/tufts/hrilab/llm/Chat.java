/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.knuddels.jtokkit.*;

import edu.tufts.hrilab.llm.openai.response.*;

public class Chat {
  static private Logger log = LoggerFactory.getLogger(Chat.class);

  public int tokens = 0;
  public List<Message> messages = new ArrayList<Message>();
  public String userName = "USER";
  public String robotName = "ASSISTANT";
  public String systemMessage = "";
  public String context = "";
  public String service = "openai";
  public String model = "gpt-4";

  public List<Integer> tokenCounts = new ArrayList<Integer>();
  private Tokenizer tokenizer = new Tokenizer();

  public Chat () {

  }

  public Chat (String user, String robot) {
    userName = user;
    robotName = robot;
  }

  public Chat (String systemText) {
    systemMessage = systemText;
  }

  public Chat (Prompt systemPrompt) {
    systemMessage = systemPrompt.toString();
  }

  public Chat (String user, String robot, String systemText) {
    userName = user;
    robotName = robot;
    systemMessage = systemText;
  }

  public Chat (List<Message> msgs) {
    messages = msgs;
  }

  public Chat (Message message) {
    messages.add(message);
    updateTokenCounts();
  }

  public Chat (String systemText, List<Message> msgs) {
    systemMessage = systemText;
    messages = msgs;
    updateTokenCounts();
  }

  public Chat (Prompt systemPrompt, List<Message> msgs) {
    systemMessage = systemPrompt.toString();
    messages = msgs;
    updateTokenCounts();
  }

  public Chat (Completion a, Completion b) {
    String str = a.toString();
    str += "\n\n";
    str += b.toString();
    systemMessage = str;
    updateTokenCounts();
  }

  /**
   * User methods
   **/

  public void setUser (String user) {
    userName = user;
  }

  public void setRobot (String robot) {
    robotName = robot;
  }

  public String getUserRole () {
    return roleWrapper(userName);
  }

  public String getRobotRole () {
    return roleWrapper(robotName);
  }

  public String roleWrapper (String role) {
    //return "[" + role + "]";
    return "<|" + role + "|>";
  }

  /**
   * Message methods
   **/

  public void addMessage (String role, String content) {
    messages.add(new Message(role, content));
    updateTokenCounts();
  }

  public void addMessage (Message message) {
    messages.add(message);
    updateTokenCounts();
  }

  public void addMessages (List<Message> msgs) {
    for (Message message : msgs) {
      messages.add(message);
    }
    updateTokenCounts();
  }

  public void setMessages (List<Message> msgs) {
    messages = msgs;
    updateTokenCounts();
  }

  public void setMessages (Chat chat) {
    messages = chat.messages;
    updateTokenCounts();
  }

  public void addUserMessage (String content) {
    messages.add(new Message("user", content));
    updateTokenCounts();
  }

  public void addRobotMessage (String content) {
    messages.add(new Message("assistant", content));
    updateTokenCounts();
  }

  public void addInteraction (Message a, Message b) {
    messages.add(a);
    messages.add(b);
    updateTokenCounts();
  }

  public void addCompletion (Completion completion) {
    messages.add(new Message("assistant", completion.text));
    updateTokenCounts();
  }

  public void addCompletion (Completion completion, boolean updateTokens) {
    if (updateTokens) {
      tokens = completion.inputTokens;
      tokens += completion.outputTokens;
    }
    messages.add(new Message("assistant", completion.text));
  }

  public List<Message> getMessages () {
    return messages;
  }

  /**
   * System + Context methods
   **/

  public void setSystem (Prompt systemPrompt) {
    systemMessage = systemPrompt.toString();
  }

  public void setSystem (String systemText) {
    systemMessage = systemText;
  }

  public void setSystem (Chat chat) {
    systemMessage = chat.systemMessage;
  }

  public String getSystem () {
    return systemMessage;
  }

  public void setContext (String contextText) {
    context = contextText;
  }

  public void setContext (Chat chat) {
    context = chat.context;
  }

  public void addContext (String addition) {
    if (!context.equals("")) {
      context += "\n" + addition;
    } else {
      context = addition;
    }
  }

  public String getContext () {
    return context;
  }

  private void updateTokenCounts () {
    int count;
    tokens = 0;
    tokenCounts = new ArrayList<Integer>();
    Message systemMessage = toSystemMessage();
    if (systemMessage != null) {
      count = tokenizer.count(systemMessage.content.toString());
      tokens += count;
      log.debug(systemMessage.role + " tokens: " + count);
    }
    for (Message message : messages) {
      count = tokenizer.count(message.content.toString());
      tokenCounts.add(count);
      tokens += count;
      log.debug(message.role + " tokens: " + count);
    }
    log.debug("total tokens: " + tokens);
  }

  public Message toSystemMessage () {
    String systemContent = "";
    if (!systemMessage.equals("")) {
      systemContent += systemMessage + "\n\n";
    }
    if (!context.equals("")) {
      systemContent += context + "\n\n";
    }
    if (!systemContent.equals("")) {
      return new Message("system", systemContent);
    }
    return null;
  }

  /**
   * Prompt methods
   **/

  public List<Message> toPrompt () {
    List<Message> prompt = new ArrayList<Message>();
    Message systemMessage = toSystemMessage();
    if (systemMessage != null) {
      prompt.add(systemMessage);
    }
    for (Message message : messages) {
      prompt.add(message);
    }
    return prompt;
  }

  public String toPromptString () {
    String prompt = "<s>\n";
    if (!systemMessage.equals("") || !context.equals("")) {
      prompt += "<SYS>\n>";
    }
    if (!systemMessage.equals("")) {
      prompt += systemMessage + "\n";
    }
    if (!context.equals("")) {
      prompt += context + "\n";
    }
    if (!systemMessage.equals("") || !context.equals("")) {
      prompt += "</SYS>\n\n";
    }
    for (Message message : messages) {
      String role = message.role.equals("user") ? getUserRole() : getRobotRole();
      prompt += role + " ";
      prompt += message.content + "\n\n";
    }
    prompt += getRobotRole() + "";
    return prompt.trim();
  }
}