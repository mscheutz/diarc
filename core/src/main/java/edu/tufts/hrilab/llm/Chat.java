/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.ArrayList;
import java.util.List;

import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;
import edu.tufts.hrilab.llm.message.Message;
import edu.tufts.hrilab.llm.message.MultiModalMessage;
import edu.tufts.hrilab.llm.message.TextMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Chat {
  static private Logger log = LoggerFactory.getLogger(Chat.class);

  public int tokens = 0;
  public List<Message> messages = new ArrayList<>();
  public String userName = "user";
  public String robotName = "assistant";
  public String systemMessage = "";
  public String context = "";
  public String service = "openai";
  public String model = "gpt-4";

  //TODO: reintroduce when having a formal tokenizer representation which words (or pulling data from completion responses)
  //public List<Integer> tokenCounts = new ArrayList<Integer>();
  //private Tokenizer tokenizer = new Tokenizer();

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
  }

  public Chat (String systemText, List<Message> msgs) {
    systemMessage = systemText;
    messages = msgs;
  }

  public Chat (Prompt systemPrompt, List<Message> msgs) {
    systemMessage = systemPrompt.toString();
    messages = msgs;
  }

  public Chat (Chat other) {
    messages = new ArrayList<>();
    for (Message msg : other.messages) {
      messages.add(new Message(msg.role, msg.content, msg.reasoning, msg.tool_calls, msg.tool_call_id));
    }
    tokens = other.tokens;
    userName = other.userName;
    robotName = other.robotName;
    systemMessage = other.systemMessage;
    context = other.context;
    service = other.service;
    model = other.model;
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
    messages.add(new TextMessage(role, content));
  }

  public void addMessage (Message message) {
    messages.add(message);
  }

  public void addMessages (List<Message> msgs) {
    for (Message message : msgs) {
      messages.add(message);
    }
  }

  public void setMessages (List<Message> msgs) {
    messages = msgs;
  }

  public void setMessages (Chat chat) {
    messages = chat.messages;
  }

  public void addUserMessage (String content) {
    messages.add(new TextMessage("user", content));
  }

  public void addRobotMessage (String content) {
    messages.add(new TextMessage("assistant", content));
  }

  public void addRobotMessage (String content, List<ToolCall> toolCalls) {
    messages.add(new TextMessage("assistant", content, toolCalls.toArray(new ToolCall[]{})));
  }

  public void addToolMessage (String content, String tool_call_id) {
    messages.add(new TextMessage("tool", content, tool_call_id));
  }

  public void addToolVisionMessage(String text, String imageUrl, String tool_call_id) {
    messages.add(new MultiModalMessage("tool", text, imageUrl, tool_call_id));
  }

  public void addUserVisionMessage(String text, String imageUrl) {
    messages.add(new MultiModalMessage("user", text, imageUrl));
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

  //private void updateTokenCounts () {
  //  int count;
  //  tokens = 0;
  //  tokenCounts = new ArrayList<Integer>();
  //  Message systemMessage = toSystemMessage();
  //  if (systemMessage != null) {
  //    count = tokenizer.count(systemMessage.content.toString());
  //    tokens += count;
  //    log.debug(systemMessage.role + " tokens: " + count);
  //  }
  //  for (Message message : messages) {
  //    count = tokenizer.count(message.content.toString());
  //    tokenCounts.add(count);
  //    tokens += count;
  //    log.debug(message.role + " tokens: " + count);
  //  }
  //  log.debug("total tokens: " + tokens);
  //}

  public TextMessage toSystemMessage () {
    String systemContent = "";
    if (!systemMessage.equals("")) {
      systemContent += systemMessage + "\n\n";
    }
    if (!context.equals("")) {
      systemContent += context + "\n\n";
    }
    if (!systemContent.equals("")) {
      return new TextMessage("system", systemContent);
    }
    return null;
  }

  /**
   * Prompt methods
   **/

  public List<Message> toPrompt () {
    List<Message> prompt = new ArrayList<Message>();
    TextMessage systemMessage = toSystemMessage();
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