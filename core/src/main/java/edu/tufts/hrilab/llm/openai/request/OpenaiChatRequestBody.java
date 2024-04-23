/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.request;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.llm.Message;

public class OpenaiChatRequestBody {
  public String model;
  public List<Message> messages;

  public OpenaiChatRequestBody () {

  }

  public OpenaiChatRequestBody (String m, List<Message> msgs) {
    model = m;
    messages = msgs;
  }
}