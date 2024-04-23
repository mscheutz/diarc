/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.message;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.UUID;
import org.apache.commons.lang3.StringEscapeUtils;
import com.google.gson.Gson;
import com.google.gson.annotations.Expose;

public class Message {

  private static Gson gson = new Gson();

  public String id;
  public String agent;
  public String action;
  public List<String> arguments;
  public List<String> response;

  public Message (String agent, String action) {
    this.id = UUID.randomUUID().toString();
    this.agent = agent;
    this.action = action;
    this.arguments = null;
  }
  
  public Message (String agent, String action, String... arguments) {
    this.id = UUID.randomUUID().toString();;
    this.agent = agent;
    this.action = action;
    this.arguments = Arrays.asList(arguments);
  }
  
  @Override
  public String toString () {
    return gson.toJson(this);
  }

  private static String removeQuotesAndUnescape (String uncleanJson) {
    String cleanJson = uncleanJson.replaceAll("^\"|\"$", "").replaceAll("\0", "").trim();
    return StringEscapeUtils.unescapeJava(cleanJson);
  }

  public static Message fromString (String s) {
    return gson.fromJson(Message.removeQuotesAndUnescape(s), Message.class);
  }
}
