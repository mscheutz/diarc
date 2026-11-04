/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import java.util.Map;

public class Tag {
  public Links links;
  public String id;
  public String mapId;
  public Position position;
  public String name;
  public Map<String, String> attributes;
}
