/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

import java.util.Map;

public class ListTagsResponse {
  public Links links;
  public Map<String, Tag> tags;
}
