/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.cognex.consultant;


import edu.tufts.hrilab.abb.consultant.pose.WobjData;

public class CognexResult {
  //Members

  WobjData wobj;

  public CognexResult(WobjData data) {
    wobj = data;
  }

  @Override
  public String toString() {
    return wobj.toString();
  }
}
