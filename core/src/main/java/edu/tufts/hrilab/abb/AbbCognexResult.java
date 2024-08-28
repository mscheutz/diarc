/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb;


import edu.tufts.hrilab.abb.consultant.pose.WobjData;
import edu.tufts.hrilab.cognex.consultant.CognexResult;

public class AbbCognexResult implements CognexResult {
  //Members

  WobjData wobj;

  public AbbCognexResult(WobjData data) {
    wobj = data;
  }

  @Override
  public String toString() {
    return wobj.toString();
  }
}
