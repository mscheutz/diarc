/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal.schedule;

import java.io.Serializable;
import java.util.Date;

public class ScheduleInfo implements Serializable {
  /**
   * Time to start executing the first scheduled goal. null meaning, do it right now.
   */
  public Date startTime = null;
  /**
   * Milliseconds. For repeated/period goals. Value less than 0 indicated no repetition (i.e., just do it once).
   */
  public long period = -1;
  /**
   * Time to stop executing this goal. null indicates no end time (i.e., run forever).
   * This only applies for repeated goals (i.e., period greater than 0).
   */
  public Date endTime = null;
}
