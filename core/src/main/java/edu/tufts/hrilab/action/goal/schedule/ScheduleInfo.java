/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal.schedule;

import java.io.Serializable;
import java.util.Date;

public class ScheduleInfo implements Serializable {
  private static final long serialVersionUID = 7653594451099284288L;

  /** Opt-in overlap policy; false retains the legacy skip-overlap behavior. */
  public boolean coalesce = false;
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

  public ScheduleInfo() {
  }

  public ScheduleInfo(ScheduleInfo other) {
    startTime = other.startTime == null ? null : new Date(other.startTime.getTime());
    period = other.period;
    endTime = other.endTime == null ? null : new Date(other.endTime.getTime());
    coalesce = other.coalesce;
  }
}
