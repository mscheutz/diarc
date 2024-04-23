/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class ReportNovelty extends Active {
  protected String message;
  protected Integer level;

  public ReportNovelty(Integer level, String message) {
    this.level = level;
    this.message = message;
  }

  @Override
  public boolean canChangeGameState() {
    return false;
  }

  @Override
  public String getCommand() {
    return "REPORT_NOVELTY -l " + level + " -m " + message;
  }
}
