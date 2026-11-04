/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Trade extends Active {
  protected String traderID;
  protected String inputArgs;

  public Trade(String traderID, String inputArgs) {
    this.traderID = traderID;
    this.inputArgs = inputArgs;
  }

  @Override
  public String getCommand() {
    return "TRADE " + traderID + " " + inputArgs;
  }
}
