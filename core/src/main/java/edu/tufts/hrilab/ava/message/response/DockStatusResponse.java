/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

/*{
    "command": "Dock",
    "detail": "Running",
    "state": "Undocked",
    "status": "IN_PROGRESS"
    "docks":
    [
    {
    "x": "0.5295923",
    "y": "0.0249549",
    "theta": "-3.10177",
    "type": "1"
    }
    ]
    }*/

public class DockStatusResponse {
  public String command;
  public String detail;
  public String state;
  public String status;
  public Dock[] docks;
  public int[] errors;
}