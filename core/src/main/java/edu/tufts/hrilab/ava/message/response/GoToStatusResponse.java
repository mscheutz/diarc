/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

/*
● status (string) - Current status
○ UNKNOWN - The robot either could not determine its current go to state, or is not currently executing an autonomous travel command.
○ IN_PROGRESS - The robot is currently attempting to reach a commanded destination.
○ SUCCEEDED - The robot has reached its commanded destination.
○ FAILED - The robot was not able to reach its commanded destination, and is no longer attempting to drive anywhere autonomously.
○ TIMEOUT - Treat as if FAILED
○ CANCELED - A teleop or stop command has been issued to the robot while it was IN_PROGRESS. This automatically cancels the autonomous command.

● detail (string) - The detail field is set whenever the status is "FAILED"
○ PATH_OBSTRUCTED - The robot could not find a path to the commanded destination.
○ WAIT_FOR_DOOR - The robot is stuck at a closed door.
○ STOP_ENGAGED - The M-stop button has been engaged on the robot.
○ FAULT - The robot is not healthy enough to execute the commanded destination.
○ NOT_LOCALIZED - The robot can't determine its current location on the map. This must be fixed by an administrator.
 */

public class GoToStatusResponse {
  public String destination;
  public String status;
  public String detail;
  public String eta;
}