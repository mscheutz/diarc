/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

/**
 * Possible fields:
 *
 * translate (float): the translation velocity (m/s)
 * sidestep (float): the lateral drive velocity (m/s, in left direction)
 * rotate (float): the rotation velocity to command (radians/sec, ccw)
 * duration (float): the duration of the velocity command (seconds -- defaults to 0.5 if omitted)
 * nudge (bool): whether or not to nudge obstacles at low speed (10 cm/s)
 */
public class VelocityRequest {
  public VelocityRequest(float translate, float rotate, float sidestep) {
    this.translate = translate;
    this.rotate = rotate;
    this.sidestep = sidestep;
  }

  float translate;
  float sidestep;
  float rotate;
  float duration = 0.5f;
  boolean nudge = false;
}
