/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.pose;

import java.util.ArrayList;

public class WobjData {
  boolean robHold = false;
  boolean uFProg = false;
  String uFMex = "";
  RSPoseData uFrame = new RSPoseData();
  RSPoseData oFrame = new RSPoseData();

  public WobjData() {
  }

  public void setRobHold(boolean robHold) {
    this.robHold = robHold;
  }

  public void setuFProg(boolean uFProg) {
    this.uFProg = uFProg;
  }

  public void setuFMex(String uFMex) {
    this.uFMex = uFMex;
  }

  public void setuFrame(ArrayList<ArrayList<Double>> uFrame) {
    this.uFrame = new RSPoseData(uFrame.get(0), uFrame.get(1));
  }

  public void setoFrame(ArrayList<ArrayList<Double>> oFrame) {
    this.oFrame = new RSPoseData(oFrame.get(0), oFrame.get(1));
  }

  public void setuFrame(RSPoseData uFrame) {
    this.uFrame = uFrame;
  }

  public void setoFrame(RSPoseData oFrame) {
    this.oFrame = oFrame;
  }

  public boolean isRobHold() {
    return robHold;
  }

  public boolean isuFProg() {
    return uFProg;
  }

  public String getuFMex() {
    return uFMex;
  }

  public RSPoseData getuFrame() {
    return uFrame;
  }

  public RSPoseData getoFrame() {
    return oFrame;
  }

  public ArrayList<Double> getuFramePos() {
    return uFrame.getPos();
  }

  public ArrayList<Double> getuFrameRot() {
    return uFrame.getRot();
  }

  public ArrayList<Double> getoFramePos() {
    return oFrame.getPos();
  }

  public ArrayList<Double> getoFrameRot() {
    return oFrame.getRot();
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("[");
    builder.append(String.valueOf(robHold).toUpperCase());
    builder.append(",");
    builder.append(String.valueOf(uFProg).toUpperCase());
    builder.append(",");
    builder.append("\"" + uFMex + "\"");
    builder.append(",");
    builder.append(uFrame.asStringPointOnly());
    builder.append(",");
    builder.append(oFrame.asStringPointOnly());
    builder.append("]");
    return builder.toString();
  }
}
