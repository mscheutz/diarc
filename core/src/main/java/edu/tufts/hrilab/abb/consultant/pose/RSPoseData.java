/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.pose;

import java.util.ArrayList;
import java.util.stream.Collectors;

public class RSPoseData {
  ArrayList<Double> pos;
  ArrayList<Double> rot;

  ArrayList<Integer> robConf;
  ArrayList<Double> extAxs;

  public RSPoseData() {
    pos = new ArrayList<>();
    rot = new ArrayList<>();
    robConf = new ArrayList<>();
    extAxs = new ArrayList<>();
  }

  public RSPoseData(ArrayList<Double> pos, ArrayList<Double> rot) {
    this.pos = pos;
    this.rot = rot;
  }

  public RSPoseData(ArrayList<ArrayList<Double>> pose) {
    pos = pose.get(0);
    rot = pose.get(1);
    robConf = (ArrayList<Integer>) pose.get(2).stream().map(Double::intValue).collect(Collectors.toList());
    extAxs = pose.get(3);
  }

  public ArrayList<Double> getPos() {
    return pos;
  }

  public ArrayList<Double> getRot() {
    return rot;
  }

  public void setPos(ArrayList<Double> pos) {
    this.pos = pos;
  }

  public void setRot(ArrayList<Double> rot) {
    this.rot = rot;
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("[");
    builder.append(pos);
    builder.append(",");
    builder.append(rot);
    builder.append(",");
    builder.append(robConf);
    builder.append(",");
    builder.append(extAxs);
    builder.append("]");
    return builder.toString();
  }

  //todo: should separate into 2 different classes (point and pose)
  public String asStringPointOnly() {
    StringBuilder builder = new StringBuilder();
    builder.append("[");
    builder.append(pos);
    builder.append(",");
    builder.append(rot);
    builder.append("]");
    return builder.toString();
  }
}
