/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tf;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.CoordinateFramesInterface;

import javax.vecmath.Matrix4d;
import java.util.List;

public class MockTFComponent extends DiarcComponent implements CoordinateFramesInterface {
  @Override
  public Matrix4d getTransform(String dstFrame) {
    return null;
  }

  @Override
  public Matrix4d getTransform(String srcFrame, String dstFrame) {
    return null;
  }

  @Override
  public List<String> getCoordinateFrames() {
    return null;
  }

  @Override
  public void addLocalStaticTransform(String parent, String child, Matrix4d transform) {

  }
}
