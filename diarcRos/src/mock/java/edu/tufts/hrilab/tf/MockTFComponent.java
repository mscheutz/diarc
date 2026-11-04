/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tf;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.CoordinateFramesInterface;

import javax.vecmath.Matrix4d;
import java.util.ArrayList;
import java.util.List;

public class MockTFComponent extends DiarcComponent implements CoordinateFramesInterface {

  @Override
  public Matrix4d getTransform(String srcFrame, String dstFrame) {
    Matrix4d tf =  new Matrix4d();
    tf.setIdentity();
    return tf;
  }

  @Override
  public List<String> getCoordinateFrames() {
    return new ArrayList<>();
  }

  @Override
  public void addLocalStaticTransform(String parent, String child, Matrix4d transform) {

  }
}
