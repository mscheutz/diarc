/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Utility methods for stereo image processing.
 */
public class StereoProc {

  static private Logger log = LoggerFactory.getLogger(StereoProc.class);

  /**
   * Convert disparity image into 3D points.
   *
   * [X Y Z W] = Q * [x y disp(x,y) 1]^T
   * 3dImage(x,y) = (X/W, Y/W, Z/W) 
   * Q =| 1 0 0 -Cx |
   *    | 0 1 0 -Cy |
   *    | 0 0 0  f  |
   *    | 0 0 a  b  |
   * ==========================
   * X = (x-Cx)*T/disp(x,y);
   * Y = (y-Cy)*T/disp(x,y);
   * Z = f*T/disp(x,y);
   * ===========================
   *
   * @param disparity
   * @param width image width (pixel units)
   * @param height image height (pixel units)
   * @param f focal length (pixel units)
   * @param T baseline length between optical centers (world units)
   * @param scale (current disparity image height) / (original disparity image height)
   * @return
   */
  static public double[][] reprojectImageTo3D(byte[] disparity, int width, int height, double f, double T, double scale) {

    // assumes principal point in center
    int cx = width / 2;
    int cy = height / 2;

    double[][] points3d = new double[width * height][3];
    for (int i = 0; i < width * height; ++i) {
      int asInt = (disparity[i * 4] & 0xFF)
              | ((disparity[i * 4 + 1] & 0xFF) << 8)
              | ((disparity[i * 4 + 2] & 0xFF) << 16)
              | ((disparity[i * 4 + 3] & 0xFF) << 24);
      float disp = Float.intBitsToFloat(asInt);
      int x = i % width;
      int y = i / width;
      if (disp > 0.1f && disp < 127.0f) {
        points3d[i][0] = (1/scale) * (x - cx) * T / disp;
        points3d[i][1] = (1/scale) * (y - cy) * T / disp;
        points3d[i][2] = f * T / disp;
      }
    }
    return points3d;
  }

  /**
   * Convert depth image (e.g., from kinect or xtion) into 3D points.
   * @param depthData raw depth data
   * @param width depth data width
   * @param height depth data height
   * @param f focal length
   * @return
   */
  static public double[][] reprojectImageTo3D(byte[] depthData, int width, int height, double f) {

    // TODO: remove hard-coded values for kinect/xtion sensor
    double fx = f * (width / 640.0);
    double fy = f * (height / 480.0);
    double cx = width / 2.0;
    double cy = height / 2.0;

    double[][] points3d = new double[width * height][3];

    for (int i = 0; i < width * height; ++i) {
      int asInt = (depthData[i * 4] & 0xFF)
              | ((depthData[i * 4 + 1] & 0xFF) << 8)
              | ((depthData[i * 4 + 2] & 0xFF) << 16)
              | ((depthData[i * 4 + 3] & 0xFF) << 24);
      float z_ir = Float.intBitsToFloat(asInt);

      if (z_ir > 0) {
        int x = i % width;
        int y = i / width;

        points3d[x + y * width][0] = ((x - cx) / fx) * z_ir; //x
        points3d[x + y * width][1] = ((y - cy) / fy) * z_ir; //y
        points3d[x + y * width][2] = z_ir; //z
      }
    }

    return points3d;
  }
}
