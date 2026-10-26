/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.lidar;

import edu.tufts.hrilab.interfaces.LidarInterface;
import edu.tufts.hrilab.diarc.DiarcComponent;

import edu.tufts.hrilab.diarcros.laserscan.LaserScanSubscriber;


public class LidarComponent extends DiarcComponent implements LidarInterface {
  private LaserScanSubscriber laserScanSubscriber;


  @Override
  protected void init() {
    laserScanSubscriber = new LaserScanSubscriber("/base_scan");
    log.debug("LidarComponent waiting on laser scan data...");
    laserScanSubscriber.waitForNode();
    log.debug("LidarComponent ready to go!");
  }

  @Override
  public float[] getLaserScan() {
    float[] ls = {};
    while (laserScanSubscriber.getScan() == null) {
    }
    ;
    ls = laserScanSubscriber.getScan().getRanges();
    return ls;
  }

  @Override
  public float getLaserAngleIncrement() {
    float angleIncrement = 0.0f;
    while (laserScanSubscriber.getScan() == null) {
    }
    ;
    angleIncrement = laserScanSubscriber.getScan().getAngleIncrement();
    return angleIncrement;
  }
}