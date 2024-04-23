/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import java.io.IOException;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.vision.gui.VideoDisplay;
import edu.tufts.hrilab.vision.util.CompressionUtil;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.Raster;
import java.awt.image.WritableRaster;
import java.util.zip.DataFormatException;

/**
 * DIARC Component to retrieve and display images captured and sent from a
 * VideoSendComponent.
 *
 * @author evankrause
 */
public class VideoReceiveComponent extends DiarcComponent {

  //image data
  private Dimension imageSize;
  private WritableRaster raster;
  private BufferedImage buffImage;
  //image display
  VideoDisplay videoDisplay;

  /**
   * VideoReceiveComponent constructor.
   */
  public VideoReceiveComponent() {
    super();

    shouldRunExecutionLoop = true;
    executionLoopCycleTime = 30; // ~30hz
  }

  @Override
  protected void executionLoop() {
    // this method is called periodically, based on the updateLoopTime.

    if (imageSize == null) {
      //initialize other objects
      try {
        imageSize = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageSize")).call(Dimension.class);
      } catch (TRADEException e) {
        log.warn("Failed to call getImageSize.", e);
        return;
      }
      raster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE, imageSize.width, imageSize.height, 3, null);
      buffImage = new BufferedImage(imageSize.width, imageSize.height, BufferedImage.TYPE_3BYTE_BGR);
      videoDisplay = new VideoDisplay(imageSize);
    }

    //get image data
    byte[] frameComp = new byte[0];
    try {
      frameComp = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFrame")).call(byte[].class);
    } catch (TRADEException e) {
      log.warn("Failed to call getFrame.", e);
      return;
    }

    byte[] frame;
    try {
      frame = CompressionUtil.decompress(frameComp);
    } catch (IOException | DataFormatException e) {
      log.error("Error decompressing image.", e);
      return;
    }

    //build java image with data
    raster.setDataElements(0, 0, imageSize.width, imageSize.height, frame);
    buffImage.setData(raster);

    //display image
    videoDisplay.displayImage(buffImage);
  }
}
