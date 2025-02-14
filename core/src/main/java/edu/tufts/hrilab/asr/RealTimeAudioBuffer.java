/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.asr;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.sound.sampled.AudioInputStream;
import java.io.IOException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class RealTimeAudioBuffer {
  private Logger log = LoggerFactory.getLogger(RealTimeAudioBuffer.class);
  private int bufferSize = 16000;
  private BlockingQueue<byte[]> bufferQueue;

  /**
   *
   *
   * @param bufferSize How big each snippet to process
   * @param queueCapacity How many snippets willing to hold before (WHAT ACTUALLY HAPPENS WHEN THE QUEUE IS FULL)
   */
  public RealTimeAudioBuffer(int bufferSize, int queueCapacity) {
    this.bufferSize = bufferSize;
    bufferQueue = new ArrayBlockingQueue<>(queueCapacity);
  }

  public void bufferAudioData(AudioInputStream audioInputStream) {
    byte[] buffer = new byte[bufferSize];
    int bytesRead;

    try (audioInputStream) {
      while ((bytesRead = audioInputStream.read(buffer, 0, bufferSize)) != -1) {
        byte[] data = new byte[bytesRead];
        System.arraycopy(buffer, 0, data, 0, bytesRead);
        bufferQueue.put(data);
      }
    } catch (IOException e) {
      log.error("[bufferAudioData] Error reading from AudioInputStream: ", e);
    } catch (InterruptedException e) {
      log.error("[bufferAudioData] Error putting data in bufferQueue: ", e);
    }
  }

  public byte[] getBufferedAudioData() throws InterruptedException {
    return bufferQueue.take();
  }
}