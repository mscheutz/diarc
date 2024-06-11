/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.cmu.sphinx.api;

import java.io.InputStream;
import javax.sound.sampled.*;

public class ConfigurableMicrophone {
  private final TargetDataLine line;
  private final InputStream inputStream;

  public ConfigurableMicrophone(float frameRate, int sampleSize, boolean signed, boolean bigEndian, int mixerID) {

    AudioFormat format = new AudioFormat(frameRate, sampleSize, 1, signed, bigEndian);

    try {
      Mixer.Info[] mixers = AudioSystem.getMixerInfo();
      this.line = AudioSystem.getTargetDataLine(format, mixers[mixerID]);
      this.line.open();
    } catch (LineUnavailableException e) {
      throw new IllegalStateException(e);
    }

    this.inputStream = new AudioInputStream(this.line);
  }

//  public byte[] getData(int start, int end) {
//
//    //TODO: not sure how to resolve the fact that the long might be bigger than an int
//    int dataLength = end - start;
//    byte[] data = new byte[dataLength];
//
//    try {
//      speechSourceProvider.getMicrophone().getStream().read(data, (int) (long) start, (int) (long) start + dataLength);
//    } catch (IOException e) {
//      System.out.println("couldn't read from stream " + e);
//    }
//    return data;
//  }

  public void startRecording() {
    this.line.start();
  }

  public void stopRecording() {
    this.line.stop();
  }

  public InputStream getStream() {
    return this.inputStream;
  }
}