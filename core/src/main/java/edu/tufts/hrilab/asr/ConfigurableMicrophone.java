/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.asr;

import java.io.InputStream;
import javax.sound.sampled.*;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ConfigurableMicrophone {
  static private Logger log = LoggerFactory.getLogger(ConfigurableMicrophone.class);
  private final TargetDataLine line;
  private final InputStream inputStream;

  public ConfigurableMicrophone(float frameRate, int sampleSize, boolean signed, boolean bigEndian, int mixerID) {

    AudioFormat format = new AudioFormat(frameRate, sampleSize, 1, signed, bigEndian);

    try {
      Mixer.Info[] mixers = AudioSystem.getMixerInfo();
      if (mixerID == -1 || mixerID >= mixers.length) {
        line = AudioSystem.getTargetDataLine(format);
        log.warn("[ConfigurableMicrophone] Given an invalid mixerID, using default mixer");
      } else {
        line = AudioSystem.getTargetDataLine(format, mixers[mixerID]);
      }
      line.open();
    } catch (LineUnavailableException e) {
      throw new IllegalStateException(e);
    }

    this.inputStream = new AudioInputStream(this.line);
  }
  public void startRecording() {
    this.line.start();
  }

  public void stopRecording() {
    this.line.stop();
  }

  public InputStream getStream() {
    return this.inputStream;
  }

  public int read (byte[] buffer, int i) {
    return line.read(buffer, i, buffer.length);
  }

  public static void showMixers () {
    int i = 0;
    Mixer.Info[] mixers = AudioSystem.getMixerInfo();
    for (Mixer.Info mixer : mixers) {
      log.info("[" + i + "] " + mixer.toString());
      log.info("    " + mixer.getDescription());
      i++;
    }
  }

  /**
   * Escapes all regex characters in a given string except for "{}".
   *
   * @param str the input string
   * @return the escaped string
   */
  private static String escapeRegexExceptBraces(String str) {
    return str.replaceAll("([\\\\+\\[\\](){}^$.|?*])", "\\\\$1").replace("\\{\\}", "{}");
  }

  /**
   * When provided a string, returns the mixer ID of the device that matches
   * the name or description first. It also has the ability to match {} with anything.
   *
   * @param match String to match, where "{}" can match any substring
   * @return mixer ID if a match is found, -1 otherwise
   */
  public static int matchMixer(String match) {
    Mixer.Info[] mixers = AudioSystem.getMixerInfo();
    String escapedMatch = escapeRegexExceptBraces(match);
    String regex = escapedMatch.replace("{}", ".*"); // Replace {} with regex to match any substring
    regex = ".*" + regex + ".*"; // necessary to work with substrings

    for (int i = 0; i < mixers.length; i++) {
      Mixer.Info mixer = mixers[i];
      if (mixer.toString().matches(regex) || mixer.getDescription().matches(regex)) {
        return i;
      }
    }
    return -1; // Return -1 if no match is found
  }
}