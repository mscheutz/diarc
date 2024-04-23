/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.simspeech;

import static edu.tufts.hrilab.util.Util.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.simspeech.gui.SimSpeechProductionComponentVis;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import ai.thinkingrobots.trade.*;

/**
 * <code>SimSpeechProductionComponent</code>. Pops up a box for spoken text
 * instead of playing it to a sound device.
 */
public class SimSpeechProductionComponent extends DiarcComponent implements SpeechProductionInterface {
  private int[] loc = {200, 150};
  private boolean isSpeaking = false;
  private Color textColor = Color.BLACK;

  private final Object syncObj = new Object();
  private String text;
  private Long textTs = 0L;

  private boolean displayGui = false;

  private SimSpeechProductionComponentVis gui;

  @Override
  public void init() {
    if (displayGui) {
      gui = new SimSpeechProductionComponentVis(textColor, loc);
    }
  }

  /**
   * Get the most recent text (if available).
   *
   * @return the text to display, or null
   */
  @TRADEService
  public List<Object> getText(long ts) {
    List<Object> ret = null;
    synchronized (syncObj) {
      if (textTs > ts) {
        ret = new ArrayList<>();
        ret.add(textTs);
        ret.add(text);
      }
    }
    return ret;
  }

  /**
   * Sends text to the speech production component.
   */
  @Override
  public boolean sayText(String text) {
    return sayText(text, true);
  }

  /**
   * Sends text to the speech production component.
   */
  @Override
  public boolean sayText(String in, boolean wait) {
    log.info("Saying Text: " + in);

    // update gui
    if (displayGui) {
      gui.showText(in);
    }

    isSpeaking = true;
    String[] words = in.split(" ");
    long tmptime = 200 * words.length;
    if (tmptime < 500) {
      tmptime = 500;
    } else if (tmptime > 2000) {
      tmptime = 2000;
    }
    final long time = tmptime;
    synchronized (syncObj) {
      textTs = System.currentTimeMillis();
      text = in;
    }

    // simulate speaking time
    if (wait) {
      // sleep, then destroy
      Sleep(time);
      isSpeaking = false;
    } else {
      new Thread(() -> {
        // sleep, then destroy
        Sleep(time);
        isSpeaking = false;
      }).start();
    }
    return true;
  }

  /**
   * Checks if Festival is producing speech.
   *
   * @return {@code true} if speech is being produced, {@code false}
   * otherwise
   */
  @Override
  public boolean isSpeaking() {
    return isSpeaking;
  }

  /**
   * Stops an ongoing utterance.
   *
   * @return {@code true} if speech is interrupted, {@code false}
   * otherwise.
   */
  @Override
  public boolean stopUtterance() {
    return isSpeaking;
  }

  // ********************************************************************
  // *** Local methods
  // ********************************************************************

  /**
   * Provide additional information for usage...
   *
   * @return
   */
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("textred").desc("red button text").build());
    options.add(Option.builder("textblue").desc("blue button text").build());
    options.add(Option.builder("loc").numberOfArgs(2).argName("x y").desc("pop-up location").build());
    options.add(Option.builder("gui").desc("Display GUI.").build());
    return options;
  }

  /**
   * Parse additional command-line arguments
   *
   */
  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("textred")) {
      textColor = Color.RED;
    }
    if (cmdLine.hasOption("textblue")) {
      textColor = Color.BLUE;
    }
    if (cmdLine.hasOption("geom")) {
      String[] values = cmdLine.getOptionValues("geom");
      try {
        loc[0] = Integer.parseInt(values[0]);
        loc[1] = Integer.parseInt(values[1]);
      } catch (NumberFormatException nfe) {
        log.error("Error parsing geom argument.", nfe);
      }
    }
    if (cmdLine.hasOption("gui")) {
      displayGui = true;
    }
  }

}
