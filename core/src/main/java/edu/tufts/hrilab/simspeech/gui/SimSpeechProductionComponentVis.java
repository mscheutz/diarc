/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.simspeech.gui;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.beans.PropertyVetoException;

import static edu.tufts.hrilab.util.Util.*;

import javax.swing.*;
import java.awt.*;

/**
 * <code>SimSpeechProductionComponentVis</code>.  Pops up a box for spoken text
 * instead of playing it to a sound device.
 */
public class SimSpeechProductionComponentVis extends JFrame {

  private static Logger log = LoggerFactory.getLogger(SimSpeechProductionComponentVis.class);
  private int tx;
  private int ty;
  private boolean isSpeaking = false;
  private Color textColor;

  private JTextArea IText;
  private int ITextWidth = 40;

  public void showText(String input) {
    String[] words = input.split(" ");
    long tmptime = 500 * words.length;
    if (tmptime < 1000) {
      tmptime = 1000;
    } else if (tmptime > 4000) {
      tmptime = 4000;
    }
    final long time = tmptime;
    isSpeaking = true;
    IText.setText(input);
    this.setSize(50 * words.length, 100);

    this.setLocation(tx, ty);
    this.setVisible(true);
    new Thread(() -> {
      // sleep, then hide
      Sleep(time);
      this.setVisible(false);
      isSpeaking = false;
    }).start();
  }

  /**
   * SimSpeechProductionComponentVis constructor.
   */
  public SimSpeechProductionComponentVis(Color color, int[] initLoc) {
    try {
      textColor = color;
      int[] loc = initLoc;
      tx = loc[0];
      ty = loc[1];
    } catch (Exception ace) {
      log.error("SSPSVis: error getting initial parameters.", ace);
      textColor = Color.BLACK;
      tx = ty = 0;
    }
    IText = new JTextArea();
    IText.setColumns(ITextWidth);
    IText.setFont(IText.getFont().deriveFont(0, 16.0f));
    IText.setLineWrap(true);
    IText.setWrapStyleWord(true);
    IText.setEditable(false);
    IText.setForeground(textColor);
    this.add(IText, BorderLayout.PAGE_START);
  }
}
