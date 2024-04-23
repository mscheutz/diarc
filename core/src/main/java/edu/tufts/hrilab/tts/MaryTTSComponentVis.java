/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tts;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

public class MaryTTSComponentVis extends JFrame {

  private static Logger log = LoggerFactory.getLogger(MaryTTSComponentVis.class);

  private Color textColor;

  private JPanel jPanel;

  private MaryTTSComponent component;

  /**
   * Constructs the SimSpeechRecognitionComponentVis
   */
  public MaryTTSComponentVis(MaryTTSComponent component) {
    this.component = component;

    // Create frame and panel
    jPanel = new JPanel(new GridLayout(0, 1));

    // Add buttons from config file
    parseConfigFile(component.getConfigFile());

    // Put it all together
    this.add(jPanel, BorderLayout.CENTER);

    Dimension minSize = this.getPreferredSize();
    int w = minSize.width;
    int h = minSize.height;
    Insets i = this.getInsets();
    w += i.left + i.right;
    h += i.top + i.bottom;
    this.setSize(w, h);

    this.setVisible(true);
  }

  private class InputListener implements ActionListener {

    @Override
    public void actionPerformed(ActionEvent event) {
      component.sayText(event.getActionCommand(), false);
    }
  }

  /**
   * Parse the config file, which is just a list of the text for which buttons
   * will be made.
   *
   * @param configFile the file from which the initialization information
   *                   resides
   */
  private void parseConfigFile(String configFile) {
    log.info(getClass().getName() + " using config file: " + configFile);
    InputStream in = getClass().getResourceAsStream(configFile);
    try {
      BufferedReader br = new BufferedReader(new InputStreamReader(in));
      String str;
      JButton tmpButton;
      while ((str = br.readLine()) != null) {
        tmpButton = new JButton(str);
        tmpButton.setForeground(textColor);
        tmpButton.addActionListener(new InputListener());
        jPanel.add(tmpButton);
      }
    } catch (IOException ioe) {
      log.error("Error parsing config file: " + ioe);
    }
  }

}
