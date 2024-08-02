/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.simspeech.gui;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;

import javax.swing.*;

import java.awt.*;
import java.awt.event.*;

/**
 * GUI clicky speech interface to stand in for speech recognition for
 * testing NLP, etc.
 */
public class SimSpeechRecognitionComponentVis extends JFrame {
  public static boolean useUnk = false;
  public static boolean useCommand = true;
  public static boolean toLower = true;
  private Color textColor;

  private JPanel SPanel;
  private JPanel SCommandPanel;
  private JButton SCommandButton;
  private JButton setSpeakerButton;
  private JButton setAddresseeButton;
  private JTextField SCommandText;
  private JButton SUnkButton;
  private long prevClick = 0;
  private SimSpeechRecognitionComponent component;

  private Logger log = LoggerFactory.getLogger(SimSpeechRecognitionComponentVis.class);

  /**
   * Constructs the SimSpeechRecognitionComponentVis
   */
  public SimSpeechRecognitionComponentVis(SimSpeechRecognitionComponent component) {
    super();

    this.component = component;

    // need to: get config file name, text color, useCommand, useUnk, etc.
    textColor = component.getTextColor();
    String SConfig = component.getConfigFile();
    boolean[] flags = component.getVisFlags();
    useCommand = flags[0];
    useUnk = flags[1];
    toLower = flags[2];

    // Create frame and panel
    SPanel = new JPanel();
    SPanel.setLayout(new BoxLayout(SPanel, BoxLayout.Y_AXIS));

    // Add buttons from config file
    parseConfigFile(SConfig);

    if (useCommand) {
      // Add text field and button
      SCommandPanel = new JPanel();
      SCommandPanel.setLayout(new BoxLayout(SCommandPanel, BoxLayout.Y_AXIS));
      SCommandText = new JTextField(2);
      SCommandText.addActionListener(new InputListener());
      SCommandText.setMaximumSize(new Dimension(1000, 30));
      SCommandText.setAlignmentX(Component.CENTER_ALIGNMENT);
      SCommandPanel.add(SCommandText);
      SCommandButton = new JButton("Command");
      SCommandButton.setMaximumSize(new Dimension(1000, 60));
      SCommandButton.setAlignmentX(Component.CENTER_ALIGNMENT);
      SCommandButton.setForeground(textColor);
      SCommandButton.addActionListener(new InputListener());
      SCommandButton.addMouseListener(new MouseInputListener());
      setAddresseeButton = new JButton("setAddressee");
      setAddresseeButton.setMaximumSize(new Dimension(1000, 30));
      setAddresseeButton.setAlignmentX(Component.CENTER_ALIGNMENT);
      setAddresseeButton.setForeground(textColor);
      setAddresseeButton.addActionListener(new InputListener());
      setAddresseeButton.addMouseListener(new MouseInputListener());
      setSpeakerButton = new JButton("setSpeaker");
      setSpeakerButton.setMaximumSize(new Dimension(1000, 30));
      setSpeakerButton.setAlignmentX(Component.CENTER_ALIGNMENT);
      setSpeakerButton.setForeground(textColor);
      setSpeakerButton.addActionListener(new InputListener());
      setSpeakerButton.addMouseListener(new MouseInputListener());
      SCommandPanel.add(SCommandButton);
//      SCommandPanel.add(setListenerButton);
//      SCommandPanel.add(setSpeakerButton);
    }
    if (useUnk) {
      SUnkButton = new JButton("<unk>");
      SUnkButton.addActionListener(new InputListener());
      SUnkButton.addMouseListener(new MouseInputListener());
      SPanel.add(SUnkButton);
    }

    // add scroll bars
    JScrollPane scrollPane = new JScrollPane(SPanel,
            ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED,
            ScrollPaneConstants.HORIZONTAL_SCROLLBAR_AS_NEEDED);
    scrollPane.setPreferredSize(new Dimension(600, 600));
    scrollPane.getVerticalScrollBar().setUnitIncrement(16);

    // Put it all together
    setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
    this.add(scrollPane);
    if (useCommand) {
      this.add(SCommandPanel);
    }

    int i = SConfig.lastIndexOf(File.separator);
    this.setTitle(SConfig.substring(i + 1));
    this.setSize(scrollPane.getPreferredSize());
    this.setVisible(true);
  }

  private class MouseInputListener implements MouseListener {
    @Override
    public void mousePressed(MouseEvent e) {
      Component c = e.getComponent();
      if (c instanceof JButton) {
        log.debug("mousePressed: " + ((JButton) c).getText());
      }
    }

    @Override
    public void mouseReleased(MouseEvent e) {
      Component c = e.getComponent();
      if (c instanceof JButton) {
        log.debug("mouseReleased: " + ((JButton) c).getText());
      }
    }

    @Override
    public void mouseEntered(MouseEvent e) {
      Component c = e.getComponent();
      if (c instanceof JButton) {
        log.debug("mouseEntered: " + ((JButton) c).getText());
      }
    }

    @Override
    public void mouseExited(MouseEvent e) {
      Component c = e.getComponent();
      if (c instanceof JButton) {
        log.debug("mouseExited: " + ((JButton) c).getText());
      }
    }

    @Override
    public void mouseClicked(MouseEvent e) {
      Component c = e.getComponent();
      if (c instanceof JButton) {
        log.debug("mouseClicked: " + ((JButton) c).getText());
      }
    }
  }

  private class InputListener implements ActionListener {
    @Override
    public void actionPerformed(ActionEvent event) {
      String Command = null;
      long t0 = 0L, t1 = 0L, t2, t3;
      if (event.getActionCommand().equals("Command")) {
        // Text command
        Command = SCommandText.getText();
        SCommandText.setText("");
        log.debug("Text " + Command);
      } else if (event.getActionCommand().equals("setAddressee")) {
        String listenerName = SCommandText.getText();
        SCommandText.setText("");
        component.setAddressee(Factory.createSymbol(listenerName));
        return;
      } else if (event.getActionCommand().equals("setSpeaker")) {
        String speakerName = SCommandText.getText();
        SCommandText.setText("");
        component.setSpeaker(Factory.createSymbol(speakerName));
        return;
      } else if (event.getActionCommand().equals("<unk>")) {
        String text;
        double p0 = Math.random();
        double p1 = Math.random();
        if (p0 < 0.25) {
          text = "I didn't catch that.";
        } else if (p0 < 0.5) {
          text = "I couldn't understand you.";
        } else if (p0 < 0.75) {
          text = "could you repeat that?";
        } else {
          text = "I missed that.";
        }
        if (p1 < 0.33) {
          text = "I'm sorry, " + text;
        } else if (p1 < 0.67) {
          text = "Sorry, " + text;
        }

        log.debug("<unk>");
      } else {
        // Button command
        long now = System.currentTimeMillis();
        long elapsed = now - prevClick;
        prevClick = now;
        if (elapsed < 100) {
          return;
        }
        Command = event.getActionCommand();
        t0 = System.currentTimeMillis();
        log.debug("Button " + Command);
        t1 = System.currentTimeMillis();
        if (useCommand) {
          SCommandText.setText("");
        }
      }
      if (Command == null) {
        return;
      }
      String input = Command.trim();
      if (toLower) {
        input = input.toLowerCase();
      }
      t2 = System.currentTimeMillis();
      // send it to the server
      component.setText(input);
      t3 = System.currentTimeMillis();
      log.debug("call times: " + t0 + " " + t1 + " " + t2 + " " + t3);
    }
  }

  /**
   * Parse the config file, which is just a list of the text for which
   * buttons will be made.
   *
   * @param configfile the file from which the initialization
   *                   information resides
   */
  private void parseConfigFile(String configfile) {
    BufferedReader br;
    InputStream in = this.getClass().getResourceAsStream(configfile);
    if (in == null) {
      log.error("[parseConfigFile] could not find config file: " + configfile);
    }
    InputStreamReader inr = new InputStreamReader(in);
    br = new BufferedReader(inr);
    try {
      String str;
      JButton TmpButton;
      while ((str = br.readLine()) != null) {
        if (!str.isEmpty() && !str.trim().startsWith("//")) {
          TmpButton = new JButton(str);
          TmpButton.setForeground(textColor);
          TmpButton.setMaximumSize(new Dimension(1000, 30));
          TmpButton.setAlignmentX(Component.CENTER_ALIGNMENT);
          TmpButton.addActionListener(new InputListener());
          TmpButton.addMouseListener(new MouseInputListener());
          SPanel.add(TmpButton);
        }
      }
    } catch (IOException ioe) {
      log.error("Error parsing config file: ", ioe);
    }
  }

  public void shutdown() {
    this.dispatchEvent(new WindowEvent(this, WindowEvent.WINDOW_CLOSING));
  }
}
