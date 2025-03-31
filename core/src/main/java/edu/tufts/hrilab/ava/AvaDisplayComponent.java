/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.ava.display.DialogueManager;
import edu.tufts.hrilab.ava.display.DisplayCanvas;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.util.resource.Resources;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.awt.Color;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.Toolkit;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.awt.Cursor;
import java.awt.Point;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.imageio.ImageIO;
import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

/**
 * Class representing the display functionality on the screen of the Ava robot.
 * More specifically the high level interaction between Ava/DIARC and the granular canvas
 *
 * @author Matt McWilliams
 */
public class AvaDisplayComponent extends DiarcComponent {

  private DisplayCanvas canvas;
  private boolean isFullscreen = false;
  private boolean useChat = false;
  private DialogueManager dialogueManager = new DialogueManager();

  private final long textTimeout = 20000; //20 seconds
  private final long idleTimeout = 600000; //10 minutes

  private long idleTimer = -1;
  private boolean idle = false;

  private final ScheduledExecutorService refreshExecutor = Executors.newScheduledThreadPool(1);

  private TRADEServiceInfo isWakeWordEnabledService = null;
  private TRADEServiceInfo getWakeWordsService = null;

  private String suggestedInput = null;

  @Override
  protected void init() {
    initializeFrame();
    displayImage("hrilab_logo.jpg", 10 * 1000);
    refreshExecutor.scheduleAtFixedRate(() -> {
      try {
        refreshMonitor();
      } catch (TRADEException e) {
        log.error("TRADE Exception while refreshing display.", e);
      } catch (Exception e) {
        log.error("Exception while refreshing display.", e);
      }
    }, 0, 100, TimeUnit.MILLISECONDS); // 10 fps
  }

  @Override
  protected void postRegistrationInit() {
    // register to get notified of "registerForDialogueHistoryNotifications" service
    try {
      TRADE.registerNotificationNowOrWhenJoined(this, new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications").argTypes(TRADEServiceInfo.class), "registerDialogueCallback");
    } catch (TRADEException e) {
      log.error("Error trying to register for registerForDialogueHistoryNotifications service.", e);
    }
  }

  /**
   * Sets up the display. This includes initializing the display environment, configuring the screen.
   */
  private void initializeFrame() {
    Dimension screenSize = (isFullscreen) ? Toolkit.getDefaultToolkit().getScreenSize() : new Dimension(800, 600);
    double screenWidth = screenSize.getWidth();
    double screenHeight = screenSize.getHeight();

    log.info("AvaDisplayComponent Resolution: " + screenWidth + "x" + screenHeight);

    canvas = new DisplayCanvas((int) screenWidth, (int) screenHeight);
    canvas.setDialogueManager(dialogueManager);
    dialogueManager.setUseChat(useChat);

    JPanel panel = new JPanel();
    panel.add(canvas);
    panel.setPreferredSize(new Dimension((int) screenWidth, (int) screenHeight));
    panel.setBackground(new Color(0, 0, 0));

    JFrame frame = new JFrame("DIARC");
    frame.setBackground(new Color(0, 0, 0));
    frame.setContentPane(panel);
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    if (isFullscreen) {
      frame.setUndecorated(true);
      frame.setResizable(false);

      GraphicsEnvironment graphics = GraphicsEnvironment.getLocalGraphicsEnvironment();
      GraphicsDevice device = graphics.getDefaultScreenDevice();
      device.setFullScreenWindow(frame);

      BufferedImage blankCursorImage = new BufferedImage(16, 16, BufferedImage.TYPE_INT_ARGB);
      Cursor blankCursor = Toolkit.getDefaultToolkit().createCustomCursor(blankCursorImage, new Point(0, 0), "blank cursor");
      frame.getContentPane().setCursor(blankCursor);
    } else {
      frame.setUndecorated(false);
      frame.setResizable(false);
      frame.setSize((int) screenWidth, (int) screenHeight);
      frame.setVisible(true);
    }

  }

  /**
   * Registers avaDisplayDialogueCallback as the callback service for dialog events.
   */
  @TRADEService
  public void registerDialogueCallback(TRADEServiceInfo service) {
    TRADEServiceInfo myService = this.getMyService("avaDisplayDialogueCallback", Utterance.class, Boolean.class);
    try {
      service.call(void.class, myService);
    } catch (TRADEException e) {
      log.error("Error registering for dialogue history notifications.", e);
    }
  }

  @TRADEService
  public void setSuggestedInput(String suggestedInput) {
    this.suggestedInput = suggestedInput;
  }

  /**
   * Update display and handle idle logic. This is called continually in a separate thread.
   */
  private void refreshMonitor() throws TRADEException {
    // If not idle and reached out idle timeout, make idle
    long currMillis = System.currentTimeMillis();
    if (!idle && (currMillis - idleTimer) >= idleTimeout) {
      setIdle();
    }

    boolean isWakeWordEnabled = false;
    if (isWakeWordEnabledService == null) {
      isWakeWordEnabledService = TRADE.getAvailableService(new TRADEServiceConstraints().name("isWakeWordEnabled").argTypes());
    }
    if (isWakeWordEnabledService != null) {
      isWakeWordEnabled = isWakeWordEnabledService.call(boolean.class);
    }

    List<String> wakeWords = null;
    if (getWakeWordsService == null) {
      getWakeWordsService = TRADE.getAvailableService(new TRADEServiceConstraints().name("getWakeWords").argTypes());
    }
    if (getWakeWordsService != null) {
      wakeWords = getWakeWordsService.call(List.class);
    }

    if (suggestedInput != null) {
      String wakeWord = "";
      if (isWakeWordEnabled && wakeWords != null && !wakeWords.isEmpty()) {
        wakeWord = wakeWords.get(0) + ", ";
      }
      displayText("Try: " + wakeWord + suggestedInput);
    } else {
      clearText();
    }

    canvas.repaint();
  }

  /**
   * Displays an image from the resources directory on the DisplayCanvas.
   *
   * @param imageName filename of image to display
   */
  private void displayImage(String imageName, long timeout) {
    log.debug("displayImage(" + imageName + ")");
    String displayDirectoryPath = "config/edu/tufts/hrilab/ava/display";
    String imageFullPath = Resources.createFilepath(displayDirectoryPath, imageName);
    BufferedImage image = null;
    try {
      image = ImageIO.read(AvaDisplayComponent.class.getResourceAsStream(imageFullPath));
    } catch (IOException ex) {
      log.error("Error getting image " + imageName, ex);
    } catch (Exception ex) {
      log.error("Other error getting image " + imageName, ex);
    }
    if (image == null) {
      return;
    }
    unsetIdle();
    canvas.setImage(image, timeout);
  }

  /**
   * Exposed TRADE service for clearing image on the DisplayCanvas.
   */
  @TRADEService
  public void clearImage() {
    log.debug("clearImage()");
    canvas.clearImage();
  }

  /**
   * Exposed TRADE service for clearing text on the DisplayCanvas.
   */
  @TRADEService
  public void clearText() {
    log.debug("clearText()");
    canvas.clearText();
  }

  /**
   * Exposed TRADE service for displaying text on the DisplayCanvas.
   *
   * @param text      string to display
   */
  @TRADEService
  public void displayText(String text) {
    log.debug("displayText(" + text + ")");
    unsetIdle();
    canvas.setText(text, textTimeout);
  }

  /**
   * Displays the robot background in place of the HRI lab logo.
   */
  @TRADEService
  public void displayRobot() {
    displayImage("robot_emoji.jpg", -1);
  }

  /**
   * Callback TRADE service to receive dialogue events and display
   * them on the DisplayCanvas.
   *
   * @param utterance object from dialogue history
   */
  @TRADEService
  public void avaDisplayDialogueCallback(Utterance utterance, boolean newAddition) {
    if (newAddition) {
      log.debug(utterance.toString());
      String speaker = utterance.getSpeaker().toString();
      String words = utterance.getWordsAsString();
      dialogueManager.addMessage(words, speaker);
      unsetIdle();
    }
  }

  /**
   * Sets the display to be idle
   */
  private void setIdle() {
    idle = true;
    canvas.setIdle(true);
  }

  /**
   * Unsets the idle state and updates the idle timer
   */
  private void unsetIdle() {
    idle = false;
    idleTimer = System.currentTimeMillis();
    canvas.setIdle(false);
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("chat").argName("boolean").desc("switch dialogue to chat format").build());
    options.add(Option.builder("fullscreen").argName("boolean").desc("make the display full screen or not").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("chat")) {
      useChat = true;
    }
    if (cmdLine.hasOption("fullscreen")) {
      isFullscreen = true;
    }
  }

  @Override
  public void shutdownComponent() {
    displayText("shutting down...");
    refreshExecutor.shutdown();
    canvas = null;
  }
}