/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.ava.display.DialogueManager;
import edu.tufts.hrilab.ava.display.DisplayCanvas;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.util.resource.Resources;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import uk.co.caprica.vlcj.player.base.MediaPlayer;
import uk.co.caprica.vlcj.player.base.MediaPlayerEventAdapter;
import uk.co.caprica.vlcj.player.component.EmbeddedMediaPlayerComponent;

import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import javax.swing.*;
import javax.imageio.ImageIO;
import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
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

  private final long textTimeout = 8000; //8 seconds
  private final long idleTimeout = 600000; //10 minutes

  private long idleTimer = -1;
  private boolean idle = false;

  private EmbeddedMediaPlayerComponent mediaPlayerComponent;

  private final ScheduledExecutorService refreshExecutor = Executors.newScheduledThreadPool(1);
  private ScheduledFuture currentImageCycleFuture = null;

  private TRADEServiceInfo isWakeWordEnabledService = null;
  private TRADEServiceInfo getWakeWordsService = null;

  private String suggestedInput = null;
  private boolean useWakeWord = false;
  private CardLayout cardLayout;
  private JPanel cardPanel;
  private String videoDir = "";
  private String idleImageDir = "";
  private String userImageDir = "";
  private List<String> videos = new ArrayList<>();
  private int currentVideo = -1;
  private int videoVolume;
  private boolean videoPlaying = false;
  private boolean userImageDisplayed = false;
  private List<String> idleImages = new ArrayList<>();
  private List<String> userImages = new ArrayList<>();
  private int currentImage = -1;
  private int trLogoIndex = 0;

  @Override
  protected void init() {
    if (!videoDir.equals("")) {
      videos = getMediaFromDir(videoDir, ".mp4");
      idleImages = getMediaFromDir(idleImageDir, ".png");
      idleImages = idleImages.stream().sorted().toList();
      userImages = getMediaFromDir(userImageDir, ".png");
      userImages = userImages.stream().sorted().toList();
      currentVideo = 0;
      currentImage = 0;
      int index = 0;
      for (String img : idleImages) {
        if (img.contains("(1)")) {
          trLogoIndex = index;
          currentImage = trLogoIndex;
        }
        index++;
      }
    }
    initializeFrame();
  }

  private String getMediaPath(String videoDir) {
    String homeDir = System.getProperty("user.home");
    String diarcDirPath = Paths.get(homeDir, ".diarc").toString();
    return Paths.get(diarcDirPath, videoDir).toString();
  }

  private List<String> getMediaFromDir(String videoDir, String fileType) {
    String videoDirPath = getMediaPath(videoDir);
    File dir = new File(videoDirPath);
    File[] files = dir.listFiles((dir1, name) -> name.toLowerCase().endsWith(fileType));
    List<String> toReturn = List.of();
    if (files != null) {
      List<File> fileList = List.of(files);
      toReturn = fileList.stream().map(x -> x.getName()).toList();
    }
    return toReturn;
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

    cardPanel = new JPanel();
    cardLayout = new CardLayout();
    cardPanel.setLayout(cardLayout);

    JPanel imagePanel = new JPanel();
    imagePanel.add(canvas);
    imagePanel.setPreferredSize(new Dimension((int) screenWidth, (int) screenHeight));
    imagePanel.setBackground(new Color(0, 0, 0));

    JFrame frame = new JFrame("DIARC");
    frame.setBackground(new Color(0, 0, 0));
    frame.setContentPane(cardPanel);
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    cardPanel.add(imagePanel);

    frame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent e) {
        mediaPlayerComponent.release();
        frame.setVisible(false);
        frame.dispose();
        System.exit(0);
      }
    });

    mediaPlayerComponent = new EmbeddedMediaPlayerComponent() {
      @Override
      public void playing(MediaPlayer mediaPlayer) {
        mediaPlayer.audio().setVolume(videoVolume);
        mediaPlayer.controls().play();
      }

      @Override
      public void stopped(MediaPlayer mediaPlayer) {
        videoPlaying = false;
        continueIdleImageCycle();
      }

    };

    cardPanel.add(mediaPlayerComponent);

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

    startIdleImageCycle(0);
  }

  private void startIdleImageCycle(int initialDelay) {
    if (currentImageCycleFuture != null) {
      log.error("[startIdleImageCycle] called while previous scheduled future still exists");
      return;
    }
    Runnable cycleImageTask = this::continueIdleImageCycle;

    currentImageCycleFuture = refreshExecutor.scheduleAtFixedRate(cycleImageTask, initialDelay, 30, TimeUnit.SECONDS);
  }

  private void continueIdleImageCycle() {
    if (!videoPlaying && !userImageDisplayed) {
      try {
        SwingUtilities.invokeAndWait(() -> {
                  displayIdleImage(idleImages.get(currentImage), -1);
                }
        );
      } catch (InterruptedException | InvocationTargetException e) {
        log.error("error advancing idle image", e);
      }

      if (currentImage >= (idleImages.size() - 1)) {
        currentImage = 0;
      } else {
        currentImage++;
      }
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

    if (useWakeWord) {
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
    }

  }

  private String getMatchingImageName(String name) {
    for (String image : userImages) {
      if (image.equalsIgnoreCase(name)) {
        return image;
      }
    }
    return "";
  }

  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "interruptDisplayImageWithName()", onSuspendServiceCall = "interruptDisplayImageWithName()")
  public boolean displayImageWithName(String imageName, Symbol timeoutSymbol) {
    try {
      long timeout = Long.parseLong(timeoutSymbol.getName());
      return displayImageWithName(imageName, timeout);
    } catch (NumberFormatException e) {
      log.error("[displayImageWithName] error parsing timeoutSymbol {}", timeoutSymbol, e);
      return false;
    }
  }

  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "interruptDisplayImageWithName()", onSuspendServiceCall = "interruptDisplayImageWithName()")
  public boolean displayImageWithName(String imageName, long timeout) {
    String name = getMatchingImageName(imageName);
    if (name.isEmpty()) {
      return false;
    }

    displayUserImage(imageName, timeout, true);

    return true;
  }

  //TODO: smarter interaction with timeout on suspend/resume? Currently will reset timer
  @TRADEService
  public void interruptDisplayImageWithName() {
    canvas.cancelSetImage();
    clearImage();
    clearText();
  }

  private void displayUserImage(String imageName, long timeout) {
    displayUserImage(imageName, timeout, false);
  }
  private void displayIdleImage(String imageName, long timeout) {
    displayIdleImage(imageName,timeout,false);
  }
  private void displayUserImage(String imageName, long timeout, boolean blocking) {
    userImageDisplayed = true;
    clearText();
    String path = getMediaPath(userImageDir) + File.separator + imageName;
    displayImage(path, timeout, blocking);
    userImageDisplayed = false;
    continueIdleImageCycle();
  }
  private void displayIdleImage(String imageName, long timeout, boolean blocking) {
    String path = getMediaPath(idleImageDir) + File.separator + imageName;
    displayImage(path, timeout, blocking);
  }

  /**
   * Displays an image from the resources directory on the DisplayCanvas.
   *
   * @param path filename of image to display
   */
  private void displayImage(String path, long timeout, boolean blocking) {
    log.debug("displayIdleImage(" + path + ")");
    BufferedImage image = null;
    try {
      image = ImageIO.read(new File(path));
    } catch (IOException ex) {
      log.error("Error getting image " + path, ex);
    } catch (Exception ex) {
      log.error("Other error getting image " + path, ex);
    }
    if (image == null) {
      return;
    }
    unsetIdle();
    if (blocking) {
      canvas.setImageBlocking(image, timeout);
    } else {
      canvas.setImage(image, timeout);
    }
  }

  @TRADEService
  public void displayNextVideo() {
    if (currentVideo == -1) {
      log.error("[displayNextVideo] no valid videos to display");
      return;
    }
    displayVideo(videos.get(currentVideo));
    if (currentVideo >= (videos.size() - 1)) {
      currentVideo = 0;
    } else {
      currentVideo++;
    }
  }

  private String getMatchingVideoName(String name) {
    for (String video : videos) {
      if (video.equalsIgnoreCase(name)) {
        return video;
      }
    }
    return "";
  }

  @TRADEService
  public boolean isVideoSaved(String name) {
    return !getMatchingVideoName(name).isEmpty();
  }

  @TRADEService
  public boolean displayVideoMatching(String match) {
    String name = getMatchingVideoName(match);
    if (name.isEmpty()) {
      return false;
    }
    displayVideo(name);
    return true;
  }

  private void displayVideo(String videoName) {
    try {
      SwingUtilities.invokeAndWait(() -> {
                cardLayout.next(cardPanel);
              }
      );
    } catch (InterruptedException | InvocationTargetException e) {
      log.error("error switching back to logo", e);
    }
    String path = getMediaPath(videoDir) + File.separator + videoName;
    mediaPlayerComponent.mediaPlayer().media().startPaused(path);
    videoPlaying = true;
    log.info("[displayNextVideo] displaying video: " + path);
  }

  @TRADEService
  public boolean isMediaPlaying() {
    return videoPlaying;
  }

  @TRADEService
  public void cancelVideoPlayback() {
    mediaPlayerComponent.mediaPlayer().controls().stop();
    try {
      SwingUtilities.invokeAndWait(() -> {
                cardLayout.previous(cardPanel);
              }
      );
    } catch (InterruptedException | InvocationTargetException e) {
      log.error("error switching back to logo", e);
    }
    videoPlaying = false;
  }

  @TRADEService
  public void pauseVideoPlayback() {
    mediaPlayerComponent.mediaPlayer().controls().pause();
  }

  @TRADEService
  public void resumeVideoPlayback() {
    mediaPlayerComponent.mediaPlayer().controls().play();
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
   * Callback TRADE service to receive dialogue events and display
   * them on the DisplayCanvas.
   *
   * @param utterance object from dialogue history
   */
  @TRADEService
  public void avaDisplayDialogueCallback(Utterance utterance, boolean newAddition) {
    //if (newAddition) {
    //  log.debug(utterance.toString());
    //  String speaker = utterance.getSpeaker().toString();
    //  String words = utterance.getWordsAsString();
    //  dialogueManager.addMessage(words, speaker);
    //  unsetIdle();
    //}
    if (utterance.getSpeaker().getName().equals("ava")) {
      new Thread(() -> {
        //String displayedText = "Ava: " + utterance.getWordsAsString() + "\n" + currentText;
        //displayText(displayedText);
        String currentText = canvas.getText();
        //Not sure what timing ideally looks like here - recognized speech string overwritten by response string
        if (!currentText.isEmpty()) {
          long sleepDuration = 500;
          if (currentText.startsWith("Recognized Speech:")) {
            sleepDuration = 1500;
          }
          try {
            Thread.sleep(sleepDuration);
          } catch (InterruptedException ignored) {
          }
        }
        displayText("Ava: " + utterance.getWordsAsString());
      }).start();
    }
  }

  @TRADEService
  public List<String> getVideoNames(boolean includeFileEndings) {
    if (includeFileEndings) {
      return new ArrayList<>(videos);
    } else {
      return new ArrayList<>(videos.stream().map(v -> v.split("\\.", 2)[0]).toList());
    }
  }

  @TRADEService
  public List<String> getImageNames(boolean includeFileEndings) {
    if (includeFileEndings) {
      return new ArrayList<>(userImages);
    } else {
      return new ArrayList<>(userImages.stream().map(v -> v.split("\\.", 2)[0]).toList());
}
  }

  @TRADEService
  public boolean setVideoVolume(Symbol volumeSymbol) {
    try {
      int volume = Integer.parseInt(volumeSymbol.getName());
      return setVideoVolume(volume);
    } catch (NumberFormatException e) {
      log.error("[setVideoVolume] supplied volumeSymbol not parseable as int: {}", volumeSymbol, e);
      return false;
    }
  }

  @TRADEService
  public boolean setVideoVolume(int volume) {
    if (volume < 0 || volume > 100) {
      log.error("[setVideoVolume] supplied volume is out of bounds [0,100]: {}",volume);
      return false;
    }
    videoVolume = volume;
    return true;
  }

  @TRADEService
  public int getVideoVolume() {
    return videoVolume;
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
    options.add(Option.builder("mediaDir").hasArg().desc("name of base directory with videos, idleImages, and userImages subdirectories in .diarc dir").build());
    options.add(Option.builder("videoVolume").hasArg().desc("volume for video playback").build());
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
    if (cmdLine.hasOption("mediaDir")) {
      String mediaDir = cmdLine.getOptionValue("mediaDir");
      videoDir = mediaDir + File.separator + "videos";
      idleImageDir = mediaDir + File.separator + "idleImages";
      userImageDir = mediaDir + File.separator + "userImages";
    }
    if (cmdLine.hasOption("videoVolume")) {
      videoVolume = Integer.parseInt(cmdLine.getOptionValue("videoVolume"));
    }
  }

  @Override
  public void shutdownComponent() {
    displayText("shutting down...");
    refreshExecutor.shutdown();
    canvas = null;
  }
}