/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.ava.stats.AvaStatisticsEvent;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.ava.display.DialogueManager;
import edu.tufts.hrilab.ava.display.DisplayCanvas;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import uk.co.caprica.vlcj.factory.MediaPlayerFactory;
import uk.co.caprica.vlcj.player.base.MediaPlayer;
import uk.co.caprica.vlcj.player.component.EmbeddedMediaPlayerComponent;

import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.geom.Point2D;
import javax.swing.*;
import javax.imageio.ImageIO;
import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.lang.reflect.Type;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.concurrent.*;
import java.util.stream.Collectors;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

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

  private MediaPlayerFactory mediaPlayerFactory;
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
  private String presentationSlidesDir = "";
  private String videoAudioDevice = "";
  private List<String> videos = new ArrayList<>();
  private int currentVideo = -1;
  private int videoVolume;
  private volatile boolean videoPlaying = false;
  private volatile long videoPlaybackStartMs = 0;
  private volatile boolean userImageDisplayed = false;
  private List<String> idleImages = new ArrayList<>();
  /** User-selected uploaded image names (from userImageDir) to include in idle cycle. Persisted in userIdleImages.json. */
  private List<String> userSelectedIdleImageNames = new ArrayList<>();
  /** Combined cycle list: "idle:filename" or "user:filename" for path resolution. */
  private List<String> idleImageCycleList = new ArrayList<>();
  private List<String> userImages = new ArrayList<>();
  private int currentImage = -1;
  private int trLogoIndex = 0;
  private volatile boolean idleImageCycleRunning = true;
  private String currentlyDisplayedImagePath = "";
  private static final String IDLE_PREFIX = "idle:";
  private static final String USER_PREFIX = "user:";

  private JPanel listeningIndicatorPane;
  private BufferedImage listeningIndicatorImage;
  private BufferedImage listeningIndicatorComposite;
  private static final String LISTENING_INDICATOR_IMAGE = System.getProperty("user.home") + "/.diarc/ava/cramertransparent2.png";
  private static final double LISTENING_INDICATOR_HEIGHT_FRACTION = 0.6;
  private static final int VIDEO_VOLUME_STEP = 15;

  @Override
  protected void init() {
    refreshMedia();
    currentVideo = 0;
    currentImage = 0;
    for (int i = 0; i < idleImageCycleList.size(); i++) {
      String entry = idleImageCycleList.get(i);
      String name = entry.startsWith(USER_PREFIX) ? entry.substring(USER_PREFIX.length()) : entry.substring(IDLE_PREFIX.length());
      if (name.contains("(1)")) {
        trLogoIndex = i;
        currentImage = trLogoIndex;
        break;
      }
    }
    initializeFrame();
  }

  @TRADEService
  public void refreshMedia() {
    if (!videoDir.equals("")) {
      videos = getMediaFromDir(videoDir, MediaTypes.VIDEO_EXTENSIONS);
      idleImages = getMediaFromDir(idleImageDir, MediaTypes.IMAGE_EXTENSIONS);
      idleImages = idleImages.stream().sorted().toList();
      userImages = getMediaFromDir(userImageDir, MediaTypes.IMAGE_EXTENSIONS);
      userImages = userImages.stream().sorted().toList();
      loadUserIdleImageSelection();
      buildIdleImageCycleList();
    }
  }

  private Path getUserIdleImagesConfigPath() {
    if (idleImageDir == null || idleImageDir.isEmpty()) {
      return Paths.get(System.getProperty("user.home"), ".diarc", "ava", "userIdleImages.json");
    }
    Path parent = Paths.get(idleImageDir).getParent();
    if (parent == null) {
      return Paths.get(System.getProperty("user.home"), ".diarc", "ava", "userIdleImages.json");
    }
    return Paths.get(getMediaPath(parent.toString()), "userIdleImages.json");
  }

  private void loadUserIdleImageSelection() {
    Path path = getUserIdleImagesConfigPath();
    if (!Files.exists(path)) {
      userSelectedIdleImageNames = new ArrayList<>();
      return;
    }
    try {
      String json = Files.readString(path, StandardCharsets.UTF_8);
      Type type = new TypeToken<List<String>>() {}.getType();
      List<String> loaded = new Gson().fromJson(json, type);
      if (loaded != null) {
        userSelectedIdleImageNames = new ArrayList<>(loaded);
      } else {
        userSelectedIdleImageNames = new ArrayList<>();
      }
    } catch (IOException e) {
      log.warn("[loadUserIdleImageSelection] could not read {}: {}", path, e.getMessage());
      userSelectedIdleImageNames = new ArrayList<>();
    }
    // Keep only names that exist in userImages and are images
    userSelectedIdleImageNames = userSelectedIdleImageNames.stream()
        .filter(name -> userImages.contains(name) && MediaTypes.isImage(name))
        .sorted()
        .collect(Collectors.toList());
  }

  private void saveUserIdleImageSelection() {
    Path path = getUserIdleImagesConfigPath();
    try {
      String json = new Gson().toJson(userSelectedIdleImageNames);
      Files.writeString(path, json, StandardCharsets.UTF_8);
    } catch (IOException e) {
      log.warn("[saveUserIdleImageSelection] could not write {}: {}", path, e.getMessage());
    }
  }

  /** Builds the idle cycle list: if the user has any selected images, use only those; otherwise use built-in idle images. */
  private void buildIdleImageCycleList() {
    idleImageCycleList = new ArrayList<>();
    if (!userSelectedIdleImageNames.isEmpty()) {
      for (String name : userSelectedIdleImageNames) {
        idleImageCycleList.add(USER_PREFIX + name);
      }
    } else {
      for (String name : idleImages) {
        idleImageCycleList.add(IDLE_PREFIX + name);
      }
    }
    if (currentImage >= idleImageCycleList.size()) {
      currentImage = idleImageCycleList.isEmpty() ? 0 : idleImageCycleList.size() - 1;
    }
  }

  private String getMediaPath(String videoDir) {
    String homeDir = System.getProperty("user.home");
    String diarcDirPath = Paths.get(homeDir, ".diarc").toString();
    return Paths.get(diarcDirPath, videoDir).toString();
  }

  private List<String> getMediaFromDir(String mediaDir, Set<String> extensions) {
    String mediaDirPath = getMediaPath(mediaDir);
    File dir = new File(mediaDirPath);
    File[] files = dir.listFiles((dir1, name) -> {
      String ext = MediaTypes.getExtension(name);
      return extensions.contains(ext);
    });
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

  @TRADEService
  public String getVideosDir() {
    return getMediaPath(videoDir);
  }

  @TRADEService
  public String getUserImageDir() {
    return getMediaPath(userImageDir);
  }

  @TRADEService
  public void deleteMedia(String path) {
    try {
      log.error("[deleteMedia] deleteMedia called on path: " + path);
      Files.deleteIfExists(Path.of(path));
    } catch (IOException e) {
      log.error("[deleteMedia] failed to delete media file: " + path, e);
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

    JPanel imagePanel = new JPanel(new BorderLayout());
    imagePanel.add(canvas, BorderLayout.CENTER);
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
        mediaPlayerFactory.release();
        frame.setVisible(false);
        frame.dispose();
        System.exit(0);
      }
    });

    String[] vlcArgs;
    if (!videoAudioDevice.isEmpty()) {
      vlcArgs = new String[]{"--aout=alsa", "--alsa-audio-device=" + videoAudioDevice};
    } else {
      vlcArgs = new String[]{};
    }
    mediaPlayerFactory = new MediaPlayerFactory(vlcArgs);

    mediaPlayerComponent = new EmbeddedMediaPlayerComponent(mediaPlayerFactory, null, null, null, null) {
      @Override
      public void playing(MediaPlayer mediaPlayer) {
        mediaPlayer.audio().setVolume(videoVolume);
      }

      @Override
      public void stopped(MediaPlayer mediaPlayer) {
        videoPlaying = false;
        try {
          SwingUtilities.invokeAndWait(() -> {
                    cardLayout.previous(cardPanel);
                  }
          );
        } catch (InterruptedException | InvocationTargetException e) {
          log.error("error switching back to idle display", e);
        }
        continueIdleImageCycle();
      }

    };

    cardPanel.add(mediaPlayerComponent);

    try {
      listeningIndicatorImage = ImageIO.read(new File(LISTENING_INDICATOR_IMAGE));
    } catch (IOException e) {
      log.warn("[initializeFrame] could not load listening indicator image {}: {}", LISTENING_INDICATOR_IMAGE, e.toString());
    }
    if (listeningIndicatorImage == null) {
      log.warn("[initializeFrame] listening indicator image unavailable: {}", LISTENING_INDICATOR_IMAGE);
    }

    // Custom paint: radial vignette behind the head plus a soft glow halo so the asset's
    // hard alpha cutout edge blends into the background instead of reading as a bad mask.
    listeningIndicatorPane = new JPanel() {
      @Override
      protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        int pw = getWidth();
        int ph = getHeight();
        if (pw <= 0 || ph <= 0) {
          return;
        }
        if (listeningIndicatorComposite == null
            || listeningIndicatorComposite.getWidth() != pw
            || listeningIndicatorComposite.getHeight() != ph) {
          listeningIndicatorComposite = renderListeningIndicator(pw, ph);
        }
        g.drawImage(listeningIndicatorComposite, 0, 0, null);
      }
    };
    listeningIndicatorPane.setOpaque(true);
    listeningIndicatorPane.setVisible(false);
    frame.setGlassPane(listeningIndicatorPane);
    // pre-render so the first show is an instant blit, not a full re-render on the EDT
    listeningIndicatorComposite = renderListeningIndicator((int) screenWidth, (int) screenHeight);

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
    if (idleImageCycleList.isEmpty()) {
      return;
    }
    if (!videoPlaying && !userImageDisplayed && idleImageCycleRunning) {
      try {
        SwingUtilities.invokeAndWait(() -> {
                  if (currentImage >= 0 && currentImage < idleImageCycleList.size()) {
                    displayIdleImage(idleImageCycleList.get(currentImage), -1);
                  }
                }
        );
      } catch (InterruptedException | InvocationTargetException e) {
        log.error("error advancing idle image", e);
      }

      if (currentImage >= (idleImageCycleList.size() - 1)) {
        currentImage = 0;
      } else {
        currentImage++;
      }
    }
  }

  @TRADEService
  @Action
  public void stopIdleImageCycle() {
    idleImageCycleRunning = false;
  }

  @TRADEService
  @Action
  public void startIdleImageCycle() {
    idleImageCycleRunning = true;
    continueIdleImageCycle();
  }

  private BufferedImage renderListeningIndicator(int pw, int ph) {
    BufferedImage out = new BufferedImage(pw, ph, BufferedImage.TYPE_INT_RGB);
    Graphics2D g2 = out.createGraphics();
    g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
    Point2D ctr = new Point2D.Float(pw / 2f, ph / 2f);
    g2.setPaint(new RadialGradientPaint(ctr, Math.max(pw, ph) * 0.75f,
        new float[]{0f, 1f}, new Color[]{new Color(38, 40, 46), new Color(0, 0, 0)}));
    g2.fillRect(0, 0, pw, ph);
    if (listeningIndicatorImage != null) {
      int th = (int) (ph * LISTENING_INDICATOR_HEIGHT_FRACTION);
      int tw = (int) (listeningIndicatorImage.getWidth() * ((double) th / listeningIndicatorImage.getHeight()));
      int x = (pw - tw) / 2;
      int y = (ph - th) / 2;
      g2.setPaint(new RadialGradientPaint(ctr, Math.max(tw, th) * 0.62f,
          new float[]{0f, 1f}, new Color[]{new Color(150, 160, 190, 110), new Color(150, 160, 190, 0)}));
      g2.fillRect(0, 0, pw, ph);
      g2.drawImage(listeningIndicatorImage, x, y, tw, th, null);
    }
    g2.dispose();
    return out;
  }

  // Lets ASL drive the listening indicator off (mirrors triggerWW's no-arg pattern;
  // passing a boolean from ASL is the same type-coercion headache as displayText).
  @TRADEService
  @Action
  public void hideListeningIndicator() {
    setListeningIndicator(false);
  }

  @TRADEService
  public void setListeningIndicator(boolean visible) {
    if (listeningIndicatorPane == null) {
      log.info("[setListeningIndicator] pane null, ignoring visible={}", visible);
      return;
    }
    SwingUtilities.invokeLater(() -> {
      listeningIndicatorPane.setVisible(visible);
      log.info("[setListeningIndicator] applied visible={} showing={} bounds={}", visible,
          listeningIndicatorPane.isVisible(), listeningIndicatorPane.getBounds());
    });
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

  /**
   * Normalize a filename for matching: lowercase, collapse dashes/underscores/spaces
   * so "RobotLedgerPage.png", "robot-ledger-page.png", "robot_ledger_page.png" all match.
   */
  private static String normalizeForMatch(String name) {
    if (name == null) return "";
    return name.toLowerCase().replaceAll("[-_\\s]+", "");
  }

  private String getMatchingImageName(String name) {
    String normalizedRequest = normalizeForMatch(name);
    if (normalizedRequest.isEmpty()) return "";
    for (String image : userImages) {
      if (normalizeForMatch(image).equals(normalizedRequest)) {
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

    displayUserImage(name, timeout, true);

    return true;
  }

  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "interruptDisplayImageWithName()", onSuspendServiceCall = "interruptDisplayImageWithName()")
  public void displaySlideWithName(String presentationName, String imageName, long timeout) {
    if (presentationName.startsWith("\"") && presentationName.endsWith("\"")) {
      presentationName = presentationName.substring(1,presentationName.length()-1);
    }
    if (imageName.startsWith("\"") && imageName.endsWith("\"")) {
      imageName = imageName.substring(1,imageName.length()-1);
    }
    String path = getMediaPath(presentationSlidesDir) + File.separator + presentationName + File.separator + imageName;
    userImageDisplayed = true;
    displayImage(path, timeout, true, true, Color.WHITE);
    userImageDisplayed = false;
    recordMediaPlayback("image", timeout * 1000L);
  }

  //TODO: smarter interaction with timeout on suspend/resume? Currently will reset timer
  @TRADEService
  public void interruptDisplayImageWithName() {
    canvas.cancelSetImage();
    clearImage();
    //clearText();
    userImageDisplayed = false;
    continueIdleImageCycle();
  }

  private void displayUserImage(String imageName, long timeout) {
    displayUserImage(imageName, timeout, false);
  }
  private void displayIdleImage(String imageName, long timeout) {
    displayIdleImage(imageName,timeout,false);
  }
  private void displayUserImage(String imageName, long timeout, boolean blocking) {
    userImageDisplayed = true;
    //clearText();
    String path = getMediaPath(userImageDir) + File.separator + imageName;
    displayImage(path, timeout, blocking, true, Color.WHITE);
    userImageDisplayed = false;
    recordMediaPlayback("image", timeout * 1000L);
    continueIdleImageCycle();
  }
  private void displayIdleImage(String cycleEntry, long timeout, boolean blocking) {
    String path;
    if (cycleEntry.startsWith(USER_PREFIX)) {
      String imageName = cycleEntry.substring(USER_PREFIX.length());
      path = getMediaPath(userImageDir) + File.separator + imageName;
    } else {
      String imageName = cycleEntry.startsWith(IDLE_PREFIX) ? cycleEntry.substring(IDLE_PREFIX.length()) : cycleEntry;
      path = getMediaPath(idleImageDir) + File.separator + imageName;
    }
    displayImage(path, timeout, blocking, false, Color.BLACK);
  }

  private void displayImage(String path, long timeout, boolean blocking, boolean scaleToFit, Color backgroundColor) {
    if (path.equals(currentlyDisplayedImagePath) && !blocking) {
      return;
    }
    log.debug("displayImage(" + path + ")");
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
    currentlyDisplayedImagePath = path;
    idle = false;
    idleTimer = System.currentTimeMillis();
    canvas.setIdleNoRepaint(false);
    if (blocking) {
      canvas.setImageBlocking(image, timeout, scaleToFit, backgroundColor);
    } else {
      canvas.setImage(image, timeout, scaleToFit, backgroundColor);
    }
    canvas.refreshText();
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
    mediaPlayerComponent.mediaPlayer().media().start(path);
    videoPlaybackStartMs = System.currentTimeMillis();
    videoPlaying = true;
    log.info("[displayVideo] playing: " + path);
  }

  @TRADEService
  public boolean isMediaPlaying() {
    return videoPlaying;
  }

  @TRADEService
  public void cancelVideoPlayback() {
    if (videoPlaying && videoPlaybackStartMs > 0) {
      long durationMs = System.currentTimeMillis() - videoPlaybackStartMs;
      recordMediaPlayback("video", durationMs);
    }
    mediaPlayerComponent.mediaPlayer().controls().stop();
    videoPlaying = false;
  }

  /** Records intentional media playback for statistics (excludes idle image rotation). */
  private void recordMediaPlayback(String mediaType, long durationMs) {
    if (durationMs <= 0) return;
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("recordAvaStatisticsEvent").argTypes(AvaStatisticsEvent.class))
          .call(void.class, AvaStatisticsEvent.builder(AvaStatisticsEvent.Kind.MEDIA_PLAYBACK)
              .mediaType(mediaType)
              .durationMs(durationMs)
              .build());
    } catch (TRADEException e) {
      log.debug("[recordMediaPlayback] Could not record event: {}", e.getMessage());
    }
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
    currentlyDisplayedImagePath = "";
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
  }

  @TRADEService
  public void displayText(String text, long timeout) {
    log.debug("displayText {} {}", text, timeout);
    unsetIdle();
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
    if (utterance.getSpeaker().getName().equals("ava") && !utterance.getAddressee().getName().equals("webapp")) {
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
    } else if (utterance.getSpeaker().getName().equals("webapp") && utterance.getAddressee().getName().equals("ava")) {
      displayText("Received External Command...", 5000);

      // Removed from chatAdapter and AvaGuiAdapter - want to add just this to either case?
      //try {
      //  TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("playAudioClip").argTypes(String.class));
      //  service.call(void.class,"ding.wav");
      //} catch (TRADEException e) {
      //  log.warn("[handleMessage] exception playing audio clip on Ava", e);
      //}
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

  /** Returns the list of uploaded image names currently used as idle images. */
  @TRADEService
  public List<String> getUserIdleImageNames() {
    return new ArrayList<>(userSelectedIdleImageNames);
  }

  /** Sets which uploaded images are used as idle images. Only image names that exist in userImages are stored. */
  @TRADEService
  public void setUserIdleImageNames(List<String> names) {
    if (names == null) names = List.of();
    userSelectedIdleImageNames = names.stream()
        .filter(name -> userImages.contains(name) && MediaTypes.isImage(name))
        .sorted()
        .collect(Collectors.toList());
    saveUserIdleImageSelection();
    buildIdleImageCycleList();
    continueIdleImageCycle();
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
    if (mediaPlayerComponent != null && videoPlaying) {
      mediaPlayerComponent.mediaPlayer().audio().setVolume(videoVolume);
    }
    return true;
  }

  @TRADEService
  public int getVideoVolume() {
    return videoVolume;
  }


  @TRADEService
  public boolean lowerVideoVolume() {
    int next = Math.max(0, videoVolume - VIDEO_VOLUME_STEP);
    return setVideoVolume(next);
  }

  @TRADEService
  public boolean raiseVideoVolume() {
    int next = Math.min(100, videoVolume + VIDEO_VOLUME_STEP);
    return setVideoVolume(next);
  }

  //Leaving temporarily for reference
  //@Action
  //@TRADEService
  //public void testLearnNewPresentationAction() {
  //  List<String> slideNames = new ArrayList<>();
  //  slideNames.add("slide 1.png");
  //  slideNames.add("Slide 2.png");
  //  slideNames.add("Slide3.png");
  //  learnNewPresentationAction("Introduction Presentation",slideNames,10);
  //}

  //Called in forgetTaughtAction if action starts with name "present"
  @TRADEService
  public void deletePresentationSlides(String presentationName) {
    if (presentationSlidesDir.isEmpty()) {
      log.error("[deletePresentationSlides] cannot delete slides if media dir is not set");
      return;
    }

    String path = getMediaPath(presentationSlidesDir) + File.separator + presentationName;
    File directory = new File(path);
    File[] files = directory.listFiles();
    if (files != null) {
      for (File file: files) {
        file.delete();
      }
    }
    directory.delete();
  }

  //Flexible on parameterization, e.g.:
  //  Can have N durations rather than 1
  //  Can provide presentation directory with sequentially named images rather than filename list
  //Return value could be a string or justification for more specific failure messages if desired
  @TRADEService
  public boolean learnNewPresentationAction(String presentationName, List<String> slideFilenames, long duration) {
    //Assumes these images are saved somewhere separate from userImages. Currently assuming presentation specific
    // subfolders under "presentationSlides" macro folder (for the purposes of deleting files when forgetting action)
    if (presentationSlidesDir.isEmpty()) {
      log.error("[learnNewPresentationAction] cannot save new presentation action if media dir is not set");
      return false;
    }

    String[] presentationNameParts = presentationName.toLowerCase().split("\\s+");
    // If deleting the presentation action, I assume we then want to delete the associated files?
    //  There is no way to know when to do this unless the presentation actions have a specific
    //  naming pattern enforced which can be checked in "forgetTaughtAction"
    // Calling method from "forgetTaughtAction" to delete all associated slide files when matching on a "present<name>" action
    // TODO: The directory these files are saved to must match <name> in order to do this. This has to be enforced wherever
    //        the images are saved to file. (<name> being capitalized, camel case joined string)
    String safePresentationName = Arrays.stream(presentationNameParts).map(p -> Character.toUpperCase(p.charAt(0)) + p.substring(1)).collect(Collectors.joining());
    String actionName = "present" + safePresentationName;

    //Check all images can be loaded properly
    for (String filename: slideFilenames) {
      String path = getMediaPath(presentationSlidesDir) + File.separator + safePresentationName + File.separator + filename;
      BufferedImage image = null;
      try {
        image = ImageIO.read(new File(path));
      } catch (IOException ex) {
        log.error("[learnNewPresentationAction] Error getting slide image " + path, ex);
      } catch (Exception ex) {
        log.error("[learnNewPresentationAction] Other error getting slide image " + path, ex);
      }
      if (image == null) {
        return false;
      }
    }

    // TODO: need teaching to be interactor/agentTeam specific here again - can have collisions if receiving instructions
    //      during this. Hoping the window is small enough that it practically won't happen.
    // Teach new presentation action at once:
    Predicate actionPredicate = Factory.createPredicate(actionName);
    try {
      // Start Action Learning
      new Thread(() -> {
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("learnAction").argTypes(Symbol.class, Predicate.class)).call(void.class, Factory.createSymbol("ava"), actionPredicate);
          } catch (TRADEException e) {
            //Error handling in threads not very safe for learning state
            log.error("[learnNewPresentationAction] exception teaching action");
          }
      }).start();
      TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForActionLearningStart").argTypes(Symbol.class, Predicate.class)).call(void.class, Factory.createSymbol("ava"),actionPredicate);

      // Teach Action steps
      for (String slideFilename : slideFilenames) {
        Predicate displaySlidePredicate = Factory.createPredicate("displaySlideWithName","ava","\""+safePresentationName+"\"","\""+slideFilename+"\"",String.valueOf(duration));

        // Need to synchronize properly around learning state updates
        ExecutorService executor = Executors.newSingleThreadExecutor();
        Future<?> future = executor.submit(() -> {
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForLearningStateUpdate").argTypes(Symbol.class, Predicate.class)).call(void.class, Factory.createSymbol("ava"), actionPredicate);
          } catch (TRADEException e) {
            log.error("[learnNewPresentationAction] exception teaching action");
          }
        });

        // Submit display image goal as learning step
        long gid = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class, ExecutionType.class, PriorityTier.class)).call(Long.class, displaySlidePredicate, ExecutionType.ACT, PriorityTier.NORMAL);
        //TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class)).call(GoalStatus.class, gid);

        future.get(5,TimeUnit.SECONDS); //What is an appropriate amount of time to wait?

      }
    } catch (Exception e) {
      log.error("[learnNewPresentationAction] exception teaching action");
      return false;
    } finally {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("endActionLearning").argTypes(Symbol.class, Predicate.class)).call(void.class, Factory.createSymbol("ava"), actionPredicate);
      } catch (TRADEException e) {
        log.error("[learnNewPresentationAction] exception ending learning");
      }
    }

    return true;
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
    options.add(Option.builder("videoAudioDevice").hasArg().desc("ALSA audio device for video playback (e.g. hdmi:CARD=PCH,DEV=0)").build());
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
      presentationSlidesDir = mediaDir + File.separator + "presentationSlides";
    }
    if (cmdLine.hasOption("videoVolume")) {
      videoVolume = Integer.parseInt(cmdLine.getOptionValue("videoVolume"));
    }
    if (cmdLine.hasOption("videoAudioDevice")) {
      videoAudioDevice = cmdLine.getOptionValue("videoAudioDevice");
    }
  }

  @Override
  public void shutdownComponent() {
    displayText("shutting down...");
    refreshExecutor.shutdown();
    if (mediaPlayerComponent != null) {
      mediaPlayerComponent.release();
    }
    if (mediaPlayerFactory != null) {
      mediaPlayerFactory.release();
    }
    canvas = null;
  }
}