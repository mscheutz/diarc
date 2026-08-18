package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.Gson;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.ava.message.response.HTTPMapResponse;
import edu.tufts.hrilab.ava.message.response.HTTPMapsResponse;
import edu.tufts.hrilab.ava.message.response.ListTagsResponse;
import edu.tufts.hrilab.ava.message.response.MapMetadataResponse;
import edu.tufts.hrilab.ava.message.response.Obstacle;
import edu.tufts.hrilab.ava.message.response.RMSListTagsResponse;
import edu.tufts.hrilab.ava.message.response.RMSMapMetadataResponse;
import edu.tufts.hrilab.ava.message.response.Tag;
import edu.tufts.hrilab.ava.stats.AvaStatisticsEvent;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.util.Convert;
import edu.tufts.hrilab.util.RotationHelpers;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.imageio.ImageIO;
import javax.sound.sampled.*;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.file.Paths;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static java.lang.Double.parseDouble;

public class MockAvaComponent extends DiarcComponent implements AvaInterface {

  boolean toggle = false;

  private final Lock chargingLock = new ReentrantLock();
  private final Condition chargingTerminatedCondition = chargingLock.newCondition();
  private OutputStream ww_process_os = null;
  private boolean shouldDing = true;
  private OutputStream ready_process_os = null;
  private boolean shouldDingReady = true;
  private boolean listenForWW = true;
  private final AtomicBoolean forcedWWInFlight = new AtomicBoolean(false);
  private final AtomicBoolean micMuted = new AtomicBoolean(false);
  private final AtomicInteger micMuteCount = new AtomicInteger(0);
  private final AtomicBoolean asrSessionTainted = new AtomicBoolean(false);
  private List<BufferedImage> mapImages = new ArrayList<>();
  private List<RMSMapMetadataResponse> mapMetadatas = new ArrayList<>();
  private int numMaps = 0;
  private List<String> dockNames = new ArrayList<>();
  private Thread sleepForDurationThread = null;
  private Symbol speaker = Factory.createSymbol("local_user");
  private int mixerID = -1;
  //boolean haveDisplay = true;
  private Lock displayLock = new ReentrantLock();
  private Condition telepresenceCondition = displayLock.newCondition();
  private final Set<TRADEServiceInfo> statisticsSubscribers = Collections.synchronizedSet(new HashSet<>());
  private boolean shouldFollow = false;

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("matchMixer")) {
      String matchString = String.join(" ", cmdLine.getOptionValues("matchMixer"));
      mixerID = ConfigurableMicrophone.matchMixer(matchString);
      log.debug("Matching mixer string \"" + matchString + "\"");
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("matchMixer").hasArgs().argName("name").desc("String to match for mixer in case of device re-ordering. Do not quote argument if spaces needed.").build());
    return options;
  }

  @TRADEService
  public void registerForAvaStatisticsNotifications(TRADEServiceInfo callback) {
    if (statisticsSubscribers.stream().anyMatch(c -> c.matches(callback))) {
      log.warn("[registerForAvaStatisticsNotifications] attempt to register duplicate callback, not adding");
    } else {
      statisticsSubscribers.add(callback);
    }
  }

  @TRADEService
  public void unregisterForAvaStatisticsNotifications(TRADEServiceInfo callback) {
    statisticsSubscribers.remove(callback);
  }

  private void notifyStatisticsSubscribers(AvaStatisticsEvent event) {
    List<TRADEServiceInfo> toRemove = new ArrayList<>();
    synchronized (statisticsSubscribers) {
      for (TRADEServiceInfo subscriber : statisticsSubscribers) {
        try {
          subscriber.call(void.class, event);
        } catch (Exception e) {
          log.error("Could not notify statistics subscriber: {}", subscriber, e);
          // TODO(pete): is matching exception text the right subscriber-cleanup signal?
          if (e.getMessage() == null || e.getMessage().startsWith("Service not available any longer") || (e.getMessage() != null && e.getMessage().contains("tsi") && e.getMessage().contains("null"))) {
            toRemove.add(subscriber);
          }
        }
      }
    }
    for (TRADEServiceInfo s : toRemove) {
      statisticsSubscribers.remove(s);
    }
  }

  @Override
  public void init() {
    // get notified when display services become available
    try {
      TRADE.registerNotificationNowOrWhenJoined(this, new TRADEServiceConstraints().name("displayRobot").argTypes(), "displayRobot");
    } catch (TRADEException e) {
      log.error("Error registering for displayRobot service notification.", e);
    }

    loadMapMetadata();

    //reach out and add references to the map component
    //todo: (pete) this stinks. we shouldn't have to be reaching out like this. can we just share the api with a specialized map component instead?
    TRADEServiceInfo addReferenceToMapService;
    try {
      addReferenceToMapService = TRADE.getAvailableService(new TRADEServiceConstraints().name("addReferencesDirectly").argTypes(Map.class));
    } catch (TRADEException e) {
      log.error("Error getting addReferencesDirectly service", e);
      return;
    }

    Map<String, Pair<Point3d, Quat4d>> tags = getAllTags();
    adjustDockTags(tags, dockNames);
    try {
      addReferenceToMapService.call(void.class, tags);
    } catch (TRADEException e) {
      log.error("[init] error calling addReferencesDirectly");
    }

    ////todo: (pete) should go in a consultant? assumes unique names? can only handle one dock?
    ////todo: (pete) it really feels like the "dock-ness" of a location should be a property?
    //if (dockNames.size() > 1) {
    //  log.error("[init] multiple tags labeled as dock in the map. only one dock currently supported");
    //} else if (dockNames.isEmpty()) {
    //  log.error("[init] no dock marked in map! a dock is required.");
    //} else {

    //  TRADEServiceInfo getReferencesService;
    //  try {
    //    getReferencesService = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapReferences").argTypes());
    //  } catch (TRADEException e) {
    //    log.error("[init] error getting getMapReferences service ", e);
    //    return;
    //  }

    //  Map<Symbol, PoseReference> poseReferences = new HashMap<>();
    //  try {
    //    poseReferences = getReferencesService.call(Map.class);
    //  } catch (TRADEException e) {
    //    log.error("[init] error calling getMapReferences");
    //  }

    //  String dockName = dockNames.get(0);
    //  for (Symbol refId : poseReferences.keySet()) {
    //    //abosultely shouldn't be blind indexing into the ref to get the name!!!!!!!
    //    if (poseReferences.get(refId).properties.get(0).getName().equals(dockName)) {
    //      dockLocation = refId;
    //    }
    //  }
    //}
  }

  @Override
  protected void postRegistrationInit() {
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
                                                new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class),
                                                "onGoalManagerRegistrationAvailable");
    } catch (TRADEException e) {
      log.error("[postRegistrationInit] Error registering for goal manager availability.", e);
    }
  }

  /**
   * Called by TRADE when registerGoalManagerNotification becomes available (now or when joined).
   * Registers our callback for GOAL_STATUS_CHANGE notifications.
   */
  @TRADEService
  public void onGoalManagerRegistrationAvailable(TRADEServiceInfo registerService) throws TRADEException {
    TRADEServiceInfo myHandler = getMyService("hideListeningIndicatorGoalListener", GoalInfo.class);
    registerService.call(void.class, myHandler, NotificationType.GOAL_STATUS_CHANGE);
    log.info("[onGoalManagerRegistrationAvailable] Registered for goal status notifications.");
  }

  // Ava-only listening indicator: drop the robot head when a real terminal task action is
  // submitted (e.g. "go to chair 1"), so the head goes away once we stop listening to go
  // do something. Keep it up for replies and for focused-context entries that start an
  // interactive instruction dialogue (scheduleTask, learnTaskSequence) -- the robot is
  // still asking the user for info there.
  @TRADEService
  public void hideListeningIndicatorGoalListener(GoalInfo info) throws TRADEException {
    String goalname = info.goal == null ? info.unboundGoal.getName() : info.goal.getName();
    switch (goalname) {
      case "generateResponseFromString":
      case "updateDialogueHistoryFromString":
      case "scheduleTask":
      case "learnTaskSequence":
        return;
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("hideListeningIndicator")).call(void.class);
    } catch (TRADEException e) {
      log.error("[hideListeningIndicatorGoalListener] unable to call hideListeningIndicator", e);
    }
  }

  //@Override
  //protected void postRegistrationInit() {
  //  // register to get notified of "registerForDialogueHistoryNotifications" service
  //  try {
  //    TRADE.registerNotificationNowOrWhenJoined(this, new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications").argTypes(TRADEServiceInfo.class), "registerDialogueCallback");
  //  } catch (TRADEException e) {
  //    log.error("Error trying to register for registerForDialogueHistoryNotifications service.", e);
  //  }
  //}

  @Override
  public void displayRobot(TRADEServiceInfo displayService) {
    try {
      displayService.call(void.class);
    } catch (TRADEException e) {
      log.error("Error displaying robot", e);
    }
  }

  @Override
  public Justification goToDock() {
    log.info("[goToDock] called.");
    return new ConditionJustification(true);
  }

  @Override
  public Justification goHomePrimitive() {
    log.info("[goHome] called.");
    return new ConditionJustification(true);
  }

  @Override
  public Justification stopFollow() {
    if (shouldFollow) {
      shouldFollow = false;
      return new ConditionJustification(true);
    } else {
      return new ConditionJustification(false, Factory.createPredicate("following()"));
    }
  }

  @Override
  public Justification follow() {
    shouldFollow = true;
    while (shouldFollow) {
      try {
        Thread.sleep(500);
      } catch (InterruptedException e) {
        log.error("[follow] interrupted", e);
      }
    }

    log.debug("[follow] has been stopped.");
    return new ConditionJustification(true);  }

  @Override
  public Justification endSession() {
    log.info("[endSession] called.");
    return new ConditionJustification(true);
  }

  @Override
  public double getBatteryLevel() {
    log.debug("[getBatteryLevel] called.");
    return 50;
  }

  @Override
  public double getRuntime() {
    log.debug("[getRuntime] called.");
    return 0;
  }

  @Override
  public double getZLift() {
    log.info("[getZLift] called.");
    return 0;
  }

  @Override
  public Justification setZLift(double lift) {
    log.info("[setZLift] called. lift: {}.", lift);
    return new ConditionJustification(true);
  }

  @Override
  public double getCameraTilt() {
    log.info("[getCameraTilt] called.");
    return 0;
  }

  @Override
  public Justification setCameraTilt(double tilt) {
    log.info("[setCameraTilt] called. tilt: {}", tilt);
    return new ConditionJustification(true);
  }

  @Override
  public List<Obstacle> getDynamicObstacles() {
    log.info("[getDynamicObstacles] called.");
    return new ArrayList<>();
  }

  @Override
  public boolean isDocked() {
    log.info("[isDocked] called.");
    return false;
  }

  @Override
  public Justification lookAtObstacle(Obstacle obs) {
    log.info("[lookAtObstacle] called. obstacle: {}", obs);
    return new ConditionJustification(true);
  }

  @Override
  public Justification lookAt(double x, double y, double duration) {
    log.info("[lookAt] called. (x,y)=({},{}). durtation: {}", x, y, duration);
    return new ConditionJustification(true);
  }

  @Override
  public float[] getLaserScan() {
    log.info("[getLaserScan] called.");
    return new float[0];
  }

  @Override
  public float getLaserAngleIncrement() {
    log.info("[getLaserAngleIncrement] called.");
    return 0;
  }

  @Override
  public Pair<Point3d, Quat4d> getPose() {
    log.debug("[getPose] called.");
    return new MutablePair<>(new Point3d(), new Quat4d());
  }

  @Override
  public void setPose(Point3d location, Quat4d orientation) {
    log.info("[setPose] called.");
  }

  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    log.info("[goToLocation] called. location: {}", location);
    if (wait) {
      Util.Sleep(10000);
    }
    return new ConditionJustification(true);
  }

//  @Override
//  public Justification goToLocation(Symbol location) {
//    log.info("[goToLocation] called. location: {}", location);
//    Util.Sleep(10000);
//    return new ConditionJustification(true);
//  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    log.info("[goToLocation] called. desiredLocation: {} initialLocation: {}", desiredLocation, initialLocation);
    Util.Sleep(10000);
    return new ConditionJustification(true);
  }

  @Override
  public Justification goToLocation(Point3d location, Quat4d orientation, boolean wait) {
    log.warn("[goToLocation] not implemented.");
    return new ConditionJustification(false);
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) {
    log.warn("[goToLocation] not implemented.");
    return new ConditionJustification(false);
  }

  @Override
  public Justification approachLocation(Symbol location) {
    log.info("[approachLocation] called. location: {}", location);
    return new ConditionJustification(true);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    log.info("[approachLocation] called. desiredLocation: {}", desiredLocation);
    return new ConditionJustification(true);
  }

  @Override
  public Justification stop() {
    log.info("[stop] called.");
    return new ConditionJustification(true);
  }

  @Override
  public Justification isMoving() {
    log.info("[isMoving] called.");
    return new ConditionJustification(true);
  }

  @Override
  public List<Map<Variable, Symbol>> checkAt(Term locationTerm) {
    log.info("[checkAt] called. locationTerm: {}", locationTerm);
    List<Map<Variable, Symbol>> results = new ArrayList<>();
    results.add(new HashMap<>()); // i.e., true
    return results;
  }

  @Override
  public Symbol parseTime(Symbol input, Symbol amOrPm) {
    String inputString = input.toString();
    if (!inputString.contains(":")) {
      inputString = new StringBuilder(inputString).insert(inputString.length() - 2, ':').toString();
    }
    try {
      LocalTime localTime = LocalTime.parse(inputString, DateTimeFormatter.ofPattern("H:m"));
      if (amOrPm.getName().strip().equals("pm")) {
        if (!localTime.isAfter(LocalTime.parse("12:00", DateTimeFormatter.ofPattern("H:m")))) {
          localTime = localTime.plusHours(12);
        }
      }
      return Factory.createSymbol(localTime.format(DateTimeFormatter.ofPattern("HH-mm-ss")));
    } catch (DateTimeParseException e) {
      log.error("[parseTime] error parsing time input: %s %s".formatted(input, amOrPm));
    }
    return Factory.createSymbol("none");
  }

  @Override
  public Integer getWifiSignalStrength() {
    toggle = !toggle;
    if (toggle) {
      return 50;
    } else {
      return 75;
    }
  }

  @Override
  public Integer measureWifiRssi() {
    toggle = !toggle;
    return toggle ? -55 : -72;
  }

  @TRADEService
  @Action
  public String locationRefToName(Symbol ref) {
    if (ref == null) return "current location";
    try {
      PoseReference pr = TRADE.getAvailableService(
                      new TRADEServiceConstraints().name("getMapReference").argTypes(Symbol.class))
              .call(PoseReference.class, ref);
      if (pr != null && !pr.properties.isEmpty()) {
        return pr.properties.get(0).getName();
      }
    } catch (TRADEException e) {
      log.error("[locationRefToName] lookup failed for {}", ref, e);
    }
    return ref.toUntypedString();
  }

  @Override
  public String rssiQualityLabel(edu.tufts.hrilab.fol.Symbol rssi) {
    int v;
    try { v = Integer.parseInt(rssi.getName()); } catch (NumberFormatException e) { return "unknown"; }
    if (v >= -50) return "excellent";
    if (v >= -60) return "strong";
    if (v >= -70) return "good";
    if (v >= -80) return "weak";
    return "poor";
  }

  @Override
  public Integer measureTemperatureDegrees() {
    return 62 + (int) (Math.random() * 20);
  }

  @Override
  public void blockToCharge() {
    log.info("[blockToCharge] blocking on chargingTerminatedCondition");
    try {
      chargingLock.lock();
      chargingTerminatedCondition.await();
    } catch (InterruptedException e) {
      log.error("[blockToCharge] condition await interrupted", e);
    } finally {
      chargingLock.unlock();
    }
  }

  @Override
  public void interruptChargingBlock() {
    log.info("[interruptChargingBlock] signaling chargingTerminatedCondition");
    chargingLock.lock();
    chargingTerminatedCondition.signal();
    chargingLock.unlock();
  }

  @TRADEService
  public void disableMicrophone() {
    int count = micMuteCount.incrementAndGet();
    micMuted.set(true);
    asrSessionTainted.set(true);
    log.debug("[disableMicrophone] count={}; tainting any in-flight ASR session", count);
  }

  @TRADEService
  public void enableMicrophone() {
    int count = micMuteCount.decrementAndGet();
    if (count <= 0) {
      micMuteCount.set(0);
      micMuted.set(false);
    }
    log.debug("[enableMicrophone] count={} muted={}", Math.max(0, count), micMuted.get());
  }

  private void updateListeningIndicator(boolean visible) {
    log.info("[updateListeningIndicator] visible={}", visible);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("setListeningIndicator").argTypes(Boolean.class))
              .call(void.class, visible);
    } catch (TRADEException e) {
      log.debug("[updateListeningIndicator] setListeningIndicator unavailable");
    }
  }

  @TRADEService
  @Action
  public void triggerWW() {
    if (ww_process_os == null) {
      log.debug("[triggerWW] python subprocess input stream is null");
      return;
    }
    BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(ww_process_os));
    log.info("[triggerWW] in method");
    forcedWWInFlight.set(true);
    updateListeningIndicator(true);
    try {
      writer.write("hello\n");
      writer.flush();
    } catch (IOException e) {
      log.error("[triggerWW] error signalling WW trigger");
    }
  }

  @TRADEService
  @Action
  public void playAudioFile(String fileName) {
    log.info("[playAudioFile] {}", fileName);
    if (fileName.startsWith("\"") && fileName.endsWith("\"")) {
      fileName = fileName.substring(1,fileName.length()-1);
    }
    String homeDir = System.getProperty("user.home");
    String path = Paths.get(homeDir, ".diarc", "ava", fileName).toString();
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("playAudioClip").argTypes(String.class)).call(void.class, path);
    } catch (TRADEException e) {
      log.error("[playAudioFile] could not call TRADE service", e);
    }
  }

  public void playAudioClip(String fileName) {
    log.info("[playAudioClip] calling method on MaryTTSComponent");
    String homeDir = System.getProperty("user.home");
    String path = Paths.get(homeDir, ".diarc","ava",fileName).toString();
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("playAudioClip").argTypes(String.class)).call(void.class, path);
    } catch (TRADEException e) {
      log.debug("[playAudioClip] exception calling playAudioClip in MaryTTSComponent", e);
    }
  }
  @Override
  public void interruptListenForWWandASR() {
    listenForWW = false;
  }
  @Override
  public void listenForWWandASR() {
    listenForWW = true;
    //TODO: ARGS ARE OUTDATED FOR MOST OF THESE CONFIGURATIONS
    //args:
    // --model_size: "tiny", "base", "small", "medium", "large"
    // --ww_model_path: ~/.diarc/python/ww/models/hey_eyva_model_float32.tflite
    // --ww_detection_thresh: 0.8
    // --use_faster_whisper
    // --gpu
    // --asr_mode: 1 == local, 2 == remote
    // --endpoint: http://<ip>:5000/
    String home = System.getProperty("user.home");
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--use_faster_whisper");

    // ensemble: local moonshine for short, remote faster_whisper (marvin) for long
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--diarc", "--model_type", "0", "--model_size", "small", "--endpoint", "http://marvin:5000/");

    // local fasterwhisper small gpu
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--model_size","small","--use_faster_whisper","--asr_mode","1","--gpu");

    // remote only (faster_whisper)
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--diarc", "--model_type","2","--endpoint", "http://10.20.9.98:5000/");

    //dual local + remote
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py","--asr_mode","0","--endpoint", "http://localhost:5000/");

    //********If haven't updated ww env and script, use the below*******
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", home+"/.diarc/python/ww/run_ww_and_asr.py", "--whisper_model", "base.en");

    //dual local setup
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--gpu", "--model_type", "0", "--model_size", "medium", "--diarc");

    processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT); //For debugging
    try {
      Process process = processBuilder.start();
      InputStream is = process.getInputStream();
      ww_process_os = process.getOutputStream();
      String result = "";
      while (listenForWW) {
        while (result.isEmpty()) {
          //// Wake Word ////
          //Wait for WW detection
          while (is.available() == 0 && listenForWW) {
            Thread.sleep(100);
          }
          //WW detected, check if due to model trigger or external interrupt (mute or shutdown)
          if (!listenForWW) {
            break;
          }
          //Read message indicating WW detection from stream buffer
          StringBuilder s = new StringBuilder();
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          log.info(s.toString());

          asrSessionTainted.set(micMuted.get());
          if (!forcedWWInFlight.compareAndSet(true, false)) {
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("stopUtterance")).call(void.class);
            } catch (TRADEException e) {
              log.debug("[listenForWWAndASR] exception calling stopUtterance", e);
            }
            // User-WW interrupt: user is speaking to us right now. Flush mute /
            // taint state so the in-flight cascade-mute brackets don't drop the
            // user's actual interrupt utterance via asrSessionTainted.
            micMuteCount.set(0);
            micMuted.set(false);
            asrSessionTainted.set(false);
            // Real wake-word brings up the listening indicator; forced/auto rearms keep it
            // up via triggerWW, so it persists across the conversation instead of per turn.
            updateListeningIndicator(true);
          }

          //Display on screen that we are now listening using ASR
          displayText("Understanding speech", 10000);
          //Also play audio indicating listening status
          if (shouldDing) {
            playAudioClip("ding.wav");
          } else {
            shouldDing = true;
          }

          //// ASR ////
          //Wait for ASR result:
          log.info("waiting for ASR result");
          while (is.available() == 0) {
            Thread.sleep(100);
          }
          //Get ASR result (using <unk> to signal ASR timed out before any results were detected)
          s.setLength(0);
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          result = s.toString();
        }

        //Parse ASR result - form: <prefix> <speech>
        //  <unk> == base failure case - ASR timed out before detecting any voice activity or was unable to transcribe anything
        //  <cutoff> == false positive ww detection heuristics triggered on python side - attempt to parse, but fail silently if so
        //  <full> == base success case  - have full ASR results
        boolean cutoff_result = false;
        //If ASR timed out, loop back to WW detection
        if (!result.isEmpty()) {
          if (asrSessionTainted.compareAndSet(true, false)) {
            log.info("[listenForWWAndASR] dropping ASR result captured while mic muted: {}", result.trim());
            result = "";
            continue;
          }
          //  TODO: Do we want this distinction between vad triggering without any transcription vs no vad at all?
          if (result.equals("<unk>\n") || result.equals("<novad>\n")) {
            if (result.equals("<unk>\n")) {
              // ASR couldn't parse, play low toned beep and loop to top
              playAudioClip("negativeASRResult.wav");
            }
            try {
              TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
              service.call(void.class);
            } catch (TRADEException e) {
              log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
            }
            if (result.equals("<novad>\n")) {
              displayText("<No voice input detected>", 5000);
            }
            updateListeningIndicator(false);
            result = "";
            continue;
          } else if (result.startsWith("<cutoff> ")) {
            // ASR detected potential false positive - attempt to parse before handling
            result = result.substring(9);
            cutoff_result = true;
          } else {
            // Positive ASR detection, play high toned beep, display understood text, and attempt to parse
            if (result.startsWith("<full> ")) {
              result = result.substring(7);
            } else {
              log.warn("[listenForWWandASR] unknown or no result prefix detected, is run_ww_and_asr.py up to date?");
            }
            playAudioClip("positiveASRResult.wav");
            displayText("Recognized Speech: " + result, 5000);
          }

          // Reformat ASR result to prevent parser issues, create utterance
          result = result.trim().toLowerCase();
          result = result.replaceAll("\\p{Punct}", "");
          Utterance utterance = new Utterance.Builder()
                  .setSpeaker(speaker)
                  .setAddressee(Factory.createSymbol("ava"))
                  .setWords(Arrays.asList(result.trim().split(" ")))
                  .setUtteranceType(UtteranceType.UNKNOWN)
                  .setIsInputUtterance(true)
                  .build();

          // Experimental <cutoff> ww false positive effort //
          //Attempt to parse first, if input can't be parsed, then assume false positive ww detection in this case and do not
          // make noticeable to the user that something happened
          if (cutoff_result) {
            try {
              utterance = TRADE.getAvailableService(new TRADEServiceConstraints().name("parseUtterance").argTypes(Utterance.class)).call(Utterance.class, utterance);
            } catch (TRADEException ex) {
              log.error("[listenForWWAndASR] error calling parseUtterance", ex);
            }
            if (utterance.getType() == UtteranceType.UNKNOWN) {
              //TODO: How should this be phrased?
              displayText("<Input could not be understood and detected as potential background noise>", 10000);
              updateListeningIndicator(false);
              result = "";
              continue;
            } else {
              // Present successful parse as normal
              //TODO: how does this practically look when interacting with the robot?
              playAudioClip("positiveASRResult.wav");
              displayText("Recognized Speech: " + result, 5000);
            }
          }

          //Pass ASR result off to dialogue pipeline and loop to top to continue listening for further input
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
            log.info("Reported utterance \"" + result + "\"");
          } catch (TRADEException ex) {
            log.warn("Failed to report speech \"" + result + "\"");
          }

          result = "";
        }
      }
      is.close();
      ww_process_os.close();
      process.destroy();
      //String result = new String(is.readAllBytes());
      //int exitCode = process.waitFor();
      //log.info(String.valueOf(exitCode));
      //log.info(result);
    } catch (IOException | InterruptedException e) {
      log.error("error running process",e);
    }
  }

  private void displayText(String text, int duration) {
    try {
      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class, Long.class));
      service.call(void.class,text,duration);
    } catch (TRADEException e) {
      log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
    }
    //new Thread(() -> {
    //  try {
    //    Thread.sleep(duration);
    //  } catch (InterruptedException e) {
    //    log.error("[listenForWWandASR] exception sleeping in clearDisplay thread");
    //  }
    //  try {
    //    TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
    //    service.call(void.class);
    //  } catch (TRADEException e) {
    //    log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
    //  }
    //}).start();
  }

  @Override
  public Symbol getDockRefId() {
    return Factory.createSymbol("location_0:location");
  }
  @Override
  public void waitForReadyAck() {
    String home = System.getProperty("user.home");
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ready_ww.py");
    processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT); //For debugging
    try {
      Process process = processBuilder.start();
      InputStream is = process.getInputStream();
      ready_process_os = process.getOutputStream();
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
        service.call(void.class, "Please say \"ready\" when you are ready to go");
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
      //Wait for WW detection
      while (is.available() == 0) {
        Thread.sleep(100);
      }
      //Clear stream buffer of message indicating WW detection
      StringBuilder s = new StringBuilder();
      while (is.available() != 0) {
        s.append((char) is.read());
      }
      if (shouldDingReady) {
        playAudioClip("positiveASRResult.wav");
      }
      shouldDingReady = true;

      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
      is.close();
      ready_process_os.close();
      process.destroy();
    } catch (IOException | InterruptedException e) {
      log.error("error running process", e);
    }
    ready_process_os = null;
  }

  @Override
  public void interruptWaitForReadyAck() {
    shouldDingReady = false;
    if (ready_process_os == null) {
      log.debug("[interruptWaitForReadyAck] python subprocess input stream is null");
      return;
    }
    BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(ready_process_os));
    try {
      writer.write("hello\n");
      writer.flush();
    } catch (IOException e) {
      log.error("[interruptWaitForReadyAck] error signalling WW trigger");
    }
  }

  private void loadMapMetadata() {
    mapMetadatas = new ArrayList<>();
    Gson gson = new Gson();
    try {
      String homeDir = System.getProperty("user.home");
      BufferedReader reader = new BufferedReader((new InputStreamReader(new FileInputStream(Paths.get(homeDir, ".diarc","ava","map_metadata.txt").toString()))));
      HTTPMapsResponse mapsResponse = gson.fromJson(reader, HTTPMapsResponse.class);
      numMaps = mapsResponse.maps.size();
      for (var entry : mapsResponse.maps.entrySet()) {
        RMSMapMetadataResponse metadataResponse = new RMSMapMetadataResponse();
        metadataResponse.response = new MapMetadataResponse();
        HTTPMapResponse m = entry.getValue();
        metadataResponse.response.width = m.dimensions.cols;
        metadataResponse.response.height = m.dimensions.rows;
        double mapScaleX = m.dimensions.width / m.dimensions.cols;
        double mapScaleY = m.dimensions.height / m.dimensions.rows;
        metadataResponse.response.mapScaleX = mapScaleX;
        metadataResponse.response.mapScaleY = mapScaleY;
        metadataResponse.response.originX = m.position.x;
        metadataResponse.response.originY = m.position.y;
        mapMetadatas.add(metadataResponse);
      }
    } catch (FileNotFoundException e) {
      log.warn("[loadMapMetadata] could not find mock map metadata jsons in ~/.diarc, mapMetadatas will be empty", e);
    }

    mapImages = new ArrayList<>();
    try {
      String homeDir = System.getProperty("user.home");
      mapImages.add(ImageIO.read(new FileInputStream(Paths.get(homeDir, ".diarc","ava","map_tile_1.png").toString())));
      mapImages.add(ImageIO.read(new FileInputStream(Paths.get(homeDir, ".diarc","ava","map_tile_2.png").toString())));
      mapImages.add(ImageIO.read(new FileInputStream(Paths.get(homeDir, ".diarc","ava","map_tile_3.png").toString())));
      mapImages.add(ImageIO.read(new FileInputStream(Paths.get(homeDir, ".diarc","ava","map_tile_4.png").toString())));
    } catch (IOException e) {
      log.warn("[loadMapMetadata] could not find mock map pngs in ~/.diarc, mapImages will be empty", e);
    }
  }

  @TRADEService
  public List<BufferedImage> getMapImages() {
    return mapImages;
  }

  @TRADEService
  public List<RMSMapMetadataResponse> getMapMetadatas() {
    return mapMetadatas;
  }

  public Map<String, Pair<Point3d, Quat4d>> getAllTags() {
    ArrayList<RMSListTagsResponse> listTagsResponses = new ArrayList<>();
    try {
      for (int i = 1; i <= numMaps; i++) {
        Gson gson = new Gson();
        RMSListTagsResponse resObj = new RMSListTagsResponse();
        String homeDir = System.getProperty("user.home");
        String videoPath = Paths.get(homeDir, ".diarc", "ava", "tags_tile_" + i + ".txt").toString();
        BufferedReader reader = new BufferedReader((new InputStreamReader(new FileInputStream(videoPath))));
        resObj.response = gson.fromJson(reader, ListTagsResponse.class);
        listTagsResponses.add(resObj);
      }
    } catch (FileNotFoundException e) {
      log.warn("[getAllTags] could not find mock tag jsons in ~/.diarc directory");
    }

    if (listTagsResponses.isEmpty()) {
      log.warn("[getAllTags] unable to get maps, make sure getMapMetadata has been called");
      return null;
    }
    Map<String, Pair<Point3d, Quat4d>> toReturn = new HashMap<>();
    dockNames.clear();
    for (RMSListTagsResponse res : listTagsResponses) {
      for (Tag tag : res.response.tags.values()) {
        if ("Dock".equals(tag.attributes.get("TagType"))) {
          dockNames.add(tag.name);
        }
        Point3d point = new Point3d(parseDouble(tag.position.x), parseDouble(tag.position.y), 0);
        Quat4d quat4d = Convert.convertToQuat4d(parseDouble(tag.position.theta), 0, 0);
        toReturn.put(tag.name, new MutablePair<>(point, quat4d));
      }
    }
    return toReturn;
  }
  private void adjustDockTags(Map<String, Pair<Point3d, Quat4d>> allTags, List<String> dockTags) {
    for (String name : dockTags) {
      Pair<Point3d, Quat4d> pose = allTags.get(name);
      Vector3d orientationVec = RotationHelpers.quaternionToNormVect(pose.getRight());
      Point3d position = pose.getLeft();
      log.info("adjusting dock tag " + name + " with position " + pose.getLeft());
      position.add(orientationVec);
      Pair<Point3d, Quat4d> newPose = new MutablePair<>(position, pose.getRight());
      log.info("new position for " + name + " is " + newPose.getLeft());
      allTags.put(name, newPose);
    }
  }

  @Action
  @TRADEService
  public void interruptWaitForLearningGoalSubmissionOrQuestionAnswer(Symbol agent, Predicate learnedActionPredicate) {
    //Should only have to cancel one inner action of the two threads the action is waiting one - the action itself should cancel the other one
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("interruptWaitForLearningStateUpdate").argTypes(Symbol.class,Predicate.class)).call(void.class,agent,learnedActionPredicate);
    } catch (TRADEException e) {
      log.error("[interruptWaitForLearningGoalSubmissionOrQuestionAnswer] Exception canceling waiting for learning state change to unblock primitive", e);
    }
  }

  //TODO: where should this live? Kind of an abomination of an action which shouldn't exist in the first place and
  //      touches both action and dialogue. Placing here because it is ava demo specific for now?
  @OnInterrupt(
          onCancelServiceCall = "interruptWaitForLearningGoalSubmissionOrQuestionAnswer(?agent,?learnedActionPredicate)",
          onSuspendServiceCall = "interruptWaitForLearningGoalSubmissionOrQuestionAnswer(?agent,?learnedActionPredicate)"
  )
  @Action
  @TRADEService
  public Map<Variable, Symbol> waitForLearningGoalSubmissionOrQuestionAnswer(Symbol agent, String questionString, List<Term> responseForms, Predicate learnedActionPredicate) {
    Semaphore threadJoinSem = new Semaphore(0);
    ExecutorService questionAskingExecutor = Executors.newSingleThreadExecutor();
    Future<HashMap<Variable,Symbol>> future = questionAskingExecutor.submit(() -> {
      HashMap<Variable,Symbol> resp = null;
      try {
        resp = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForResponse").argTypes(String.class, List.class, Symbol.class)).call(HashMap.class,questionString, responseForms, agent);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception waiting for question response", e);
      }
      threadJoinSem.release();
      return resp;
    });

    final Boolean[] updateReceived = new Boolean[1];
    Thread learningStateThread = new Thread(() -> {
      try {
        updateReceived[0] = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForLearningStateUpdate").argTypes(Symbol.class, Predicate.class)).call(Boolean.class,agent,learnedActionPredicate);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception waiting for learning state change", e);
      }
      threadJoinSem.release();
    });
    learningStateThread.start();

    try {
      threadJoinSem.acquire();
    } catch (InterruptedException e) {
      log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] exception waiting for one of the two conditions to occur", e);
      return null;
    }

    if (future.isDone()) {
      HashMap<Variable, Symbol> questionResponse;
      try {
        questionResponse = future.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] exception getting question asking response bindings",e);
        return null;
      }
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("interruptWaitForLearningStateUpdate").argTypes(Symbol.class,Predicate.class)).call(void.class,agent,learnedActionPredicate);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception canceling waiting for learning state change", e);
      }
      learningStateThread.interrupt();
      return questionResponse;
    } else if (!learningStateThread.isAlive()) {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("cancelWaitForResponse").argTypes(String.class, List.class, Symbol.class)).call(Map.class,questionString, responseForms, agent);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception canceling waiting for question response", e);
      }
      questionAskingExecutor.shutdownNow();
      if (!updateReceived[0]) {
        log.warn("[waitForLearningGoalSubmissionOrQuestionAnswer] learning state listener interrupted but no question answer was detected");
        return null;
      }
      return new HashMap<>();
    } else {
      log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] semaphore signalled but neither thread terminated?");
      return null;
    }
  }

  @Override
  public void waitForDuration(long seconds) {
    log.info("[waitForDuration] sleeping for {} seconds", seconds);
    try {
      sleepForDurationThread = Thread.currentThread();
      Thread.sleep(seconds*1000);
    } catch (InterruptedException ignored) {}
    sleepForDurationThread = null;
    log.info("[waitForDuration] done sleeping");
  }

  @Override
  public void interruptWaitForDuration() {
    if (sleepForDurationThread != null) {
      sleepForDurationThread.interrupt();
    }
  }

  @Override
  public void enableAdminMode() {
    speaker = Factory.createSymbol("local_user");
  }

  @Override
  public void disableAdminMode() {
    speaker = Factory.createSymbol("local_visitor");
  }

  @TRADEService
  @Action
  public void switchDisplayToTRACS() {
    displayLock.lock();
    telepresenceCondition.signal();
    displayLock.unlock();
  }

  @OnInterrupt(onCancelServiceCall = "switchDisplayToTRACS()", onSuspendServiceCall = "switchDisplayToTRACS()")
  @TRADEService
  @Action
  public void switchDisplayToTelepresencePrimitive() {
    //Switch to telepresence
    String homeDir = System.getProperty("user.home");
    String scriptDir = Paths.get(homeDir, ".diarc").toString();
    String statusCommand = "/bin/bash " + scriptDir + "getDisplay.sh";
    String statusResult = callLocal(statusCommand);
    log.info("[switchDisplayToTelepresence] status result: " + statusResult);
    List<String> results = Arrays.stream(statusResult.split("MainVideoSource")).toList();
    int id;
    if (results.size() > 1) {
      id = Integer.valueOf(results.get(1).substring(1,1));
    } else {
      id = 0;
    }
    log.info("[switchDisplayToTelepresence] id of current display input: " + id);
    //TODO: Error handling based on current state due to external forces? What to do if already in telepresence display
    if (id != 1) {
      log.warn("[switchDisplayToTelepresence] already in telepresence display?");
    }
    String result = callLocal("/bin/bash " + scriptDir + "setDisplay1.sh");
    log.info("[switchDisplayToTelepresence] to telepresence result: " + result);
    //TODO: what is proper error handling/result when switch failed?

    //Wait until condition is signalled by switchDisplayToTRACS
    try {
      displayLock.lock();
      telepresenceCondition.await();
      displayLock.unlock();
    } catch (InterruptedException e) {
      log.error("[switchDisplayToTelepresence] interrupted waiting on telepresence condition",e);
    }

    //TODO: What if out of telepresence display externally? Won't trigger anything here
    //      Double check current display again?
    //Switch back to TRACS
    result = callLocal("/bin/bash " + scriptDir + "setDisplay2.sh");
    log.info("[switchDisplayToTelepresence] to TRACS result: " + result);
  }

  //@TRADEService
  //public void toggleDisplay() {
  //  String homeDir = System.getProperty("user.home");
  //  String scriptDir = Paths.get(homeDir, ".diarc").toString();
  //  String statusCommand = "/bin/bash " + scriptDir + "getDisplay.sh";
  //  String statusResult = callLocal(statusCommand);
  //  log.info("[toggleDisplay] status result: " + statusResult);
  //  List<String> results = Arrays.stream(statusResult.split("MainVideoSource")).toList();
  //  int id;
  //  if (results.size() > 1) {
  //    id = Integer.valueOf(results.get(1).substring(1,1));
  //  } else {
  //    id = 0;
  //  }
  //  log.info("[toggleDisplay] id of current display input: " + id);
  //  String command;
  //  if (id == 1) {
  //    command = "/bin/bash " + scriptDir + "setDisplay2.sh";
  //  } else {
  //    command = "/bin/bash " + scriptDir + "setDisplay1.sh";
  //  }
  //  String result = callLocal(command);
  //  log.info("[toggleDisplay] result: " + result);
  //  //TODO: what is proper error handling/result when switch failed?
  //  haveDisplay = !haveDisplay;
  //}
  //@TRADEService
  //public boolean getHaveDisplay() {
  //  return haveDisplay;
  //}

  private String callLocal(String command) {
    StringBuilder result = new StringBuilder();
    String s;
    try {
      Process p = Runtime.getRuntime().exec(command);
      BufferedReader br = new BufferedReader(
              new InputStreamReader(p.getInputStream()));
      while ((s = br.readLine()) != null)
        result.append(s);
      p.waitFor();
      log.info("[callLocal] p: " + p);
      p.destroy();
    } catch (Exception e) {
      log.error("[showTRACS] error changing display input");
    }
    return result.toString();
  }

  ///**
  // * Registers avaDisplayDialogueCallback as the callback service for dialog events.
  // */
  //@TRADEService
  //public void registerDialogueCallback(TRADEServiceInfo service) {
  //  TRADEServiceInfo myService = this.getMyService("avaDialogueCallback", Utterance.class, Boolean.class);
  //  try {
  //    service.call(void.class, myService);
  //  } catch (TRADEException e) {
  //    log.error("Error registering for dialogue history notifications.", e);
  //  }
  //}

  ////Cannot do this with current nlg.asl script because it interrupts the sayText, needs to be synchronous
  //@TRADEService
  //public void avaDialogueCallback(Utterance u, boolean newUtterance) {
  //  //TODO: Agent name matches should be generally pulled from specific demo configuration somehow
  //  if (u.getType().equals(UtteranceType.QUESTION) && (u.getAddressee().getName().equals("local_user") || u.getAddressee().getName().equals("local_visitor"))) {
  //    triggerWW();
  //  }
  //}
}