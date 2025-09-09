package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.Gson;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.ava.message.response.HTTPMapResponse;
import edu.tufts.hrilab.ava.message.response.HTTPMapsResponse;
import edu.tufts.hrilab.ava.message.response.ListTagsResponse;
import edu.tufts.hrilab.ava.message.response.MapMetadataResponse;
import edu.tufts.hrilab.ava.message.response.Obstacle;
import edu.tufts.hrilab.ava.message.response.RMSListTagsResponse;
import edu.tufts.hrilab.ava.message.response.RMSMapMetadataResponse;
import edu.tufts.hrilab.ava.message.response.Tag;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.Semaphore;
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
  private List<BufferedImage> mapImages = new ArrayList<>();
  private List<RMSMapMetadataResponse> mapMetadatas = new ArrayList<>();
  private int numMaps = 0;
  private List<String> dockNames = new ArrayList<>();
  private Thread sleepForDurationThread = null;
  private Symbol speaker = Factory.createSymbol("local_user");

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
    return new ConditionJustification(true);
  }

  @Override
  public Justification follow() {
    return new ConditionJustification(true);
  }

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
  @Action
  public void triggerWW() {
    if (ww_process_os == null) {
      log.debug("[triggerWW] python subprocess input stream is null");
      return;
    }
    BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(ww_process_os));
    //TODO: hook up to QA where desired
    log.info("[triggerWW] in method");
    //shouldDing = false;
    try {
      writer.write("hello\n");
      writer.flush();
    } catch (IOException e) {
      log.error("[triggerWW] error signalling WW trigger");
    }
  }

  @TRADEService
  public void playAudioClip(String fileName) {
    try {
      String homeDir = System.getProperty("user.home");
      File audioFile = new File(Paths.get(homeDir, ".diarc","ava",fileName).toString());
      AudioInputStream audioStream = AudioSystem.getAudioInputStream(audioFile);
      AudioFormat format = audioStream.getFormat();
      DataLine.Info info = new DataLine.Info(Clip.class, format);
      Clip audioClip = (Clip) AudioSystem.getLine(info);
      audioClip.open(audioStream);
      FloatControl control = (FloatControl) audioClip.getControl(FloatControl.Type.MASTER_GAIN);
      control.setValue(-12f);
      audioClip.start();
      Thread.sleep(1000); //TODO: implement LineListener and actually block a correct amount of time
      audioClip.close();
      audioStream.close();
    } catch (Exception e) {
      log.error("[listenForWWandASR] error playing ww detection audio clip", e);
    }
  }
  @Override
  public void interruptListenForWWandASR() {
    listenForWW = false;
  }
  @Override
  public void listenForWWandASR() {
    listenForWW = true;
    //TODO: Configure paths/get resources better
    //args:--whisper_model, --ww_model_path, --ww_detection_thresh
    String home = System.getProperty("user.home");
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py");
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", home+"/.diarc/python/ww/run_ww_and_asr.py", "--whisper_model", "base.en");
    processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT); //For debugging
    try {
      Process process = processBuilder.start();
      InputStream is = process.getInputStream();
      ww_process_os = process.getOutputStream();
      String result = "";
      while (listenForWW) {
        while (result.isEmpty()) {
          //Wait for WW detection
          while (is.available() == 0 && listenForWW) {
            Thread.sleep(100);
          }
          if (!listenForWW) {
            break;
          }
          //Clear stream buffer of message indicating WW detection
          StringBuilder s = new StringBuilder();
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          log.info(s.toString());
          //Display on screen that we are listening
          try {
            TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class, Long.class));
            service.call(void.class,"Understanding speech",15);
          } catch (TRADEException e) {
            log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
          }
          if (shouldDing) {
            playAudioClip("ding.wav");
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("stopUtterance")).call(void.class);
            } catch (TRADEException e) {
              log.debug("[listenForWWAndASR] exception calling stopUtterance", e);
            }
          } else {
            shouldDing = true;
          }

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
          //If ASR timed out, loop back to WW detection
          if (result.equals("<unk>\n")) {
            playAudioClip("negativeASRResult.wav");
            result = "";
            try {
              TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
              service.call(void.class);
            } catch (TRADEException e) {
              log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
            }
          } else {
            playAudioClip("positiveASRResult.wav");
            try {
              TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
              service.call(void.class,"Recognized Speech: " + result);
            } catch (TRADEException e) {
              log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
            }
            new Thread(() -> {
              try {
                Thread.sleep(5000);
              } catch (InterruptedException e) {
                log.error("[listenForWWandASR] exception sleeping in clearDisplay thread");
              }
              try {
                TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
                service.call(void.class);
              } catch (TRADEException e) {
                log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
              }
            }).start();
          }
        }
        if (!result.isEmpty()) {
          result = result.trim().toLowerCase();
          result = result.replaceAll("\\p{Punct}", "");
          Utterance utterance = new Utterance.Builder()
                  .setSpeaker(speaker)
                  .setAddressee(Factory.createSymbol("ava"))
                  .setWords(Arrays.asList(result.trim().split(" ")))
                  .setUtteranceType(UtteranceType.UNKNOWN)
                  .setIsInputUtterance(true)
                  .build();
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

  @Override
  public Symbol getDockRefId() {
    return Factory.createSymbol("location_0:location");
  }
  @Override
  public void waitForReadyAck() {
    String home = System.getProperty("user.home");
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", "-u", home+"/.diarc/python/ww/run_ready_ww.py");
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
}