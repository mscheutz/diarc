package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.ava.message.response.Obstacle;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static java.lang.System.getProperty;

public class MockAvaComponent extends DiarcComponent implements AvaInterface {

  boolean toggle = false;

  private final Lock chargingLock = new ReentrantLock();
  private final Condition chargingTerminatedCondition = chargingLock.newCondition();

  @Override
  public void init() {
    // get notified when display services become available
    try {
      TRADE.registerNotificationNowOrWhenJoined(this, new TRADEServiceConstraints().name("displayRobot").argTypes(), "displayRobot");
    } catch (TRADEException e) {
      log.error("Error registering for displayRobot service notification.", e);
    }
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
    log.info("[getPose] called.");
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

  @Override
  public void listenForWWandASR() {
    //TODO: Configure paths/get resources better
    //args:--whisper_model, --ww_model_path, --ww_detection_thresh
    String home = System.getProperty("user.home");
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", home+"/.diarc/python/ww/run_ww_and_asr.py");
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", home+"/.diarc/python/ww/run_ww_and_asr.py", "--whisper_model", "base.en");
    processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT); //For debugging
    try {
      Process process = processBuilder.start();
      InputStream is = process.getInputStream();
      String result = "";
      while (true) {
        while (result.isEmpty()) {
          //Wait for WW detection
          log.info("waiting for wakeword");
          while (is.available() == 0) {
            Thread.sleep(100);
          }
          //Clear stream buffer of message indicating WW detection
          StringBuilder s = new StringBuilder();
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          log.info(s.toString());
          //Display on screen that we are listening
          try {
            TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
            service.call(void.class,"Understanding speech");
          } catch (TRADEException e) {
            log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
          }

          //Wait for ASR result:
          log.info("waiting for ASR result");
          while (is.available() == 0) {
            Thread.sleep(100);
          }
          //Clear listening text from screen (or could write understood results)
          try {
            TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
            service.call(void.class);
          } catch (TRADEException e) {
            log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
          }
          //Get ASR result (using <unk> to signal ASR timed out before any results were detected)
          s.setLength(0);
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          result = s.toString();
          //If ASR timed out, loop back to WW detection
          if (result.equals("<unk>\n")) {
            result = "";
          }
        }
        result = result.trim().toLowerCase();
        result = result.replaceAll("\\p{Punct}", "");
        Utterance utterance = new Utterance.Builder()
                .setSpeaker(Factory.createSymbol("user"))
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
      //String result = new String(is.readAllBytes());
      //int exitCode = process.waitFor();
      //log.info(String.valueOf(exitCode));
      //log.info(result);
    } catch (IOException | InterruptedException e) {
      log.error("error running process",e);
    }
  }
}

