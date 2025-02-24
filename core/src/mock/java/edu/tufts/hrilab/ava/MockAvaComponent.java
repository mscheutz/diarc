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
import edu.tufts.hrilab.util.Util;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MockAvaComponent extends DiarcComponent implements AvaInterface {

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
  public Justification goHome() {
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
    log.info("[lookAt] called. (x,y)=({},{}). durtation: {}", x,y,duration);
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

  @Override
  public Justification goToLocation(Symbol location) {
    log.info("[goToLocation] called. location: {}", location);
    Util.Sleep(10000);
    return new ConditionJustification(true);
  }

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
    List<Map<Variable,Symbol>> results = new ArrayList<>();
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
}
