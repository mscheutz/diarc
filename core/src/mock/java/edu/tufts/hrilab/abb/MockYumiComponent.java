/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.abb.consultant.cognex.CognexConsultant;
import edu.tufts.hrilab.abb.consultant.cognex.CognexJob;
import edu.tufts.hrilab.abb.consultant.cognex.CognexReference;
import edu.tufts.hrilab.abb.consultant.cognex.CognexResult;
import edu.tufts.hrilab.abb.consultant.pose.RWSPoseConsultant;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MockYumiComponent extends DiarcComponent implements RWSRobotComponentInterface {
  protected RWSPoseConsultant rwsPoseConsultant;
  protected CognexConsultant cognexConsultant;

  long sleepTime = 500;
  private boolean errorState = false;
  private boolean cameraResultDetected = true;

  public void setSleepTime(long sleepTime) {
    this.sleepTime = sleepTime;
  }

  private void sleep(long ms) {
    try {
      Thread.sleep(ms);
    } catch (InterruptedException e) {
    }
  }

  @Override
  protected void init() {
    rwsPoseConsultant = new RWSPoseConsultant(this.getMyGroups());

    List<String> componentGroups = this.getMyGroups();
    componentGroups.add(rwsPoseConsultant.getKBName());
    try {
      TRADE.registerAllServices(rwsPoseConsultant, componentGroups);
    } catch (TRADEException e) {
      log.error("Error registering with trade ", e);
    }
  }

  public void setErrorState(boolean errorState) {
    this.errorState = errorState;
  }

  public void setCameraResultDetected(boolean cameraResultDetected) {
    this.cameraResultDetected = cameraResultDetected;
    this.errorState = !cameraResultDetected;
  }

  @Override
  public boolean goToPoseName(String poseName) {
    log.info("[goToPoseName] " + poseName);
    List<Symbol> candidateRefIds = rwsPoseConsultant.getRefIdsWithPropertyNamed(poseName);
    if (candidateRefIds.size() > 1) {
      log.warn("[goToPoseName] more than one pose named " + poseName + " picking one.");
    } else if (candidateRefIds.size() == 0) {
      log.error("[goToPoseName] no pose named " + poseName);
      return false;
    }
    goToCameraPose(candidateRefIds.get(0));
    return true;
  }

  @Override
  public void openGripperRapid() {
    log.info("[openGripper] opening gripper");
    sleep(sleepTime);
  }

  @Override
  public void closeGripperRapid() {
    log.info("[closeGripper] closing gripper");
    sleep(sleepTime);
  }

  @Override
  public void putDownItem(Symbol refId, Symbol pose) {
    log.info("[putDownObject] putting down item " + refId + " at pose " + pose);
    sleep(sleepTime);
  }

  @Override
  public Symbol addCameraPose(String propertyName, String poseString) {
    log.info("[addCameraPose] " + propertyName + " " + poseString);
    return rwsPoseConsultant.addCameraPose(propertyName, poseString);
  }

  @Override
  public void addDropoffPose(String propertyName, String cameraPoseName, String poseString) {
    log.info("[addDropoffPose] " + propertyName + " " + cameraPoseName + " " + poseString);
    rwsPoseConsultant.addDropoffPose(propertyName, cameraPoseName, poseString);
  }

  @Override
  public void goToCameraPose(Symbol locationRef) {
    String target = rwsPoseConsultant.getReference(locationRef).getPose();
    log.info("[goToCameraPose]: " + locationRef.getName() + " pose: " + target);
    sleep(sleepTime);
  }

  @Override
  public boolean defineGraspPointForDescriptor(Symbol refId, Symbol itemType) {
    log.info("[defineGraspPointForDescriptor] " + refId + " " + itemType);
    sleep(sleepTime);
    return true;
  }

  @Override
  public boolean perceiveEntityFromSymbol(Symbol refId) {
    log.info("[perceiveEntityFromSymbol] " + refId);
    sleep(sleepTime);
    CognexReference ref = cognexConsultant.getCognexReferenceForID(refId);

    CognexJob job = cognexConsultant.getCognexJobForCognexReference(ref);
    String jobName = job.getName();

    return cameraResultDetected;
  }

  @Override
  public void pickupItem(Symbol refId) {
    log.info("[pickupItem] " + refId);
    moveToObject(refId);
  }

  @Override
  public Justification cookItem(Symbol refId, Term time) {
    log.info("[cookItem] cooking " + refId + " for " + getTimeInSeconds(time) + " seconds ");
    sleep(getTimeInSeconds(time) * 1000);
    return new ConditionJustification(true);
  }

  @Override
  public Justification sauteItem(Symbol refId, Term time) {
    log.info("[saute] sauteing " + refId + " for " + getTimeInSeconds(time) + " seconds ");
    sleep(getTimeInSeconds(time) * 1000);
    return new ConditionJustification(true);
  }

  @Override
  public void calibrateGripper() {
    log.info("[calibrateGripper] calibrating");
  }

  @Override
  public void pourSauce() {
    log.info("[pourSauce] pouring sauce");
    sleep(sleepTime);
  }

  @Override
  public void setGraspPoint(Symbol itemType, String targetString) {
    log.info("[setGraspPoint] defining grasp point for " + itemType + " to be " + targetString);
    cognexConsultant.setGraspPointForJob(cognexConsultant.getJobForDescriptor(itemType.getName()), targetString);
  }

  @Override
  public void moveToObject(Symbol refId) {
    CognexReference cogRef = cognexConsultant.getCognexReferenceForID(refId);
    CognexJob job = cognexConsultant.getCognexJobForCognexReference(cogRef);
    String graspPoint = cognexConsultant.getGraspPoseForJob(job);
    String wobjString = String.valueOf(cogRef.result);
    log.info("moveToObject moving to object" + refId);
  }

  @Override
  public void setCognexConsultant(CognexConsultant cognexConsultant) {
    this.cognexConsultant = cognexConsultant;
  }

  @Override
  public boolean cancelCurrentRapidAction() {
    return true;
  }

  @Override
  public List<Map<Variable, Symbol>> checkError(Term t) {
    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    if (!errorState) {
      bindings.add(new HashMap<>());
    }
    return bindings;
  }

  private long getTimeInSeconds(Term time) {
    if (time.getArgs().size() != 2) {
      log.error("[turnPatty] " + time + " is an invalid time duration");
    }
    String timeUnits = time.get(1).getName();
    String timeString = time.get(0).getName();
    long timeInSeconds = -1;
    if (timeUnits.equals("minutes")) {
      timeInSeconds = Long.parseLong(timeString) * 60;
    } else if (timeUnits.equals("seconds")) {
      timeInSeconds = Long.parseLong(timeString);
    } else {
      log.error("[turnPatty] invalid time units provided " + timeUnits + " invalid. Only \"seconds\" and \"minutes\" supported");
    }
    return timeInSeconds;
  }

  @Override
  public void shutdownComponent() {
    return;
  }

}
