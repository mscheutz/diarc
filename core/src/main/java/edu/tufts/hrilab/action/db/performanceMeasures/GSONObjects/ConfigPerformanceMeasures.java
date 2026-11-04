/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects;

public class ConfigPerformanceMeasures {
  int experience;
  int success;
  double successTimeMean;
  double successTimeVar;
  double successSumSqDiff;
  double failureTimeMean;
  double failureTimeVar;
  double failureSumSqDiff;
  ConfigEffectMeasures effectMeasures;

  public int getExperience() {
    return experience;
  }

  public void setExperience(int exp) {
    experience = exp;
  }

  public int getSuccess() {
    return success;
  }

  public void setSuccess(int succ) {
    success = succ;
  }

  public double getSuccessTimeMean() {
    return successTimeMean;
  }

  public void setSuccessTimeMean(double timeM) {
    successTimeMean = timeM;
  }

  public double getSuccessTimeVar() {
    return successTimeVar;
  }

  public void setSuccessTimeVar(double timeV) {
    successTimeVar = timeV;
  }

  public double getSuccessSumSqDiff() {
    return successSumSqDiff;
  }

  public void setSuccessSumSqDiff(double val) {
    successSumSqDiff = val;
  }

  public double getFailureTimeMean() {
    return failureTimeMean;
  }

  public void setFailureTimeMean(double timeM) {
    failureTimeMean = timeM;
  }

  public double getFailureTimeVar() {
    return failureTimeVar;
  }

  public void setFailureTimeVar(double timeVar) {
    failureTimeVar = timeVar;
  }

  public double getFailureSumSqDiff() {
    return failureSumSqDiff;
  }

  public void setFailureSumSqDiff(double val) {
    failureSumSqDiff = val;
  }

  public ConfigEffectMeasures getEffectMeasures() {
    return effectMeasures;
  }


  public void setEffectMeasures(ConfigEffectMeasures effMeasures) {
    effectMeasures = effMeasures;
  }

  public ConfigPerformanceMeasures() {
    effectMeasures = new ConfigEffectMeasures();
  }

  public ConfigPerformanceMeasures(int experience, int success, double timeMean, double timeVar, double sumSqDiff, ConfigEffectMeasures effMeasures) {
    this.experience = experience;
    this.success = success;
    this.successTimeMean = timeMean;
    this.successTimeVar = timeVar;
    this.successSumSqDiff = sumSqDiff;
    this.effectMeasures = effMeasures;
  }
}
