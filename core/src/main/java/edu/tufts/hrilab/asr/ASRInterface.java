/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.asr;

import ai.thinkingrobots.trade.TRADEService;

public interface ASRInterface {
  @TRADEService
  public String transcribeSpeech (float samples[]);
  @TRADEService
  public void transcribeAndReportSpeech (float samples[]);
  @TRADEService
  public void disableMicrophone ();
  @TRADEService
  public void enableMicrophone ();
}