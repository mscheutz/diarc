package edu.tufts.hrilab.grasp;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.grasp.swig.GraspDetector;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GraspComponent extends DiarcComponent {

  static {
    System.loadLibrary("grasp");
  }

  /**
   * SWIG wrapped native grasp detector.
   */
  private GraspDetector nativeDetector;

  @Override
  protected void init() {
    GraspDetector nativeDetector = new GraspDetector();
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    return new ArrayList<>();
  }

  @TRADEService
  public List<Grasp> calculateGraspOptions(Symbol refId) {
    return calculateGraspOptions(refId, new ArrayList<>());
  }

  @TRADEService
  public List<Grasp> calculateGraspOptions(Symbol refId, List<? extends Term> constraints) {
    MemoryObject mo = null;
    try {
      //this is pretty messy
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      mo = tsi.call(MemoryObject.class, refId, MemoryObject.class);
    } catch (TRADEException e) {
      log.error("[moveTo] exception getting memory object from reference, returning null", e);
    }

    return calculateGraspOptions(mo, constraints);
  }

  public List<Grasp> calculateGraspOptions(MemoryObject mo) {
    return calculateGraspOptions(mo, new ArrayList<>());
  }

  public List<Grasp> calculateGraspOptions(MemoryObject mo, List<? extends Term> constraints) {
//    nativeDetector.calculateGraspOptions(mo);
    return new ArrayList<>();
  }
}
