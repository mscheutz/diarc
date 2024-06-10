package edu.tufts.hrilab.grasp;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.vision.common.swig.CommonModule;
import edu.tufts.hrilab.vision.grasp.swig.GraspDetectorModule;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.vecmath.Matrix4d;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

public class GraspComponent extends DiarcComponent {

  static {
    System.loadLibrary("common");
    System.loadLibrary("grasp");
  }

  /**
   * Default path to vision resources (configs).
   */
  private String defaultConfigPath = "config/edu/tufts/hrilab/vision";
  private String log4cxxConfig = "log4cxx.txt";

  @Override
  protected void init() {

    URL url = Resources.getResource(defaultConfigPath + "/logging", log4cxxConfig);
    if (url != null) {
      CommonModule.setLoggingConfiguration(url.getPath());
    } else {
      log.error("Could not find log4cxx configuration file: {}.", log4cxxConfig);
    }
    sandbox();
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
    GraspDetectorModule.calculateGraspPoses(mo);
    return new ArrayList<>();
  }

  private void sandbox() {
    MemoryObject mo = new MemoryObject();
    mo.setBaseTransform(new Matrix4d(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15));
    calculateGraspOptions(mo);
  }
}
