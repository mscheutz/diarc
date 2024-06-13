package edu.tufts.hrilab.grasp;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.vision.common.swig.CommonModule;
import edu.tufts.hrilab.vision.grasp.swig.GraspDetectorModule;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

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
    shouldRunExecutionLoop = false;
  }

  @Override
  protected void executionLoop() {
    if (TRADE.getAvailableServices(new TRADEServiceConstraints().name("getTypeId").argTypes(Symbol.class)).isEmpty()) {
      return;
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
    double[][] cloud = mo.getPointCloud();
    int numPoints = cloud.length;
    double[] cloudArray = new double[numPoints * 3];
    for (int n = 0; n < numPoints; ++n) {
      for (int dim = 0; dim < 3; ++dim) {
        cloudArray[dim + n*3] = cloud[n][dim];
      }
    }

    List<Grasp> graspOptions = GraspDetectorModule.calculateGraspPoses(mo.getBaseTransformArray(), cloudArray, mo.getImageWidth(), mo.getImageHeight());
    return graspOptions;
  }

  private void sandbox() {
//    MemoryObject mo = new MemoryObject();
//    mo.setBaseTransform(new Matrix4d(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15));
//    mo.setNumPoints(3);
//    mo.addPoint(0, 1,2,3);
//    mo.addPoint(1, 4,5,6);
//    mo.addPoint(2, 7,8,9);
//    mo.setImageSize(1,3);

    try {
      long typeId = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTypeId").argTypes(Symbol.class).notInGroups("physobj")).call(Long.class, Factory.createPredicate("object(X)"));
      List<MemoryObject> mos = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTokens").argTypes(Long.class).notInGroups("physobj")).call(List.class, typeId);

      for (MemoryObject mo : mos) {
        calculateGraspOptions(mo);
      }
    } catch (TRADEException e) {
      log.error("Error in sandbox", e);
    }
  }
}
