/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.capture.Camera;
import edu.tufts.hrilab.vision.consultant.VisionConsultant;
import edu.tufts.hrilab.vision.imgproc.AvailableImageProcessors;
import edu.tufts.hrilab.vision.tracker.AvailableTrackers;
import edu.tufts.hrilab.vision.detector.AvailableDetectors;
import edu.tufts.hrilab.vision.display.Display;
import edu.tufts.hrilab.vision.stm.AvailableSearchManagers;
import edu.tufts.hrilab.vision.common.swig.CommonModule;
import edu.tufts.hrilab.vision.learn.AvailableLearners;
import edu.tufts.hrilab.vision.stm.SearchManager;
import edu.tufts.hrilab.util.IdGenerator;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.awt.Dimension;
import java.util.Collection;
import java.util.List;

public class Vision {
  //static blocks are called in order of appearance in file
  //this must be called before any JNI methods can be used
  //(ie. must appear before other static blocks that use any JNI methods)

  static {
    System.loadLibrary("common");
  }

  public static IdGenerator typeIdGenerator = new IdGenerator();
  public static Camera camera;
  public static Display display;
  public static AvailableDetectors availableDetectors;
  public static AvailableTrackers availableTrackers;
  //objects that cameracontrolpanel "listens to" for display purposes.  also controls MO types, image proc types, display windows, and blob colors
  //TODO: figure out a cleaner way to make the image processor types available (ie, one factory)
  public static AvailableImageProcessors availableValidationProcessors;    //contains validation processors
  public static AvailableImageProcessors availableSaliencyProcessors; //contains saliency operators
  public static AvailableImageProcessors availableImageProcessors; //contains all other non-saliency and non-validation processors (eg, sift)
  public static AvailableSearchManagers availableSearchTypes;
  public static AvailableLearners availableLearners;
  public static VisionConsultant consultant;
  //GUIs
  public static FramesPerSecondPanel FPSpanel = new FramesPerSecondPanel(false);
  public static CameraControlPanel ccp;
  protected static Logger log = LoggerFactory.getLogger("com.vision.Vision");

  /**
   * Initializes capture pipeline, fills all detector, tracker, image processing
   * factories, and "built-in" memory object types and connects all their
   * interdependencies.
   *
   * @param displayFlag
   * @param detectorsConfig
   * @param trackersConfig
   * @param imgProcConfig
   * @param saliencyProcConfig
   * @param validationProcConfig
   * @param searchTypeConfig
   * @param learnersConfig
   * @param loggingConfig
   * @param runSearchTypes
   * @param showWindows
   */
  public static void init(final boolean displayFlag,
                          final String captureConfig, final String calibConfig,
                          final String detectorsConfig, final String trackersConfig, final String imgProcConfig,
                          final String saliencyProcConfig, final String validationProcConfig, final String searchTypeConfig,
                          final String learnersConfig,
                          final String loggingConfig,
                          final List<String> runSearchTypes,
                          final List<List<String>> showWindows,
                          final VisionConsultant consultant,Collection<String> groups) {
    //SHOULD HAPPEN FIRST IN THIS METHOD:
    CommonModule.setLoggingConfiguration(loggingConfig);

    // =====================================================================
    //init camera (and start it after initializing everthing else)
    camera = new Camera(displayFlag, captureConfig, calibConfig);

    // get image size
    Dimension imageSize = new Dimension();
    imageSize.height = camera.getImageHeight();
    imageSize.width = camera.getImageWidth();

    // =====================================================================
    //init all "type" objects - internally populated based on enums above
    availableImageProcessors = new AvailableImageProcessors(imageSize.width, imageSize.height, camera.hasStereo(), imgProcConfig);
    availableSaliencyProcessors = new AvailableImageProcessors(imageSize.width, imageSize.height, camera.hasStereo(), saliencyProcConfig);
    availableValidationProcessors = new AvailableImageProcessors(imageSize.width, imageSize.height, camera.hasStereo(), validationProcConfig);
    availableDetectors = new AvailableDetectors(imageSize.width, imageSize.height, detectorsConfig);
    availableTrackers = new AvailableTrackers(imageSize.width, imageSize.height, trackersConfig);
    availableSearchTypes = new AvailableSearchManagers(searchTypeConfig);
    availableLearners = new AvailableLearners(imageSize.width, imageSize.height, learnersConfig);

    // =====================================================================
    // vision consultant
    Vision.consultant = consultant;
    try {
      groups.add(consultant.getKBName());
      TRADE.registerAllServices(Vision.consultant,groups);
    } catch (TRADEException e) {
      log.error("Error trying to register vision consultant with TRADE.", e);
    }

    //======================================================================
    //init vision GUIs

    //EAK: this initializes the openCV high gui, and MUST be called before any
    //java guis are instantiated (see note in NativeVisionInterface.cpp)
    //Camera.initOpenCvHighGUI();
    boolean showCapture = false;//showWindows.contains(capture);
    boolean showDepth = false; //showWindows.contains(depth);
    for (List<String> showDescription : showWindows) {
      for (String descriptor : showDescription) {
        if (descriptor.equalsIgnoreCase("capture")) {
          showCapture = true;
        } else if (descriptor.equalsIgnoreCase("depth")) {
          showDepth = true;
        }
      }
    }
    ccp = new CameraControlPanel(displayFlag, showCapture, showDepth, camera.hasStereo(), camera.getUndistort());

    //start camera thread (after FPSpanel is created)
    camera.start();

    //init display and start display thread
    display = new Display();
    display.start();

    // =====================================================================
    //start requested SearchManager
    for (String searchString : runSearchTypes) {
      //get descriptors
      List<? extends Term> description = PredicateHelper.convertToVisionForm(PredicateHelper.createPredicate(searchString));
      //try to start type
      SearchManager searchType = availableSearchTypes.getInstance(null, description, true);
      if (searchType != null) {
        searchType.start(null);
      } else {
        log.warn("searchType is unavailable: " + description.toString());
      }
    }
  }

  /**
   * Reinitializes capture pipeline. This should only be used for testing
   * purposes.
   *
   * @param displayFlag
   * @param captureConfig
   * @param calibConfig
   * @param detectorsConfig
   * @param trackersConfig
   * @param imgProcConfig
   * @param saliencyProcConfig
   * @param validationProcConfig
   * @param searchTypeConfig
   */
  public static void reinit(final boolean displayFlag,
                            final String captureConfig,
                            final String calibConfig,
                            final String detectorsConfig, final String trackersConfig, final String imgProcConfig,
                            final String saliencyProcConfig, final String validationProcConfig, final String searchTypeConfig) {

    // =====================================================================
    //init camera (and start it after initializing everthing else)
    camera.stopAndWait();
    camera = new Camera(displayFlag,
            captureConfig, calibConfig);
    camera.start();

    // get image size
    Dimension imageSize = new Dimension();
    imageSize.height = camera.getImageHeight();
    imageSize.width = camera.getImageWidth();

    // =====================================================================
    //init all "type" objects - internally populated based on enums above
    availableImageProcessors = new AvailableImageProcessors(imageSize.width, imageSize.height, camera.hasStereo(), imgProcConfig);
    availableSaliencyProcessors = new AvailableImageProcessors(imageSize.width, imageSize.height, camera.hasStereo(), saliencyProcConfig);
    availableValidationProcessors = new AvailableImageProcessors(imageSize.width, imageSize.height, camera.hasStereo(), validationProcConfig);
    availableDetectors = new AvailableDetectors(imageSize.width, imageSize.height, detectorsConfig);
    availableTrackers = new AvailableTrackers(imageSize.width, imageSize.height, trackersConfig);
    availableSearchTypes = new AvailableSearchManagers(searchTypeConfig);
  }

  /**
   * Add clean up code here. This is mainly to ensure necessary c++ destructors are called on SWIG wrapped code.
   *
   * TODO: add stopAndWait methods to img proc and mem types ???
   */
  public static void shutdown() {
    log.debug("[Vision::shutdown] local shutdown: started");

    //log.debug("[Vision::shutdown] local shutdown: img procs");
//        for (ImageProcessor ipType : Vision.availableImageProcessors.getAll()) {
//            if (ipType.isRunning()) {
//                ipType.stop();
//            }
//        }
    log.debug("[Vision::shutdown] local shutdown: SearchManagers");
    for (SearchManager searchType : availableSearchTypes.getAll()) {
      if (searchType.isRunning()) {
        searchType.stop(null, true);
      }
    }

    log.debug("[Vision::shutdown] local shutdown: Camera");
    camera.stopAndWait();

    // close GUIs
    /*
    log.debug("[Vision::shutdown] local shutdown: GUIs");

    try {
    EventQueue.invokeAndWait(new Runnable() {
      @Override
      public void run() {
        FPSpanel.dispose();
        ccp.dispose();
      }
    });
    } catch (InterruptedException | InvocationTargetException e) {
      log.error("Exception trying to shut down GUIs.", e);
    }
    */

    log.debug("[Vision::shutdown] local shutdown: finished");
  }
}
