/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision;

import java.io.IOException;
import java.net.URL;
import java.util.*;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.util.PragUtil;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.vision.consultant.RealVisionConsultant;
import edu.tufts.hrilab.vision.consultant.VisionReference;
import edu.tufts.hrilab.vision.stm.*;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.reflection.TaskPerformanceInformation;
import edu.tufts.hrilab.vision.util.CompressionUtil;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import java.awt.Dimension;
import javax.vecmath.Matrix4d;

import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

/**
 * DIARC Vision component.
 */
public class VisionComponent extends DiarcComponent implements VisionInterface {

  private boolean runSandbox = false;
  private boolean printVisionStats = false;
  private boolean introspection = false;
  private boolean displayControlPanel = true;
  private List<List<String>> showWindows = new ArrayList();
  private List<String> runMemObjTypes = new ArrayList();

  // vision configuration filename and path info
  
  /**
   * Default path to vision resources (configs).
   */
  private String defaultConfigPath = "config/edu/tufts/hrilab/vision";
  private String captureConfig = "default.xml";
  private String calibConfig = null;
  private String detectorConfig = "detectors.xml";
  private String trackerConfig = "trackers.xml";
  private String imgProcConfig = "imgProcs.xml";
  private String saliencyProcConfig = "saliencyProcs.xml";
  private String validationProcConfig = "validationProcs.xml";
  private String searchConfig = "searchTypes.xml";
  private String learnersConfig = "learners.xml";
  private String log4cxxConfig = "log4cxx.txt";

  // file to load pre-defined references and their properties from file
  private String refsConfigFile;

  // camera coordinate frame info
  private Matrix4d currVisionTransform = new Matrix4d();    //transform from ground/base(x-right, y-forward, z-up) to vision coordinates (x-right, y-down, z-forward)
  private double rotX = -Math.PI / 4.0;    //rotation of vision coordinates around x-axis (using vision coordinate system)
  private double transHeight = 1.5;    //distance of vision sensor from ground (meters)
  private String cameraCoordinateFrame = "Camera";

  // flags for making TRADE service calls
  private boolean useCoordinateFrames = false;
  private boolean publishPointCloud = false;

  // registering with belief component for notifications
  private boolean registerForBeliefNotifications = false;

  // to keep from spamming the terminal for various logging messages
  private boolean shouldLogTransformMsg = true;
  private boolean shouldLogBeliefMsg = true;

  public VisionComponent() {
    super();
  }

  @Override
  protected void init() {
    // set path for config files if it hasn't been specified
    captureConfig = createFilepath(defaultConfigPath, "capture", captureConfig);
    calibConfig = createFilepath(defaultConfigPath, "calibration", calibConfig);
    detectorConfig = createFilepath(defaultConfigPath, "detectors", detectorConfig);
    trackerConfig = createFilepath(defaultConfigPath, "trackers", trackerConfig);
    imgProcConfig = createFilepath(defaultConfigPath, "imgProcs", imgProcConfig);
    saliencyProcConfig = createFilepath(defaultConfigPath, "imgProcs", saliencyProcConfig);
    validationProcConfig = createFilepath(defaultConfigPath, "imgProcs", validationProcConfig);
    searchConfig = createFilepath(defaultConfigPath, "searchTypes", searchConfig);
    learnersConfig = createFilepath(defaultConfigPath, "learners", learnersConfig);
    log4cxxConfig = createFilepath(defaultConfigPath, "logging", log4cxxConfig);

    log.info("Starting vision component with capture config: {}", captureConfig);

    if (printVisionStats) {
      TaskPerformanceInformation.setPrintStats(printVisionStats);
    }

    Vision.init(displayControlPanel,
            captureConfig,
            calibConfig,
            detectorConfig, trackerConfig, imgProcConfig,
            saliencyProcConfig, validationProcConfig, searchConfig,
            learnersConfig,
            log4cxxConfig,
            runMemObjTypes,
            showWindows,
            new RealVisionConsultant(this,"physobj"),this.getMyGroups());

    if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
      String filename = Resources.createFilepath(defaultConfigPath, refsConfigFile);
      Vision.consultant.loadReferencesFromFile(filename);
    }

    if (!useCoordinateFrames) {
      //set default
      currVisionTransform.rotX(-Math.PI / 2.0 + rotX);
      currVisionTransform.m23 = transHeight;
    }

    shouldRunExecutionLoop = true;
  }

  /**
   * Should only be used for running component tests to reinitialize capture
   * setting (e.g., use a different image, or camera mode).
   *
   * @param camRotX
   * @param camHeight
   */
  public void reinitializeVisionComponent(double camRotX, double camHeight) {

    rotX = camRotX;
    transHeight = camHeight;

    Vision.reinit(displayControlPanel, captureConfig, calibConfig,
            detectorConfig, trackerConfig, imgProcConfig,
            saliencyProcConfig, validationProcConfig, searchConfig);

    // reset camera coordinate frame
    currVisionTransform.rotX(-Math.PI / 2.0 + rotX);
    currVisionTransform.m23 = transHeight;
  }

  @Override
  protected void executionLoop() {
    // TODO: this should be replaces with learn action calls instead of belief notifications
    if (registerForBeliefNotifications) {
      try {
        TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForNotification"));
        if (!shouldLogBeliefMsg) {
          log.warn("registerForNotification service now available.");
        }
        Term instanceOfTerm = Factory.createPredicate("instanceOf(X,Y)");
        Term definitionOfTerm = Factory.createPredicate("definitionOf(X,Y)");
        try {

          TRADEServiceInfo learnService = getMyService("learn", Term.class, List.class);
          registerService.call(void.class,instanceOfTerm, learnService);
          registerService.call(void.class,definitionOfTerm, learnService);
          registerForBeliefNotifications = false; //don't want to keep on registering
        } catch (TRADEException e) {
          log.warn("Could not register for belief notifications. Will try again.", e);
          registerForBeliefNotifications = true;
        }
      } catch (TRADEException e){
        if (shouldLogBeliefMsg) {
          shouldLogBeliefMsg = false;
          log.warn("registerForNotification service is not yet available. Will only print once.",e);
        }
      }
    }

    //update vision pipeline
    threadMethod();

    // run sandbox (only once)
    if (runSandbox) {
      sandbox();
      runSandbox = false;
    }
  }

  @Override
  public void pauseCapture() {
    Vision.camera.pause();
  }

  @Override
  public void resumeCapture() {
    Vision.camera.resume();
  }

  @Override
  public void stopAllSearches() {
    Vision.availableSearchTypes.stopAllSearches(this);
  }

  @Override
  public void restartAllStoppedSearches() {
    Vision.availableSearchTypes.restartAllStoppedSearches(this);
  }

  /**
   * Helper method to add default path if only filename had been specified.
   *
   * @param defaultDir
   * @param subDir
   * @param filename
   * @return
   */
  private String createFilepath(String defaultDir, String subDir, String filename) {
    if (filename == null || filename.isEmpty()) {
      return filename;
    }
    URL url = Resources.getResource(defaultDir + "/" + subDir, filename);
    if (url == null) {
      log.warn("Filepath resource not found: {}", filename);
      return null;
    } else {
      return url.getPath();
    }
  }

  // method that is called during the main loop
  private void threadMethod() {
    log.trace("[threadMethod] entered.");

    //get any necessary outside info to pass into camera for detection, tracking, etc...
    //EAK: This is where the coordinate transform should be retrieved from the appropriate component,
    //so that vision can convert memory objects from its own coordinate system to the
    //appropriate robot coordinate system. This should probably be replaced/modified to
    //use the DIARC notification mechansim when it becomes available, so that the conversion
    //data is only updated when it changes.
    if (useCoordinateFrames) {
      log.trace("Trying to get camera coordinate frame...");

      TRADEServiceConstraints additionalConstraints = this.getMyGroupConstraints();
      try {
        // TODO: replace this with a notify call
        TRADEServiceInfo getTransformService = TRADE.getAvailableService(additionalConstraints.name("getTransform").argTypes(String.class));
        if (!shouldLogTransformMsg) {
          shouldLogTransformMsg = true;
          log.warn("getTransform service is now available.");
        }
        try {
          Matrix4d tmpTransform = getTransformService.call(Matrix4d.class, cameraCoordinateFrame);
          if (tmpTransform != null) {
            currVisionTransform = tmpTransform;
          } else {
            log.warn("getTransform returned null, not updating vision transform.");
          }
          log.trace(String.format("Got camera coordinate transform: %s.", currVisionTransform));
        } catch (TRADEException e) {
          log.warn("Could not call getTransform.", e);
        }
      } catch (TRADEException e) {
        if (shouldLogTransformMsg) {
          shouldLogTransformMsg = false;
          log.warn("getTransform service not yet available. Will only print once.",e);
        }
      }
    }

    if (publishPointCloud) {
      byte[] depthData = getDepthFrame();

        TRADE.getAvailableServices(new TRADEServiceConstraints().name("publishPointCloudToRos")).forEach( s ->{
          try {
            s.call(Boolean.class, depthData);
          } catch (TRADEException e) {
            log.warn("Could not publish point cloud.", e);
          }
                }
        );
    }

    Vision.camera.passAdditionalDataToCamera(currVisionTransform);

    //check introspection constraints on currently running types
    if (introspection) {
      log.error("Introspection is currently unavailable!");

//      log.trace("introspection is on");
//      List<SearchManager> searchTypes = Vision.availableSearchTypes.getAll();
//      for (SearchManager currSearchType : searchTypes) {
//        if (currSearchType.isRunning()) {   //if is currently running
//          //analyse introspection data
//          Collection<TaskAnalysisPolicy.Result> introspectionResults = currSearchType.meetsExpectations();
//
//          for (TaskAnalysisPolicy.Result result : introspectionResults) {
////                            if (result  != TaskAnalysisPolicy.Result.CONTINUE) {
////                                log.debug(String.format("Introspection results for type %s: %s.", currMoType.toString(), introspectionResults.toString()));
////                            }
//
//            //take action based on results
//            if (result == TaskAnalysisPolicy.Result.CONTINUE) {
//              break;
//            } else if (result == TaskAnalysisPolicy.Result.QUIT) {
//              //turn off moType detection and tracking
//              currSearchType.suspend();
//            } else if (result == TaskAnalysisPolicy.Result.FIND_ALT) {
//              log.trace(String.format("Vision FIND_ALT to: %s", currSearchType.getSelectedDetector().getType().toString()));
//
//              //stop current detector/tracker, to prepare for switch
//
//              log.trace("Vision FIND_ALT turn off.");
//              currSearchType.suspend();
//
//              //attempt to find alternatives, otherwise keep using current detector and tracker
//              while (currSearchType.isRunning()) {
//                try {
//                  Thread.sleep(5);
//                } catch (InterruptedException e) {
//                }
//              }
//              log.trace("Vision FIND_ALT switch.");
//              currSearchType.findAlternativeDetectorAndTracker();
//
//              //restart with newdetector/tracker
//              log.trace("Vision FIND_ALT turn back on.");
//              currSearchType.start();;
//            } else if (result == TaskAnalysisPolicy.Result.PAUSE_MOTION) {
//              log.trace("Vision PAUSE_MOTION.");
//              if (useGoalManager) {
//                // not sure how long to wait
//                long pauseMillis = 5000;
//                // create a goal predicate
//                Predicate goal = new Predicate("paused", pauseMillis + "");
//                // submit the goal
//                gmComponent.call("submitGoal", void.class, goal);
//              }
//            }
//          }
//        }
//      }
    }

  }

  @Override
  public void shutdownComponent() {
    Vision.shutdown();
  }

  // provide  information for usage...
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("hideControls").longOpt("hideControlPanel").desc("hide all GUIs (otherwise on by default)").build());
    options.add(Option.builder("capture").longOpt("captureConfig").hasArg().argName("file").desc("camera capture configuration file").build());
    options.add(Option.builder("calibConfig").longOpt("calibConfig").hasArg().argName("file").desc("stereo calibration parameters (must be running stereo cameras)").build());
    options.add(Option.builder("dc").longOpt("detectConfig").hasArg().argName("file").desc("set available Detectors and their config files").build());
    options.add(Option.builder("tc").longOpt("trackConfig").hasArg().argName("file").desc("set available Trackers and their config files").build());
    options.add(Option.builder("ipc").longOpt("imgProcConfig").hasArg().argName("file").desc("set available ImageProcessors and their config files").build());
    options.add(Option.builder("spc").longOpt("saliencyConfig").hasArg().argName("file").desc("set available SaliencyProcessors and their config files").build());
    options.add(Option.builder("vpc").longOpt("validationConfig").hasArg().argName("file").desc("set available ValidationProcessors and their config files").build());
    options.add(Option.builder("sc").longOpt("searchConfig").hasArg().argName("file").desc("set available SearchManagers configuration").build());
    options.add(Option.builder("lc").longOpt("learnersConfig").hasArg().argName("file").desc("set available Learners configuration").build());
    options.add(Option.builder("logc").longOpt("logcxxConfig").hasArg().argName("file").desc("set log4cxx configuration file. uses vision/native/data/logging/log4cxx.txt by default").build());
    options.add(Option.builder("rotX").hasArg().argName("x").desc("degrees to rotate vision coordinates around x-axis (neg to \"look down\", default: -45)").build());
    options.add(Option.builder("height").hasArg().argName("x").desc("distance of vision coordinates above ground (meters)").build());
    options.add(Option.builder("cameraFrame").hasArg().argName("frame").desc("set name of camera coordinate frame and use CoordinateFramesComponent for coord transform").build());
    options.add(Option.builder("publishPointCloud").hasArgs().argName("groups").optionalArg(true).desc("send point cloud data to another componen to be published to ROS").build());
    options.add(Option.builder("belief").hasArgs().argName("groups").optionalArg(true).desc("egister for notifications from com.belief.BeliefComponent").build());
    options.add(Option.builder("i").longOpt("introspection").desc("turn on vision introspection").build());
    options.add(Option.builder("sandbox").desc("run testing sandbox method").build());
    options.add(Option.builder("show").hasArgs().argName("types").desc("display window of memory object type").build());
    options.add(Option.builder("runType").hasArgs().argName("types").desc("turn on detection/tracking of specified memory object type(s)").build());
    options.add(Option.builder("printStats").desc("automatically print VisionProcess stats on task completion to visionStats_<processname>.txt").build());
    options.add(Option.builder("refs").longOpt("references").hasArg().argName("file").desc("load pre-defined object references and their properties").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("cameraFrame")) {
      useCoordinateFrames = true;
      cameraCoordinateFrame = cmdLine.getOptionValue("cameraFrame");
    }
    if (cmdLine.hasOption("publishPointCloud")) {
      publishPointCloud = true;
    }
    if (cmdLine.hasOption("belief")) {
      registerForBeliefNotifications = true;
    }
    if (cmdLine.hasOption("rotX")) {
      double rotDeg = Double.parseDouble(cmdLine.getOptionValue("rotX"));
      if (rotDeg > 0) {
        log.warn("Setting rotX to positive value which is \"looking up\". Did you mean to set a negative value?");
      }
      rotX = Math.toRadians(rotDeg);
    }
    if (cmdLine.hasOption("height")) {
      transHeight = Double.parseDouble(cmdLine.getOptionValue("height"));
    }
    if (cmdLine.hasOption("show")) {
      showWindows.add(Arrays.asList(cmdLine.getOptionValues("show")));
    }
    if (cmdLine.hasOption("runType")) {
      runMemObjTypes.addAll(Arrays.asList(cmdLine.getOptionValues("runType")));
      log.info("Added runType: " + runMemObjTypes);
    }
    if (cmdLine.hasOption("hideControls")) {
      displayControlPanel = false;
    }
    if (cmdLine.hasOption("i")) {
      introspection = true;
    }
    if (cmdLine.hasOption("capture")) {
      captureConfig = cmdLine.getOptionValue("capture");
    }
    if (cmdLine.hasOption("calib")) {
      calibConfig = cmdLine.getOptionValue("calib");
    }
    if (cmdLine.hasOption("dc")) {
      detectorConfig = cmdLine.getOptionValue("dc");
    }
    if (cmdLine.hasOption("tc")) {
      trackerConfig = cmdLine.getOptionValue("tc");
    }
    if (cmdLine.hasOption("ipc")) {
      imgProcConfig = cmdLine.getOptionValue("ipc");
    }
    if (cmdLine.hasOption("spc")) {
      saliencyProcConfig = cmdLine.getOptionValue("spc");
    }
    if (cmdLine.hasOption("vpc")) {
      validationProcConfig = cmdLine.getOptionValue("vpc");
    }
    if (cmdLine.hasOption("sc")) {
      searchConfig = cmdLine.getOptionValue("sc");
    }
    if (cmdLine.hasOption("lc")) {
      learnersConfig = cmdLine.getOptionValue("lc");
    }
    if (cmdLine.hasOption("logc")) {
      log4cxxConfig = cmdLine.getOptionValue("logc");
    }
    if (cmdLine.hasOption("printStats")) {
      printVisionStats = true;
    }
    if (cmdLine.hasOption("sandbox")) {
      runSandbox = true;
    }
    if (cmdLine.hasOption("refs")) {
      refsConfigFile = cmdLine.getOptionValue("refs");
    }
  }

  /* Publicly Available Methods, Advertised in VisionComponent */
  /* Short Term Memory Methods */
  @Override
  public void startType(final long typeId) {
    Vision.availableSearchTypes.getInstance(this, typeId).start(this);
  }

  @Override
  public void stopType(final long typeId) {
    Vision.availableSearchTypes.getInstance(this, typeId).stop(this, false);
  }

  @Override
  public void stopAndRemoveType(final long typeId) {
    //final long startTime = System.currentTimeMillis();
    //log.trace(String.format("stopAndRemoveType id: %d", typeId));

//		Thread t = new Thread(new Runnable() {
//			public void run() {
//				Vision.availableSearchTypes.removeType(typeId);
//				log.trace(String.format("stopAndRemoveType returning. id: %d. time: ", typeId, System.currentTimeMillis() - startTime));
//			}
//		});
//		t.start();
    //EAK: TODO: temporarily taking out
    SearchManager search = Vision.availableSearchTypes.getInstance(this, typeId);
    search.stop(this, true);
    Vision.availableSearchTypes.release(this, search);
  }

  @Override
  public List<Long> getAvailableTypeIds() {
    return Vision.availableSearchTypes.getTypeIds();
  }

  @Override
  public List<Long> getTypeIds(final double conf) {
    return ShortTermMemoryInterface.getTypeIds(conf);
  }

  @Override
  public Long getTypeId(final List<? extends Symbol> descriptors) {
    log.debug(String.format("[getTypeId] called with: %s.", descriptors.toString()));

    Queue<Symbol> toFilter = new ArrayDeque<>(descriptors);
    Set<Symbol> alreadyFiltered = new HashSet<>();
    List<Symbol> filteredDescriptors = new ArrayList<>();

    // find any POWER object references, instantiate search manager for each ref (if it hasn't been instantiated already)
    // and replace each ref with its descriptors
    //
    // TODO: ensure consistency across variables from object refs and object ref descriptors
    // e.g., "on(object_0, objects_1)" might convert to "on(X,X), cup(X), table(X)" which is nonsensical and needs
    // to be converted to something like "on(X,Y), cup(X), table(Y)"
    while (!toFilter.isEmpty()) {
      Symbol d = toFilter.poll();
      alreadyFiltered.add(d);
      if (d.isTerm()) {
        Term d_pred = (Term) d;
        Term filteredPred = d_pred.clone();
        for (Symbol s : d_pred.getOrderedLeaves()) {
          if (s.getName().startsWith(Vision.consultant.getKBName())) {
            // found POWER object ref
            VisionReference reference = Vision.consultant.getReference(s);

            // add all of ref's properties to toFilter (ignoring properties that have already been filtered to prevent infinite loop)
            reference.properties.stream()
                    .filter(java.util.function.Predicate.not(alreadyFiltered::contains))
                    .forEach(toFilter::add);

            filteredPred = PredicateHelper.replace(filteredPred, s, reference.variable);
          } else if (!s.isVariable() && Vision.availableSearchTypes.hasExistingSearchManager(PredicateHelper.convertToVisionForm(s))) {
            // TODO: remove this when agents are resolved by POWER
            // temporary work around to make sure non-power descriptors maintain variable consistency
            // with existing searches. For example, if there's already a search for bill(B), and this search
            // is for touching(bill, objects_0), this search should use B for its free variable for bill.
            SearchManager searchManager = Vision.availableSearchTypes.getInstance(this, PredicateHelper.convertToVisionForm(s), true);
            Variable var = (Variable) searchManager.getDescriptors().get(0).get(0);
            filteredPred = PredicateHelper.replace(filteredPred, s, new Term(s.getName(), var));
          }
        }

        // add descriptor after all args have been filtered for POWER refs
        filteredDescriptors.add(filteredPred);
      } else {
        // nothing to filter -- just pass through descriptor as is
        filteredDescriptors.add(d);
      }

    }

    // convert descriptors to vision component form
    List<Term> visionDescriptors = PredicateHelper.convertToVisionForm(filteredDescriptors);

    Long typeId = -1L;
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, visionDescriptors, true);

    if (searchType != null) {
      log.info("[getTypeId] searchTypeId: " + searchType.getTypeId() + " hasIterationCompleted: " + searchType.hasIterationCompleted());
      typeId = searchType.getTypeId();
      if (!searchType.isRunning()) {
        searchType.start(this);
      }
    }

    log.debug(String.format("[getTypeId] returning with typeId: %d.", typeId));
    return typeId;
  }

  @Override
  public Long getTypeId(final Symbol objectRef) {
    log.debug("[getTypeId] objectRef: " + objectRef);
    return Vision.consultant.getTypeId(objectRef);
  }

  @Override
  public boolean nameDescriptors(final List<? extends Symbol> descriptors, final Symbol typeName) {
    log.trace(String.format("[nameDescriptors] called with: %s.", descriptors.toString()));

    // convert descriptors to vision component form
    List<Term> visionDescriptors = PredicateHelper.convertToVisionForm(descriptors);
    List<Term> visionTypeName = PredicateHelper.convertToVisionForm(typeName);
    if (visionTypeName.size() != 1) {
      log.error("[nameDescriptors] cannot name descriptors with more than one type name: " + visionTypeName);
      return false;
    }

    if (Vision.availableSearchTypes.canCreateCapableSearchManager(visionDescriptors)) {
      Vision.availableSearchTypes.nameDescriptors(visionDescriptors, visionTypeName.get(0));
      return true;
    }
    return false;
  }

  @Override
  public boolean nameDescriptors(final Long typeId, final Symbol typeName) {
    log.trace(String.format("[nameDescriptors] called with typeId: %d.", typeId));

    List<Term> visionTypeName = PredicateHelper.convertToVisionForm(typeName);
    if (visionTypeName.size() != 1) {
      log.error("[nameDescriptors] cannot name descriptors with more than one type name: " + visionTypeName);
      return false;
    }

    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);
    if (null != searchType) {
      Vision.availableSearchTypes.addTypeName(searchType.getTypeId(), visionTypeName.get(0));
      return true;
    }
    return false;
  }

  @Override
  public List<Term> getDescriptors(final Long typeId) {
    List<Term> descriptors = new ArrayList();
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);
    if (null != searchType) {
      descriptors = searchType.getDescriptors();
    }
    return descriptors;
  }

  @Override
  public List<Long> getTokenIds(final double conf) {
    log.trace("[getTokenIds] called.");
    return ShortTermMemoryInterface.getTokenIds(conf);
  }

  @Override
  public List<Long> getTokenIds(final long typeId, final double conf) {
    log.debug(String.format("[getTokenIds] (by typeId) called with typeId: %d. conf: %f.", typeId, conf));
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);

    if (searchType != null) {
      log.info("[getTokenIds] typeId: " + searchType.getTypeId() + " hasIterationCompleted: " + searchType.hasIterationCompleted());

      boolean isRunning = searchType.isRunning();

      //if not already running, turn on type detection/tracking
      if (!isRunning) {
        searchType.start(this);
      }

      //wait till at least one detection and one tracking iteration has finished
      //TODO: change this to use wait/notify instead of sleep
      log.debug(String.format("getTokenIds (by typeId). waiting for results. isRunning: %b. id: %d", isRunning, typeId));
      while (!searchType.hasIterationCompleted()) {
        isRunning = searchType.isRunning();
        log.trace(String.format("getTokenIds (by typeId). waiting for results. isRunning: %b. id: %d", isRunning, typeId));
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
        }

      }
      log.debug(String.format("getTokenIds (by typeId). Done waiting for results! isRunning: %b. id: %d", isRunning, typeId));

      //finally, get results
      log.trace(String.format("getTokenIds (by typeId). fetching results: %d", typeId));
      return ShortTermMemoryInterface.getTokenIds(typeId, conf);
    } else {
      log.warn(String.format("Invalid type %d passed to getTokenIds.", searchType));
      return new ArrayList<>();
    }
  }

  @Override
  public List<Long> getTokenIds(final List<? extends Symbol> descriptors, final double conf) {
    log.debug(String.format("[getTokenIds] (by description) called with: %s.", descriptors.toString()));
    long typeId = getTypeId(descriptors);
    return getTokenIds(typeId, conf);
  }

  @Override
  public List<MemoryObject> getTokens(final double conf) {
    log.trace("[getTokens] called.");
    return ShortTermMemoryInterface.getTokens(conf);
  }

  @Override
  public List<MemoryObject> getTokens(final long typeId, final double conf) {
    log.trace(String.format("[getTokens] (by typeId) called: %d.", typeId));

    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);
    if (searchType != null) {
      //if not already running, turn on type detection/tracking
      if (!searchType.isRunning()) {
        searchType.start(this);
      }

      //wait till at least one tracking iteration has finished
      //TODO: change this to use wait/notify instead of sleep
      boolean isRunning = searchType.isRunning();
      log.debug(String.format("getTokens (by typeId). waiting for results. isRunning: %b. id: %d", isRunning, typeId));
      while (!searchType.hasIterationCompleted()) {
        isRunning = searchType.isRunning();
        log.trace(String.format("getTokens (by typeId). waiting for results. isRunning: %b. id: %d", isRunning, typeId));
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
        }
      }
      log.debug(String.format("getTokens (by typeId). Done waiting for results! isRunning: %b. id: %d", isRunning, typeId));

      //finally, get results
      log.debug("getTokensByTypeId fetching results: " + typeId);
      return ShortTermMemoryInterface.getTokens(typeId, conf);
    } else {
      log.debug(String.format("Invalid type %d passed to getTokens (by typeId).", searchType));
      return new ArrayList<>();
    }
  }

  @Override
  public List<MemoryObject> getTokens(final List<? extends Symbol> descriptors, final double conf) {
    log.trace(String.format("getTokens (by descriptors): %s", descriptors));
    long typeId = getTypeId(descriptors);
    return getTokens(typeId, conf);
  }

  @Override
  public MemoryObject getToken(final long tokenId, final double conf) {
    log.trace(String.format("getToken: %d", tokenId));
    MemoryObject mo = ShortTermMemoryInterface.getToken(tokenId, conf);
    log.trace(String.format("got Token: %d", tokenId));
    return mo;
  }

  // ============== START Incremental Search Methods ====================================
  @Override
  public Long createNewType() {
    log.trace("createNewType");
    Long typeId = -1L;
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this);
    if (searchType != null) {
      typeId = searchType.getTypeId();
    }

    return typeId;
  }

  @Override
  public boolean addDescriptor(final long typeId, final Symbol descriptor) {
    log.trace(String.format("addDescriptor id: %d. predicate: %s", typeId, descriptor));
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);
    if (searchType == null) {
      log.warn(String.format("addDescriptor SearchManager ID %d does not name a valid SearchManager. Not adding constraint.", typeId));
      return false;
    }

    // convert descriptors to vision component form
    List<Term> visionDescriptors = PredicateHelper.convertToVisionForm(descriptor);

    //add constraint(s)
    boolean allDescriptorsAdded = true;
    for (Term visionDescriptor : visionDescriptors) {
      if (!searchType.addConstraint(visionDescriptor)) {
        allDescriptorsAdded = false;
      }
    }

    //request that it starts automatically (if not already running)
    if (allDescriptorsAdded) {
      searchType.start(this);
    }

    return allDescriptorsAdded;
  }

  @Override
  public boolean removeDescriptor(final long typeId, final Symbol descriptor) {
    log.trace(String.format("removeDescriptor id: %d predicate: %s", typeId, descriptor));
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);
    if (searchType == null) {
      log.warn(String.format("removeDescriptor. SearchManager ID %d does not name a valid SearchManager. Not adding constraint.", typeId));
      return false;
    }

    // convert descriptors to vision component form
    List<Term> visionDescriptors = PredicateHelper.convertToVisionForm(descriptor);

    //remove constraint(s)
    boolean allDescriptorsRemoved = true;
    for (Term visionDescriptor : visionDescriptors) {
      if (!searchType.removeConstraint(visionDescriptor)) {
        allDescriptorsRemoved = false;
      }
    }
    return allDescriptorsRemoved;
  }

  //TODO: remove
  @Override
  public void endDescriptorChanges(final long typeId) {
    log.trace(String.format("endDescriptorChanges id: %d", typeId));
    SearchManager searchType = Vision.availableSearchTypes.getInstance(this, typeId);
    if (searchType == null) {
      log.warn(String.format("endDescriptorChanges SearchManager ID %d does not name a valid SearchManager.", typeId));
      return;
    }

    //TODO: add explicit flag disallowing additional constraints once this has been called!!
    //or maybe just don't allow added constraints while the Detector and Tracker are running?
    //start Detector and Tracker
    //a default Detector and Tracker will be assigned and used if particulars
    //ones have not been assigned through a addContraintToVisualSearch call.
    searchType.start(this);
  }
  // ============== END Incremental Search Methods ====================================

  @Override
  public boolean confirmToken(final long tokenId) {
    log.trace("ConfirmObject by key: " + tokenId);
    return ShortTermMemoryInterface.confirmObject(tokenId);
  }

  @Override
  public boolean confirmToken(final MemoryObject token) {
    log.trace("ConfirmObject by MO: " + token);
    return ShortTermMemoryInterface.confirmObject(token);
  }

  @Override
  public void takeSnapshot(final String filename) {
    if (filename != null && filename.endsWith(".jpg")) {
      Vision.camera.writeFrame(filename);
    } else {
      log.info("No filename, or invalid filename passed. Saving as \"snapshot.jpg\".");
      Vision.camera.writeFrame("snapshot.jpg");
    }

  }

  @Override
  public byte[] getFrame() {
    byte[] imgRaw = Vision.camera.getRawImageData();
    byte[] imgComp = null;
    try {
      imgComp = CompressionUtil.compress(imgRaw);
    } catch (IOException e) {
      log.error("Error compressing rgb image.", e);
    }
    return imgComp;
  }

  @Override
  public byte[] getDisparityFrame() {
    byte[] imgRaw = Vision.camera.getRawDisparityData();
    byte[] imgComp = null;
    try {
      imgComp = CompressionUtil.compress(imgRaw);
    } catch (IOException e) {
      log.error("Error compressing disparity frame.", e);
    }
    return imgComp;
  }

  @Override
  public byte[] getDepthFrame() {
    byte[] imgRaw = Vision.camera.getRawDepthData();
    byte[] imgComp = null;
    try {
      imgComp = CompressionUtil.compress(imgRaw);
    } catch (IOException e) {
      log.error("Error compressing depth frame.", e);
    }
    return imgComp;
  }

  @Override
  public Dimension getImageSize() {
    return new Dimension(Vision.camera.getImageWidth(), Vision.camera.getImageHeight());
  }

  @Override
  public List<Symbol> getUnsatisfiableConstraints(List<? extends Symbol> descriptors) {
    List<Term> visionDescriptors = PredicateHelper.convertToVisionForm(descriptors);
    List<Term> unsatisfiableConstraints = Vision.availableSearchTypes.getUnsatisfiableConstraints(visionDescriptors);
    log.debug("[getUnsatisfiableConstraints] descriptors: " + descriptors + " unsatisfiableConstraints: " + unsatisfiableConstraints);
    // convert from "vision form" to "parser form"
    // e.g., vision form: "cup(X)" parser form: "cup"
    List<Symbol> unsatisfiableSymbols = new ArrayList<>();
    for (Term uc : unsatisfiableConstraints) {
      unsatisfiableSymbols.add(new Symbol(uc.getName()));
    }
    return unsatisfiableSymbols;
  }

  @Override
  public List<Symbol> getUnsatisfiableConstraints(Symbol descriptor) {
    List<Symbol> descriptors = new ArrayList<>();
    descriptors.add(descriptor);
    return getUnsatisfiableConstraints(descriptors);
  }

  @Override
  public boolean learn(Term learningTerm, List<Map<Variable, Symbol>> bindings) {
    log.debug("[learn] learningTerm: " + learningTerm);

    if (learningTerm.size() != 2) {
      log.error("[learn] expects a Term with two arguments: " + learningTerm);
      return false;
    }

    boolean somethingLearned = false;

    // iterate through sets of bindings
    for (Map<Variable, Symbol> binding : bindings) {
      // get the bindings from the bindings list
      Term boundLearningTerm = PragUtil.getBoundTerm(binding, learningTerm);
      Term nameLearned = Vision.availableLearners.learn(boundLearningTerm);
      if (nameLearned != null) {
        somethingLearned = true;
      }
    }

    log.debug("[learn] somethingLearned: " + somethingLearned);
    log.debug("[learn] Object Ref Summary:\n" + Vision.consultant.getReferenceSummaries());
    return somethingLearned;
  }

  @Override
  public boolean unlearn(Term learnTerm, List<Map<Variable, Symbol>> bindings) {
    log.debug("[unlearn] called. learnTerm: " + learnTerm + " bindings: " + bindings);

    if (learnTerm.size() != 2) {
      log.error("[unlearn] expects a Term with two arguments: " + learnTerm);
      return false;
    }

    boolean somethingUnlearned = false;

    // iterate through sets of bindings
    Term nameToUnlearn;
    List<Term> descriptorsToUnlearn;
    for (Map<Variable, Symbol> binding : bindings) {

      // separate the name of the concept being learned and the definition of that concept
      Term boundLearningTerm = PragUtil.getBoundTerm(binding, learnTerm);

      Term nameUnlearned = Vision.availableLearners.unlearn(boundLearningTerm);

      if (nameUnlearned != null) {
        somethingUnlearned = true;
      }
    }

    log.debug("[unlearn] Object Ref Summary:\n" + Vision.consultant.getReferenceSummaries());
    return somethingUnlearned;
  }

  @Override
  public List<Map<Variable, Symbol>> makeObservations(Term observation) {
    log.debug("[makeObservations] called with observation: " + observation);

    // free variable check
    if (!observation.getVars().isEmpty()) {
      log.error("[makeObservations] cannot handle free variables: " + observation);
    }

    // perform vision query
    List<Term> observationList = new ArrayList<>();
    observationList.add(observation);
    Long typeId = getTypeId(observationList);
    if (typeId == -1L) {
      log.warn("[makeObservations] can not observe: " + observation);
      return null;
    }

    List<Long> tokenIds = getTokenIds(typeId, 0.0);

    long timeout = 5000L; // ms
    long startTime = System.currentTimeMillis();
    while (tokenIds.isEmpty() && (System.currentTimeMillis() - startTime) < timeout) {
      log.info("[makeObservations] still looking for: " + observation);
      Util.Sleep(500);
      tokenIds = getTokenIds(typeId, 0.0);
    }
    log.info("[makeObservations] typeId: " + typeId + " tokenIds: " + tokenIds);

    // check results construct return results
    List<Map<Variable, Symbol>> results = new ArrayList<>();
    if (!tokenIds.isEmpty()) {
      log.info("[makeObservations] FOUND: " + observation);
      results.add(new HashMap<>());
    } else {
      log.info("[makeObservations] did NOT find: " + observation);
    }

    stopType(typeId);

    return results;
  }

  /**
   * Write vision references to file.
   * @param filename
   */
  @TRADEService
  public void writeReferencesToFile(String filename) {
    Vision.consultant.writeReferencesToFile(filename);
  }

  /**
   * Load pre-defined vision references from file.
   * @param filename
   */
  @TRADEService
  public void loadReferencesFromFile(String filename) {
    Vision.consultant.loadReferencesFromFile(Resources.createFilepath(defaultConfigPath, filename));
  }

  public void sandbox() {
    log.info("[sandbox] trying to build a castle...");

    TRADEServiceInfo learnService = getMyService("learn", Term.class, List.class);

//    for (MemoryObject mo : getTokens(0.0)) {
//      Set<Term> sceneDescriptors = MemoryObjectUtil.getBoundSceneGraphDescriptors(mo);
//      log.info("sceneDescriptors for object " + mo.getIdentifier() + ". " + Arrays.toString(sceneDescriptors.toArray()));
//    }

    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////
    // if you build a transform T from A-frame to B-frame, transforming a location
    // in B-frame by T gives the location in A-frame.
//    Point3d x = new Point3d(1, 0, 0);
//    Point3d y = new Point3d(0, 1, 0);
//    Point3d z = new Point3d(0, 0, 1);
//
//    Matrix4d testtransform = new Matrix4d();
//    Matrix4d xrot = new Matrix4d();
//    xrot.setIdentity();
//    xrot.rotX(-Math.PI / 2.0);
//    Matrix4d zrot = new Matrix4d();
//    zrot.setIdentity();
//    zrot.rotZ(-Math.PI / 2.0);
//    testtransform.mul(zrot, xrot);
//    log.debug("test transform: " + testtransform);
//
//    Matrix4d cameratransform = new Matrix4d();
////    cameratransform.m02 = 1;
////    cameratransform.m10 = -1;
////    cameratransform.m21 = -1;
////    cameratransform.m33 = 1;
//
//    cameratransform.setIdentity();
//
//    cameratransform.rotX(Math.PI / 2.0);
//    cameratransform.rotY(Math.PI / 2.0);
//
//    //cameratransform.rotZ(Math.PI / 2.0);
//    //cameratransform.rotX(Math.PI / 2.0);
//    //cameratransform.rotY(Math.PI / 2.0);
//    //cameratransform.rotX(-Math.PI / 2.0);
//    //cameratransform.rotZ(-Math.PI / 2.0);
//    //cameratransform.rotX(-Math.PI / 2.0);
//    log.debug("camera transform: " + cameratransform);
//
//    Matrix4d transform = new Matrix4d(currVisionTransform);
////    transform.m03 = 0.0;
////    transform.m13 = 0.0;
////    transform.m23 = 0.0;
//    log.debug("nao transform: " + transform);
//    transform.mul(cameratransform);
//    log.debug("nao camera transform: " + transform);
//
//    log.debug("pre-x: " + x);
//    transform.transform(x);
//    log.debug("post-x: " + x);
//    log.debug("pre-y: " + y);
//    transform.transform(y);
//    log.debug("post-y: " + y);
//    log.debug("pre-z: " + z);
//    transform.transform(z);
//    log.debug("post-z: " + z);
//    List<MemoryObject> mos = getTokens(0.5d);
//    MemoryObject mo = mos.get(0);
//    mo.transformToBase();
    log.info("[sandbox] castle has been built!");
  }

  @TRADEService
  @Observes({"objectAt(?objRef, ?locRef)"})
  public List<Map<Variable, Symbol>> checkObjAt(Predicate predicate) {
    ArrayList<Map<Variable,Symbol>> returnVal = new ArrayList<>();
    Symbol objRef = predicate.getArgs().get(0);
    List<Long> objs = Vision.consultant.getTokenIds(objRef);
    if (!objs.isEmpty()) {
      log.info("object " + objRef + " is at " + predicate.getArgs().get(1));
      returnVal.add(new HashMap<>());
    } else {
      log.info("object " + objRef + " is NOT at " + predicate.getArgs().get(1));
    }
    return returnVal;
  }
}
