/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.reflection.TaskAnalysisPolicy;
import edu.tufts.hrilab.vision.reflection.TaskAnalysisPolicyDefault;
import edu.tufts.hrilab.vision.reflection.TaskPerformanceInformation;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @author Evan Krause
 */
public abstract class SearchManager {

  protected final long typeId;
  protected Term name = null;
  protected List<Term> description = new ArrayList();
  protected TaskPerformanceInformation performanceInfo = null;
  protected TaskAnalysisPolicy analysisPolicy = new TaskAnalysisPolicyDefault();
  private final HashSet<Object> startCallers = new HashSet();    //keeps track of clients that have called start
  //set to false when terminate() is called
  protected boolean validFlag = true;
  //default detector if one isn't specified
  protected Term defaultDetectorPredicate;
  //flags to control vision searches
  protected boolean incrementalImgProc = false;
  protected boolean incrementalDetector = false;
  protected boolean serialProcessing = false;
  protected boolean singleIteration = false; //for both detectors and imgprocs
  protected Logger log = LoggerFactory.getLogger(this.getClass());
  
  
  
  /**
   * Constructor is package-private so that the AvailableSearchMangers factory
   * is the only way a SearchManager can be instantiated. Use this constructor
   * when this SearchManager is going to be part of a hierarchical search (e.g.,
   * as a sub-type in a CompositeSearchManager). In this case, the typeId
   * should be that of the parent SearchManager.
   *
   * @param typeId top-level SearchManager typeId
   * @param incrementalImgProcFlag
   * @param incrementalDetectorFlag
   * @param serialProcessingFlag
   * @param singleIterationFlag
   */
  SearchManager(long typeId, boolean incrementalImgProcFlag, boolean incrementalDetectorFlag, boolean serialProcessingFlag, boolean singleIterationFlag) {
    this.typeId = typeId;
    
    performanceInfo = new TaskPerformanceInformation(String.format("SearchManager: %d.", typeId));

    incrementalImgProc = incrementalImgProcFlag;
    incrementalDetector = incrementalDetectorFlag;
    serialProcessing = serialProcessingFlag;
    singleIteration = singleIterationFlag;

    if (Vision.camera.hasDepth()) {
      defaultDetectorPredicate = Factory.createPredicate("object(X)");
    } else {
      defaultDetectorPredicate = Factory.createPredicate("object2d(X)");
    }
  }

  /**
   * Constructor is package-private so that the AvailableSearchMangers factory
   * is the only way a SearchManager can be instantiated. Creates a new unique
   * typeId.
   *
   * @param incrementalImgProcFlag
   * @param incrementalDetectorFlag
   * @param serialProcessingFlag
   * @param singleIterationFlag
   */
  SearchManager(boolean incrementalImgProcFlag, boolean incrementalDetectorFlag, boolean serialProcessingFlag, boolean singleIterationFlag) {
    this(Vision.typeIdGenerator.getNext(), incrementalImgProcFlag, incrementalDetectorFlag, serialProcessingFlag, singleIterationFlag);
  }

  /**
   * Not synchronized because the GUI frequently calls this method for repainting
   * so calling invokeAndWait (which needs to happen occasionally) will cause a
   * deadlock.
   * @return 
   */
  @Override
  final public String toString() {
    return String.valueOf(typeId) + " " + description.toString();
  }

  /**
   * Set name of SearchManager. a SearchManager is not required to have a name.
   * @param typeName 
   */
  final public synchronized void setName(final Term typeName) {
    isValid();

    name = typeName;
  }

  /**
   * Get name of SearchManager. This is null if not explicitly set by the
   * setName method.
   * @return 
   */
  final public synchronized Term getName() {
    isValid();

    return name;
  }

  /**
   * Get predicates of search type. (e.g., color(X,white), type(X,box),
   * color(Y,red), type(Y,cross), on(Y,X))
   *
   * @return
   */
  final public synchronized List<Term> getDescriptors() {
    isValid();

    //shallow copy
    List<Term> results = new ArrayList(description);
    return results;
  }

  /**
   * Get unique SearchManager ID (search typeId).
   *
   * @return
   */
  final public synchronized long getTypeId() {
    isValid();

    return typeId;
  }

  /**
   * Get a list of SearchManager IDs
   * This is especially important for searches that
   * contain sub-searches like relation
   * @return
   */
  public List<Long> getRelatedTypeIds() {
    isValid();
    List<Long> ids = new ArrayList<>();
    ids.add(typeId);
    return ids;
  }

  /**
   * Returns true if predicates exactly* match SearchManager's description.
   *
   * (*) Currently adds a default detector if a detector is not specified in the
   * descriptors, and removes any descriptors that don't map to any detectors or
   * image processors.
   *
   * @param descriptors
   * @return
   */
  final public synchronized boolean matchesDescriptors(final List<? extends Term> descriptors) {
    isValid();
    //System.out.println("matchesDescriptors this: " + description + " other: " + descriptors);

    //remove the default detector from the comparison
    //and don't modify object's description
    List<Term> this_description = new ArrayList(description);
    if (this_description.contains(defaultDetectorPredicate)) {
      this_description.remove(defaultDetectorPredicate);
    }

    //remove descriptors that don't map to any detectors or validation processors
    //and don't modify passed in descriptors
    List<Term> temp_descriptors = new ArrayList();
    for (Term d : descriptors) {
      if (Vision.availableDetectors.hasCapableDetector(d) || Vision.availableValidationProcessors.hasCapableProcessorType(d)) {
        temp_descriptors.add(d);
      }
    }

    boolean matchesDescription = Utilities.predicatesMatch(this_description, temp_descriptors);

    //if descriptions don't match, check if the name matches
    if (!matchesDescription && name != null
            && descriptors.size() == 1 && Utilities.predicatesMatch(name, descriptors.get(0))) {

      matchesDescription = true;
    }

    return matchesDescription;
  }
  
  /**
   * Add a sub-SearchManager to this SearchManager. This is only valid for
   * SearchManagers that are composed of other SearchManagers.
   * 
   * @param variables
   * @param searchManager 
   */
  public void addSearchManager(final List<Variable> variables, final SearchManager searchManager) {
    throw new UnsupportedOperationException("Can not add SearchManagers to SearchManagers of this type.");
  }

  /**
   * Add ImageProcessor or Detector constraint. Uses the flags set at runtime:
   * startFlag, incrementalFlag, and singleIteration.
   *
   * @param term
   * @return bool - if predicate mapped to an ImageProcessor or Detector
   */
  final public synchronized boolean addConstraint(final Term term) {
    isValid();

    if (startCallers.size() > 1) {
      log.error("[addConstraint] Dangerous to add new constraints when shared by more than one SearchManager. Not allowing!");
      return false;
    }

    return addConstraint(term, incrementalImgProc, incrementalDetector, singleIteration, serialProcessing);
  }

  /**
   * Add ImageProcessor or Detector constraint. This overloaded method should
   * only be called from the GUI in order to maintain consistency across all
   * uses from the Vision API.
   *
   * @param term
   * @param incrementalImgProcFlag - if constraint is image processor, should
   * run in incremental mode
   * @param incrementalDetectorFlag - if constraint is detector, should run in
   * incremental mode
   * @param singleIterationFlag - if constraint should only run a single
   * iteration
   * @param serialProcessingFlag - if constraints should run in serial/parallel
   * @return boolean - if predicate mapped to a valid VisionProcessor
   */
  public abstract boolean addConstraint(final Term term,
          final boolean incrementalImgProcFlag, final boolean incrementalDetectorFlag,
          final boolean singleIterationFlag, final boolean serialProcessingFlag);

  /**
   * Remove constraint from ImageProcessor or Detector. First tries to remove
   * from ImageProcessor constraints, and if nothing was found matching the
   * constraint, then tries to remove Detector (and replace with default
   * Detector). NOTE: doesn't remove Detector yet!!
   *
   * @param term to remove
   * @return True, if was successfully removed. If wasn't contained, returns
   * false.
   */
  public abstract boolean removeConstraint(final Term term);
  
  /**
   * This allows notifications to be sent between SearchManagers. This 
   * SearchManager's Tracker will notify the passed in processor.
   * @param processor VisionProcess to be notified
   */
  public abstract void registerForNotification(final VisionProcess processor);
  
  /**
   * This allows notifications between SearchManagers to be canceled.
   * @param processor VisionProcess to be notified
   */
  public abstract void unregisterForNotification(final VisionProcess processor);
  
  /**
   * Get the first VisionProcess in the processing chain for this search. This is
   * usually a detector (but could conceivably be SaliencyOperators in the future).
   * @return 
   */
  public abstract VisionProcess getFirstVisionProcessor();
  
  /**
   * Get the last VisionProcess in the processing chain for this search. This is
   * usually the Tracker, but can be a Detector and Validator if there is no
   * Tracker (as is the case with sub-SearchMangers in compositeSearchManagers).
   * @return 
   */
  public abstract VisionProcess getLastVisionProcessor();

  /**
   * Get all of the VisionProcessers in the processing chain for this search. Currently
   * used for the Vision Pipeline GUI
   */
  public abstract Set <VisionProcess> getAllVisionProcessors();

  /**
   * Is this search currently running.
   *
   * @return
   */
  public abstract boolean isRunning();

  /**
   * Check if caller has called start on this VisionProcessor.
   * @return
   */
  public synchronized boolean isStartCaller(final Object caller) {
    isValid();

    return startCallers.contains(caller);
  }

  /**
   * Start detector(s), tracker(s), and constraint(s) threads of everything
   * associated with this search.
   * 
   * @param caller 
   */
  final public synchronized void start(Object caller) {
    isValid();
    log.debug(String.format("[start] typeId: %d.", typeId));

    //we only care about keeping track of different SearchManager callers, 
    //so we can lump together all other callers using null.
    if (!(caller instanceof SearchManager)) {
      caller = null;
    }

    //keep track of start/stop callers (in case this is shared by multiple SearchManagers)
    if (startCallers.contains(caller)) {
      log.warn(String.format("[start] %s already contains caller.", toString()));
      return;
    }
	

    //if not already running, try to start everything
    if (!isRunning()) {
      start();
	  performanceInfo.start();
    } else {
	  // if this is already running, count this "start" call 
	  // as a client use so SM isn't stopped prematurely
	  setTimeOfLastClientUse(System.currentTimeMillis());
	}

    //started, or already running. add to caller list
    log.debug(String.format("[start] %s adding caller to list.", toString()));
    startCallers.add(caller);
  }

  /**
   * Stop detector(s), tracker(s), and constraint(s) of everything associated
   * with this search if they are running.
   *
   * @param caller 
   * @param wait if should wait for everything to stop before returning
   */
  final public synchronized void stop(Object caller, boolean wait) {
    isValid();

    log.debug(String.format("[stop] typeId: %d. wait: %b.", typeId, wait));

    //we only care about keeping track of different SearchManager callers, 
    //so we can lump together all other callers using null.
    if (!(caller instanceof SearchManager)) {
      caller = null;
    }

    //keep track of stop/start callers
    if (!startCallers.contains(caller)) {
      log.warn(String.format("[stop] caller did not start %s.", toString()));
      return;
    } else {
      log.trace(String.format("[stop] caller did start %s.", toString()));
      startCallers.remove(caller);
    }

    //only stop/suspend if all callers of start have called stop/suspend
    if (!startCallers.isEmpty()) {
      log.trace(String.format("[stop] startCallers not empty, not stopping %s.", toString()));
      return;
    }

    stop(wait);
  }

  /**
   * Check if this SearchManager has been started by the caller.
   * @param caller
   * @return
   */
  final public synchronized boolean hasStarted(Object caller) {
    isValid();
    log.debug(String.format("[hasStarted] typeId: %d.", typeId));

    //we only care about keeping track of different SearchManager callers,
    //so we can lump together all other callers using null.
    if (!(caller instanceof SearchManager)) {
      caller = null;
    }

    if (startCallers.contains(caller)) {
      return true;
    } else {
      return false;
    }
  }

  protected abstract void start();

  protected abstract void stop(final boolean wait);
  
  final public synchronized boolean shouldBeRunning() {
    return (startCallers.size() > 0);
  }

  /**
   * Checks if at least one full detection -> tracking iteration has completed.
   * This is required in order to get detection results.
   * @return 
   */
  public abstract boolean hasIterationCompleted();

  /**
   * Set the incremental processing flag for the Detectors.
   *
   * @param flag
   */
  public abstract void setDetectorIncrementalProcessing(final boolean flag);

  /**
   * Get incremental processing flag for selected Detectors. Returns
   * "incrementalDetectorFlag" if no detector exists (as default preference to
   * use incremental processing).
   *
   * @return
   */
  public abstract boolean getDetectorIncrementalProcessing();

  /**
   * Set the incremental processing flag for all image processors.
   *
   * @param flag
   */
  public abstract void setConstraintsIncrementalProcessing(final boolean flag);

  /**
   * Get incremental processing flag for all image processors. Will print error
   * if they are not consistent. Returns true if no image processors exist (as
   * default preference to use incremental processing).
   *
   * @return
   */
  public abstract boolean getConstraintsIncrementalProcessing();

  public abstract void setSingleIteration(final boolean flag);

  public abstract boolean getSingleIteration();

  public abstract void setSerialProcessing(final boolean flag);

  public abstract boolean getSerialProcessing();

  /**
   * Update introspection info: time of last client use.
   *
   * @param time
   */
  public abstract void setTimeOfLastClientUse(long time);
  
    /**
   * Does the search meet the expectation of its set analysis policy. This is
   * for vision introspection.
   *
   * @return
   */
  public abstract Collection<TaskAnalysisPolicy.Result> meetsExpectations();

  /**
   * This method must be called when a SearchManager is "deleted" so that
   * Detectors, Trackers, and ImageProcessors can be "freed" from their
   * respective factories (AvailableTrackers, AvailableDetctors, etc.). A
   * SearchManager is no longer valid after this method is called.
   */
  abstract void terminate();

  protected void isValid() {
    if (!validFlag) {
      throw new IllegalStateException("SearchManager has been terminated and is no longer valid.");
    }
  }
}
