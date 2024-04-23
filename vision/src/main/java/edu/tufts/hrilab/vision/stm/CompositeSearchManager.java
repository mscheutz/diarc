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
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.common.swig.CommonModule;
import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.reflection.TaskAnalysisPolicy;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;
import edu.tufts.hrilab.vision.tracker.Tracker;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;

import java.util.*;

/**
 * @author Evan Krause
 */
final public class CompositeSearchManager extends SearchManager {

  /**
   * SearchManagers that collectively make this SearchManager.
   */
  private final Map<List<Variable>, SearchManager> searchTypes = new HashMap();
  /**
   * Constraints between lower-level searchTypes. This is realized by Validators
   * and is often relations such as on(X,Y), near(W,Z).
   */
  private final Map<List<Variable>, ImageProcessor> validationConstraints = new HashMap();
  /**
   * Chain of VisionProcessor registrations made by this CompositeSearchManager
   * between searchTypes and validationConstraints. Note that this chain doesn't
   * include all VisionProcessors used by this search, such as those completely
   * contained inside a searchType.
   */
  private final List<VisionProcess> processorChain = new LinkedList<>();
  /**
   * The object tracker for this search.
   */
  private Tracker tracker;

  /**
   * Constructor is package-private so that the AvailableSearchMangers factory
   * is the only way a SearchManager can be instantiated. Use this constructor
   * when this SearchManager is going to be part of a hierarchical search (e.g.,
   * as a sub-type in another SearchManager). In this case, the typeId should
   * be that of the parent SearchManager.
   *
   * @param typeId                  typeId of parent SearchManager
   * @param incrementalImgProcFlag
   * @param incrementalDetectorFlag
   * @param serialProcessingFlag
   * @param singleIterationFlag
   */
  CompositeSearchManager(long typeId, boolean incrementalImgProcFlag, boolean incrementalDetectorFlag,
                         boolean serialProcessingFlag, boolean singleIterationFlag) {
    super(typeId, incrementalImgProcFlag, incrementalDetectorFlag, serialProcessingFlag, singleIterationFlag);
  }

  /**
   * Constructor is package-private so that the AvailableSearchMangers factory
   * is the only way a SearchManager can be instantiated.
   *
   * @param incrementalImgProcFlag
   * @param incrementalDetectorFlag
   * @param serialProcessingFlag
   * @param singleIterationFlag
   */
  CompositeSearchManager(boolean incrementalImgProcFlag, boolean incrementalDetectorFlag,
                         boolean serialProcessingFlag, boolean singleIterationFlag) {
    super(incrementalImgProcFlag, incrementalDetectorFlag, serialProcessingFlag, singleIterationFlag);
  }

  /**
   * More efficient alternative to addConstraint when all (or at least more than
   * one) constraints are known at a time.
   *
   * @param separatedDescriptors
   * @return if all constraints (descriptors) were met
   */
  public synchronized boolean addConstraints(Map<List<Variable>, ? extends List<? extends Term>> separatedDescriptors) {

    // build a new SearchManager (or get existing one) for each set of Variables
    SearchManager searchManager;
    for (Map.Entry<List<Variable>, ? extends List<? extends Term>> entry : separatedDescriptors.entrySet()) {

      if (entry.getKey().size() == 1) {
        // a single variable

        searchManager = Vision.availableSearchTypes.getInstance(this, entry.getValue(), false);

        if (searchManager != null) {
          searchTypes.put(entry.getKey(), searchManager);
          //set description
          for (Term p : entry.getValue()) {
            description.add(p);
          }
        } else {
          log.info(String.format("[addConstraints] Could not instantiate SearchManager: %s.", entry.getValue().toString()));
          return false;
        }

      } else if (entry.getKey().size() == 2) {
        // two variables -- each predicate is treated as separate validator even
        // if they have the same variables

        for (Term p : entry.getValue()) {
          ImageProcessor validationConstraint = Vision.availableValidationProcessors.getInstance(this, p);
          if (validationConstraint != null) {
            validationConstraints.put(entry.getKey(), validationConstraint);

            validationConstraint.addProcessingDescriptor(p, this);

            //set description
            description.add(p);
          } else {
            log.error("[addConstraints] No validation constraint could be found for: " + p);
            return false;
          }
        }
      } else {
        log.error(String.format("[addConstraints] Can't handle Predicates with %d Variables.", entry.getKey().size()));
        return false;
      }
    }

    // order the constraints
    List<List<Variable>> orderVariables = PredicateHelper.orderVariableLists(separatedDescriptors.keySet());

    // chain together all the search managers and non-search manager constraints
    VisionProcess lastVisionProcessor = null;   // last proc of previous iteration
    VisionProcess currVisionProcessor = null;
    for (List<Variable> vars : orderVariables) {
      // look through SearchManagers first
      SearchManager manager = searchTypes.get(vars);
      if (manager != null) {
        currVisionProcessor = manager.getFirstVisionProcessor();
        if (currVisionProcessor != null) {
          if (lastVisionProcessor != null) {
            lastVisionProcessor.registerForNotification(currVisionProcessor, typeId);
          }
          processorChain.add(currVisionProcessor);
          lastVisionProcessor = manager.getLastVisionProcessor();
        }
      }

      currVisionProcessor = validationConstraints.get(vars);
      if (currVisionProcessor != null) {
        if (lastVisionProcessor != null) {
          lastVisionProcessor.registerForNotification(currVisionProcessor, typeId);
        }
        processorChain.add(currVisionProcessor);
        lastVisionProcessor = currVisionProcessor;
      }
    }

    // set tracker
    setTracker();

    return true;
  }

  private void setTracker() {
    // finish any initialization and registration
    if (tracker == null) {
      //TODO: how should tracker type be configured?
      if (CommonModule.hasOpenCVTracking()) {
        tracker = Vision.availableTrackers.getInstance(this, TrackerType.KCF);
      } else {
        tracker = Vision.availableTrackers.getInstance(this, TrackerType.CMT);
      }
      for (Term t : description) {
        tracker.addProcessingDescriptor(t, this);
      }
    }
  }

  @Override
  public void addSearchManager(List<Variable> variables, SearchManager searchManager) {
    if (searchTypes.get(variables) != null) {
      log.error("[addSearchManager] Overwriting existing SearchManager.");
    }
    searchTypes.put(variables, searchManager);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized boolean addConstraint(final Term term,
                                            final boolean incrementalImgProcFlag, final boolean incrementalDetectorFlag,
                                            final boolean singleIterationFlag, final boolean serialProcessingFlag) {
    isValid();

    /*
     * TODO: A lot of additional logic needs to go in here to determine if constraints
     * should be added to existing searches. 
     * 
     * NOTES: Is it possible that existing SingleSearchManagers will need to be "upgraded" 
     * to a CompositeSearchManager or removed from this CompositeSearchManager and
     * attached to new CompositeSearchManager which is used by this CompositeSearchManager?
     */
    List<Variable> vars = term.getOrderedVars();
    SearchManager searchType = searchTypes.get(vars);
    boolean addedConstraint = false;
    if (searchType == null) {
      //create new searchType
      List<Term> tmpDescription = new ArrayList();
      tmpDescription.add(term);
      searchType = Vision.availableSearchTypes.getInstance(this, tmpDescription, false);
      if (searchType != null) {
        searchTypes.put(vars, searchType);
        addedConstraint = true;
      }
    } else {
      //just try to add constraint
      addedConstraint = searchType.addConstraint(term,
              incrementalImgProcFlag, incrementalDetectorFlag, singleIterationFlag, serialProcessingFlag);
    }

    //if predicate maps to a vision process, add to our local description
    if (addedConstraint) {
      description.add(term);
      if (tracker != null) {
        tracker.addProcessingDescriptor(term, this);
      }
    }

    return addedConstraint;
  }

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
  @Override
  public synchronized boolean removeConstraint(final Term term) {
    isValid();

    List<Variable> vars = term.getOrderedVars();
    SearchManager memoryObjectType = searchTypes.get(vars);
    if (memoryObjectType == null) {
      log.warn("[removeConstraint] couldn't find SearchManager to remove constraint from.");
      return false;
    }

    if (memoryObjectType.removeConstraint(term)) {
      log.warn(String.format("[removeConstraint] could not remove: %s.", term.toString()));
      return false;
    }

    //fire event on ui event queue to update description
    Vision.availableSearchTypes.updateGuiEntry(this);
    return true;
  }

  @Override
  public synchronized List<Long> getRelatedTypeIds() {
    List<Long> ids = new ArrayList<>();
    ids.add(getTypeId());
    for (SearchManager current : searchTypes.values()) {
      if (current.getTypeId() != getTypeId())
        ids.add(current.getTypeId());
    }
    return ids;
  }

  @Override
  public VisionProcess getFirstVisionProcessor() {
    return processorChain.get(0);
  }

  @Override
  public VisionProcess getLastVisionProcessor() {
    return tracker;
  }

  @Override
  public Set<VisionProcess> getAllVisionProcessors() {
    Set<VisionProcess> results = new HashSet<>();
    if (!processorChain.isEmpty()) {
      results.addAll(processorChain);
    }

    for (SearchManager current : searchTypes.values())
    {
      results.addAll(current.getAllVisionProcessors());
    }

    results.add(tracker);
    return results;
  }

  @Override
  public void registerForNotification(VisionProcess processor) {
    if (tracker == null) {
      log.error("[registerForNotification] trying to register with null Tracker.");
    }
    tracker.registerForNotification(processor, typeId);
  }

  @Override
  public void unregisterForNotification(VisionProcess processor) {
    if (tracker == null) {
      log.error("[unregisterForNotification] trying to unregister with null Tracker.");
    }
    tracker.unregisterForNotification(processor, typeId);
  }

  /**
   * Set the incremental processing flag for the Detectors.
   *
   * @param flag
   */
  @Override
  public synchronized void setDetectorIncrementalProcessing(final boolean flag) {
    isValid();

    for (SearchManager searchType : searchTypes.values()) {
      searchType.setDetectorIncrementalProcessing(flag);
    }
  }

  /**
   * Get incremental processing flag for selected Detectors. Returns
   * "incrementalDetectorFlag" if no detector exists (as default preference to
   * use incremental processing).
   *
   * @return
   */
  @Override
  public synchronized boolean getDetectorIncrementalProcessing() {
    isValid();

    if (searchTypes.isEmpty()) {
      return incrementalDetector;
    }

    //verify that all detector are consistent
    boolean first = true;
    boolean flag = false;
    for (SearchManager searchType : searchTypes.values()) {
      if (first) {
        first = false;
        flag = searchType.getDetectorIncrementalProcessing();
        continue;
      }
      if (searchType.getDetectorIncrementalProcessing() != flag) {
        log.error("[getDetectorIncrementalProcessing]: Incremental Processing flags are not consistent.");
      }
    }

    return flag;
  }

  /**
   * Set the incremental processing flag for all image processors.
   *
   * @param flag
   */
  @Override
  public synchronized void setConstraintsIncrementalProcessing(final boolean flag) {
    isValid();

    for (SearchManager searchType : searchTypes.values()) {
      searchType.setConstraintsIncrementalProcessing(flag);
    }
  }

  /**
   * Get incremental processing flag for all image processors. Will print error
   * if they are not consistent. Returns true if no image processors exist (as
   * default preference to use incremental processing).
   *
   * @return
   */
  @Override
  public synchronized boolean getConstraintsIncrementalProcessing() {
    isValid();

    if (searchTypes.isEmpty()) {
      return incrementalImgProc;
    }

    //verify that all detector are consistent
    boolean first = true;
    boolean flag = false;
    for (SearchManager searchType : searchTypes.values()) {
      if (first) {
        first = false;
        flag = searchType.getConstraintsIncrementalProcessing();
        continue;
      }
      if (searchType.getConstraintsIncrementalProcessing() != flag) {
        log.error("[getConstraintsIncrementalProcessing]: Incremental Processing flags are not consistent.");
      }
    }

    return flag;
  }

  @Override
  public synchronized void setSingleIteration(final boolean flag) {
    isValid();

    for (SearchManager searchType : searchTypes.values()) {
      searchType.setSingleIteration(flag);
    }
  }

  @Override
  public synchronized boolean getSingleIteration() {
    isValid();

    if (searchTypes.isEmpty()) {
      return singleIteration;
    }

    //verify that all SearchManagers are consistent
    boolean first = true;
    boolean flag = false;
    for (SearchManager searchType : searchTypes.values()) {
      if (first) {
        first = false;
        flag = searchType.getSingleIteration();
        continue;
      }
      if (searchType.getSingleIteration() != flag) {
        log.error("[getSingleIteration]: Single Iteration flags are not consistent.");
      }
    }

    return flag;
  }

  @Override
  public synchronized void setSerialProcessing(final boolean flag) {
    isValid();

    for (SearchManager searchType : searchTypes.values()) {
      searchType.setSerialProcessing(flag);
    }
  }

  @Override
  public synchronized boolean getSerialProcessing() {
    isValid();

    if (searchTypes.isEmpty()) {
      return singleIteration;
    }

    //verify that all SearchManagers are consistent
    boolean first = true;
    boolean flag = false;
    for (SearchManager searchType : searchTypes.values()) {
      if (first) {
        first = false;
        flag = searchType.getSerialProcessing();
        continue;
      }
      if (searchType.getSerialProcessing() != flag) {
        log.error("[getSerialProcessing]: Serial Processing flags are not consistent.");
      }
    }

    return flag;
  }

  /**
   * Is this search currently running.
   *
   * @return
   */
  @Override
  public synchronized boolean isRunning() {
    isValid();

    if (searchTypes.isEmpty()) {
      return false;
    }

    //check if all SearchManagers and VisionProcessors are running
    for (SearchManager searchType : searchTypes.values()) {
      if (!searchType.isStartCaller(this)) {
        return false;
      }
    }

    for (ImageProcessor imageProc : validationConstraints.values()) {
      if (!imageProc.isStartCaller(this)) {
        return false;
      }
    }

    if (tracker == null || !tracker.isStartCaller(this)) {
      return false;
    }

    return true;
  }

  /**
   * Start type's detection, tracker, and constraints threads.
   */
  @Override
  final protected synchronized void start() {
    isValid();
    log.debug(String.format("[start] id: %d.", typeId));

    // finish any initialization and registration
    setTracker();

    // finish any registration
    if (!processorChain.isEmpty()) {
      processorChain.get(0).registerForCaptureNotification(); //TODO: how to generalize what needs this?
      processorChain.get(processorChain.size() - 1).registerForNotification(tracker, typeId);
    }

    // start all sub-SearchManagers
    for (SearchManager searchType : searchTypes.values()) {
      searchType.start(this);
    }

    // start all validators
    for (ImageProcessor imageProc : validationConstraints.values()) {
      imageProc.start(this);
    }

    // start tracker
    tracker.start(this);

    //EAK: no idea what this is for -- delete this???
//    log.trace(String.format("[start] called: %s. submitting start to executor.", toString()));
//    final Object thisProcessor = this;
//    executor.submit(new Runnable() {
//      @Override
//      public void run() {
//        log.trace(String.format("[start] called: %s. running job.", thisProcessor.toString()));
//        try {
//          for (SearchManager searchType : searchTypes.values()) {
//            long typeId = searchType.getTypeId();
//            ShortTermMemoryInterface.getTokenIds(typeId, 0.0);
//          }
//        } catch (Exception e) {
//          log.error("[start] Exception caught in perform.", e);
//        }
//      }
//    });
  }

  /**
   * Stop constraints, detectors, and trackers if they are running.
   *
   * @param wait if should wait for everything to stop before returning
   */
  @Override
  final protected synchronized void stop(final boolean wait) {
    isValid();

    // stop all sub-SearchManagers
    for (SearchManager searchType : searchTypes.values()) {
      searchType.stop(this, wait);
    }

    // stop all validators
    for (ImageProcessor imageProc : validationConstraints.values()) {
      imageProc.stop(this, wait);
    }

    // stop tracker
    if (tracker != null) {
      tracker.stop(this, wait);
    }

  }

  @Override
  public synchronized boolean hasIterationCompleted() {
    isValid();

    if (tracker == null) {
      return false;
    }

    return tracker.hasIterationCompleted(typeId);
  }

  @Override
  public synchronized void setTimeOfLastClientUse(long time) {
    isValid();

    performanceInfo.setTimeOfLastClientUse(time);
    for (SearchManager searchType : searchTypes.values()) {
      searchType.setTimeOfLastClientUse(time);
    }

    for (ImageProcessor imageProc : validationConstraints.values()) {
      imageProc.getPerformanceInfo().setTimeOfLastClientUse(time);
    }

    if (tracker != null) {
      tracker.getPerformanceInfo().setTimeOfLastClientUse(time);
    }
  }

  @Override
  public synchronized Collection<TaskAnalysisPolicy.Result> meetsExpectations() {
    isValid();
    return analysisPolicy.analyse(performanceInfo);
  }

  /**
   * This method must be called when a SearchManager is "deleted" so that
   * Detectors, Trackers, and ImageProcessors can be "freed" from their
   * respective factories (AvailableTrackers, AvailableDetctors, etc.). A
   * SearchManager is no longer valid after this method is called.
   */
  @Override
  synchronized void terminate() {
    isValid();

    //should already be stopped, but just in case...
    //stop();
    validFlag = false;

    //"release" SearchManagers
    for (SearchManager searchType : searchTypes.values()) {
      Vision.availableSearchTypes.release(this, searchType);
    }
    searchTypes.clear();

    for (ImageProcessor imageProc : validationConstraints.values()) {
      Vision.availableValidationProcessors.release(this, imageProc);
    }
    validationConstraints.clear();

    if (tracker != null) {
      Vision.availableTrackers.release(this, tracker);
      tracker = null;
    }
  }
}
