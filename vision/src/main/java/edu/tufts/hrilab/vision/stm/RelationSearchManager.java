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
import edu.tufts.hrilab.vision.detector.Detector;
import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.reflection.TaskAnalysisPolicy;
import edu.tufts.hrilab.vision.tracker.Tracker;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;

import java.util.*;

/**
 * @author Evan Krause
 */
public class RelationSearchManager extends SearchManager {

  private Detector detector = null;
  private Tracker tracker = null;
  private List<Detector> availableDetectors = new ArrayList();
  private List<Tracker> availableTrackers = new ArrayList();
  private Term relation = null;
  //SearchManagers that collectively make this SearchManager
  SearchManager referentSearchManager;
  SearchManager relatumSearchManager;

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
  RelationSearchManager(long typeId, boolean incrementalImgProcFlag, boolean incrementalDetectorFlag,
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
  RelationSearchManager(boolean incrementalImgProcFlag, boolean incrementalDetectorFlag,
                        boolean serialProcessingFlag, boolean singleIterationFlag) {
    super(incrementalImgProcFlag, incrementalDetectorFlag, serialProcessingFlag, singleIterationFlag);
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
     */
    boolean addedConstraint = false;
//    Set<Variable> vars = term.getVars();
//    if (vars.size() == 2) {
//      addedConstraint = addDetectorConstraint(term, incrementalImgProcFlag,
//              singleIterationFlag, serialProcessingFlag);
//    } else {
//      SearchManager searchType = searchTypes.get(vars);
//      if (searchType == null) {
//        //create new searchType
//        List<Term> tmpDescription = new ArrayList();
//        tmpDescription.add(term);
//        searchType = Vision.availableSearchTypes.getInstance(this, tmpDescription);
//        if (searchType != null) {
//          searchTypes.put(vars, searchType);
//          addedConstraint = true;
//        }
//      } else {
//        //just try to add constraint
//        addedConstraint = searchType.addConstraint(term, incrementalImgProcFlag,
//                incrementalDetectorFlag, singleIterationFlag, serialProcessingFlag);
//      }
//    }

    return addedConstraint;
  }

  public synchronized boolean addConstraints(Map<List<Variable>, ? extends List<? extends Term>> separatedDescriptors) {
    isValid();

    // make deep copy, so we can remove descriptors as they're sorted
    Map<List<Variable>, ? extends List<? extends Term>> descriptorsToSort = new HashMap<>(separatedDescriptors);

    // first find a relation descriptor that is a detector
    for (Map.Entry<List<Variable>, ? extends List<? extends Term>> entry : descriptorsToSort.entrySet()) {
      // if has two vars and one term
      if (entry.getKey().size() == 2 && entry.getValue().size() == 1) {
        if (Vision.availableDetectors.hasCapableDetector(entry.getValue().get(0))) {
          relation = entry.getValue().get(0);
          descriptorsToSort.remove(entry.getKey());
          break;
        }
      }
    }

    if (relation == null) {
      log.error("[addConstraints] no relation with Detector found.");
      return false;
    }

    // then separate all other descriptors into referent and relatum descriptors
    List<Term> referentDescriptors = getSeparatedDescriptors(descriptorsToSort, (Variable) relation.get(0));
    List<Term> relatumDescriptors = getSeparatedDescriptors(descriptorsToSort, (Variable) relation.get(1));

    // check that all descriptors were separated
    if (!descriptorsToSort.isEmpty()) {
      log.error("[addConstraints] could not separate descriptors into referent and relatum.");
      return false;
    }

    // get detector (and tracker)
    if (!addDetectorConstraint(relation, incrementalDetector, singleIteration, serialProcessing)) {
      return false;
    }

    // then build a new SearchManager (or get existing one) for referent descriptors and relatum descriptors
    referentSearchManager = Vision.availableSearchTypes.getInstance(this, referentDescriptors, true);
    relatumSearchManager = Vision.availableSearchTypes.getInstance(this, relatumDescriptors, true);
    if (referentSearchManager == null || relatumSearchManager == null) {
      return false;
    }

    description.addAll(referentDescriptors);
    description.addAll(relatumDescriptors);
    return true;
  }

  /**
   * Helper method to pull out all descriptors from a in list of descriptors related to a target variable. For instance,
   * "object(A), person(B), gp(C), on(C,A)" with target variable "A" will return "object(A), on(C,A), gp(C)" because
   * "A" is related to "C" via the "on" descriptor but nothing relates "B" to "A".
   *
   * @param descriptorsToSort list of descriptors to sort -- this is modified during method call
   * @param varToSort target variable
   * @return
   */
  private List<Term> getSeparatedDescriptors(Map<List<Variable>, ? extends List<? extends Term>> descriptorsToSort, Variable varToSort) {
    List<Variable> varsToSort = new ArrayList<>();
    varsToSort.add(varToSort);

    List<Term> separatedDescriptors = new ArrayList<>();
    while (!varsToSort.isEmpty()) {
      Variable referentVar = varsToSort.remove(0);
      for (Iterator<? extends Map.Entry<List<Variable>, ? extends List<? extends Term>>> itr = descriptorsToSort.entrySet().iterator(); itr.hasNext(); ) {
        Map.Entry<List<Variable>, ? extends List<? extends Term>> entry = itr.next();
        if (entry.getKey().size() == 1) {
          // single variable descriptor
          Variable var0 = entry.getKey().get(0);
          if (var0.getName().equals(referentVar.getName())) {
            // if referent variable
            separatedDescriptors.addAll(entry.getValue());
            itr.remove();
          }

        } else if (entry.getKey().size() == 2) {
          // two variable descriptor
          Variable var0 = entry.getKey().get(0);
          Variable var1 = entry.getKey().get(1);
          if (var0.getName().equals(referentVar.getName())) {
            separatedDescriptors.addAll(entry.getValue());
            varsToSort.add(var1); // add other arg to referent vars to process
            itr.remove();
          } else if (var1.getName().equals(referentVar.getName())) {
            separatedDescriptors.addAll(entry.getValue());
            varsToSort.add(var0); // add other var to referent vars to process
            itr.remove();
          }

        } else {
          log.error("[addConstraints] can't handle descriptors with > 2 arguments: " + entry.getValue());
          return new ArrayList<>();
        }
      }
    }

    return separatedDescriptors;
  }

  private boolean addDetectorConstraint(final Term term, final boolean incrementalFlag,
                                        final boolean singleIterationFlag, final boolean serialProcessingFlag) {
    if (detector != null) {
      log.error(String.format("[addConstraint] Detector type has already been "
              + "selected for SearchManager: %d. Can not re-assign Detector.", typeId));
      return false;
    }

    List<Detector> newDetectors = Vision.availableDetectors.getInstances(this, term);
    if (!newDetectors.isEmpty()) {
      //if more than one detector option, just choosing first one for now
      detector = newDetectors.get(0);
      availableDetectors.addAll(newDetectors);

      //register detector to be notified by its own dependencies
      List<ImageProcessorType> detectorDependencyType = detector.getDependencies();
      for (ImageProcessorType dependencyType : detectorDependencyType) {
        ImageProcessor dependency = detector.getDependency(dependencyType);
        dependency.registerForNotification(detector, typeId);
      }

      //get/add trackers from all detector options (avoiding duplicate types)
      for (Detector d : newDetectors) {
        for (NativeTracker.TrackerType usableTrackerType : d.getUsableTrackers()) {
          //avoiding duplicate types
          boolean alreadyAdded = false;
          for (Tracker addedTracker : availableTrackers) {
            if (addedTracker.isType(usableTrackerType)) {
              alreadyAdded = true;
              break;
            }
          }
          if (!alreadyAdded) {
            Tracker newTracker = Vision.availableTrackers.getInstance(this, usableTrackerType);
            if (newTracker == null) {
              log.error("[addDetectorConstraint] couldn't find usable Tracker: " + usableTrackerType);
              return false;
            }
            availableTrackers.add(newTracker);
          }
        }

        //set selected detector's tracker and also check to make sure
        //there's at least one tracker available for every detector option
        if (!availableTrackers.isEmpty()) {
          if (d.equals(detector)) {
            tracker = availableTrackers.get(0);

            //register to notify tracker
            detector.registerForNotification(tracker, typeId);
          }
        } else {
          log.error(String.format("[addDetectorConstraint] No Trackers available for %s detector.", detector.getType().toString()));
        }
      }
    } else {
      log.error(String.format("[addDetectorConstraint] No Detectors available for %s.", term.toString()));
      return false;
    }

    if (tracker == null) {
      log.error(String.format("[addDetectorConstraint] No Tracker available for %s detector.", detector.getType().toString()));
      return false;
    }

    //add relation to Detector, Tracker, and SearchManager description
    detector.addProcessingDescriptor(term, this);
    tracker.addProcessingDescriptor(term, this);
    description.add(term);

    //set flags for all new detectors
    for (Detector d : newDetectors) {
      //set single iteration flag
      d.setSingleIteration(singleIterationFlag);

      //set incremental processing
      d.setIncrementalProcessing(incrementalFlag);

      //set serial processing
      d.setSerialProcessing(serialProcessingFlag);
    }

    //start detector and tracker
    if (shouldBeRunning()) {
      detector.start(this);
      tracker.start(this);
    }

    return true;
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

    SearchManager searchManager = null;
    if (referentSearchManager.getDescriptors().contains(term)) {
      searchManager = referentSearchManager;
    } else if (relatumSearchManager.getDescriptors().contains(term)) {
      searchManager = relatumSearchManager;
    } else {
      log.error("[removeConstraint] coulnd't find SearchManager with constraint: " + term);
      return false;
    }

    if (searchManager == null) {
      log.warn("[removeConstraint] couldn't find SearchManager to remove constraint from.");
      return false;
    }

    if (searchManager.removeConstraint(term)) {
      log.warn(String.format("[removeConstraint] could not remove: %s.", term.toString()));
      return false;
    }

    //keep all descriptor info in trackers
    if (tracker != null) {
      tracker.removeProcessingDescriptor(term, this);
    }

    //fire event on ui event queue to update description
    Vision.availableSearchTypes.updateGuiEntry(this);
    return true;
  }

  @Override
  public VisionProcess getFirstVisionProcessor() {
    return detector;
  }

  @Override
  public VisionProcess getLastVisionProcessor() {
    return tracker;
  }

  @Override
  public Set <VisionProcess> getAllVisionProcessors() {
    Set<VisionProcess> results = new HashSet();
    results.add(detector);
    results.add(tracker);
    results.addAll(referentSearchManager.getAllVisionProcessors());
    results.addAll(relatumSearchManager.getAllVisionProcessors());
    return results;
  }

  @Override
  public List<Long> getRelatedTypeIds() {
    List<Long> ids = new ArrayList<>();
    ids.add(getTypeId());
    ids.addAll(referentSearchManager.getRelatedTypeIds());
    ids.addAll(relatumSearchManager.getRelatedTypeIds());
    return ids;
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

    incrementalDetector = flag;
    detector.setIncrementalProcessing(flag);
    referentSearchManager.setDetectorIncrementalProcessing(flag);
    relatumSearchManager.setDetectorIncrementalProcessing(flag);
  }

  /**
   * Get incremental processing flag for selected Detectors.
   *
   * @return
   */
  @Override
  public synchronized boolean getDetectorIncrementalProcessing() {
    isValid();

    return incrementalDetector;
  }

  /**
   * Set the incremental processing flag for all image processors.
   *
   * @param flag
   */
  @Override
  public synchronized void setConstraintsIncrementalProcessing(final boolean flag) {
    isValid();

    incrementalImgProc = flag;
    detector.setIncrementalProcessing(flag);
    tracker.setIncrementalProcessing(flag);
    referentSearchManager.setConstraintsIncrementalProcessing(flag);
    relatumSearchManager.setConstraintsIncrementalProcessing(flag);
  }

  /**
   * Get incremental processing flag for all image processors.
   *
   * @return
   */
  @Override
  public synchronized boolean getConstraintsIncrementalProcessing() {
    isValid();

    return incrementalImgProc;
  }

  @Override
  public synchronized void setSingleIteration(final boolean flag) {
    isValid();

    singleIteration = flag;
    detector.setSingleIteration(flag);
    tracker.setSingleIteration(flag);
    referentSearchManager.setSingleIteration(flag);
    relatumSearchManager.setSingleIteration(flag);
  }

  @Override
  public synchronized boolean getSingleIteration() {
    isValid();

    return singleIteration;
  }

  @Override
  public synchronized void setSerialProcessing(final boolean flag) {
    isValid();

    serialProcessing = flag;
    detector.setSerialProcessing(flag);
    tracker.setSerialProcessing(flag);
    referentSearchManager.setSerialProcessing(flag);
    relatumSearchManager.setSerialProcessing(flag);
  }

  @Override
  public synchronized boolean getSerialProcessing() {
    isValid();

    return serialProcessing;
  }

  /**
   * Are this search's detector, tracker, and sub-SearchManagers currently
   * running.
   *
   * @return
   */
  @Override
  public synchronized boolean isRunning() {
    isValid();

    if (referentSearchManager == null || relatumSearchManager == null || detector == null || tracker == null) {
      return false;
    }

    //check if all SearchManagers and detector and tracker are running
    if (!referentSearchManager.isStartCaller(this) || !relatumSearchManager.isStartCaller(this)
            || !detector.isStartCaller(this) || !tracker.isStartCaller(this)) {
      return false;
    }

    return true;
  }

  /**
   * Start type's detection, tracker, and sub-SearchManagers threads.
   */
  @Override
  final protected synchronized void start() {
    isValid();
    log.debug(String.format("[start] id: %d.", typeId));

    //start both sub-SearchManagers and register detector to be notified by both
    referentSearchManager.registerForNotification(detector);
    relatumSearchManager.registerForNotification(detector);

    referentSearchManager.start(this);
    relatumSearchManager.start(this);

    detector.start(this);
    tracker.start(this);
  }

  /**
   * Stop constraints, detectors, and trackers if they are running.
   *
   * @param wait if should wait for everything to stop before returning
   */
  @Override
  final protected synchronized void stop(final boolean wait) {
    isValid();

    //stop both sub-SearchManagers and unregister detector;
    referentSearchManager.unregisterForNotification(detector);
    relatumSearchManager.unregisterForNotification(detector);

    referentSearchManager.stop(this, wait);
    relatumSearchManager.stop(this, wait);

    detector.stop(this, wait);
    tracker.stop(this, wait);
  }

  @Override
  public synchronized boolean hasIterationCompleted() {
    isValid();

    if (!tracker.hasIterationCompleted(typeId)) {
      return false;
    }

    return true;
  }

  @Override
  public synchronized void setTimeOfLastClientUse(long time) {
    isValid();

    performanceInfo.setTimeOfLastClientUse(time);
    referentSearchManager.setTimeOfLastClientUse(time);
    relatumSearchManager.setTimeOfLastClientUse(time);
    detector.getPerformanceInfo().setTimeOfLastClientUse(time);
    tracker.getPerformanceInfo().setTimeOfLastClientUse(time);
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
    Vision.availableSearchTypes.release(this, referentSearchManager);
    Vision.availableSearchTypes.release(this, relatumSearchManager);

    //"release" detectors
    for (Detector d : availableDetectors) {
      Vision.availableDetectors.release(this, d);
    }

    //"release" trackers
    for (Tracker t : availableTrackers) {
      Vision.availableTrackers.release(this, t);
    }
  }
}
