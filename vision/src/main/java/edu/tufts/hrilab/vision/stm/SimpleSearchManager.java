/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.vision.reflection.TaskAnalysisPolicy;
import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.detector.Detector;
import edu.tufts.hrilab.vision.detector.swig.NativeDetector.DetectorType;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.tracker.Tracker;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;

import java.util.*;
import javax.swing.DefaultListModel;
import javax.swing.ListModel;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;

/**
 * A SimpleSearchManager is capable of visually searching (detecting and
 * tracking) for a certain object type (e.g., face, coke can, etc.) or visual
 * region with certain properties (e.g., red, round, etc). A SimpleSearchManager
 * can have multiple means of detection and/or tracking but is guaranteed to
 * have at least one instantiated Detector. Using a Tracker is optional
 * depending on if this is a standalone search manager or part of a
 * CompositeSearchManager.
 *
 * ============ IMPORTANT ==================== (1) terminate() must be called
 * when a SearchManager is no longer used. (2) Every non-private class method
 * must be synchronized and call isValid() at the beginning of the method.
 * ===========================================
 *
 * @author Evan Krause
 */
final public class SimpleSearchManager extends SearchManager {

  private Detector detector = null;
  private Tracker tracker = null;
  private final DefaultListModel<ImageProcessor> saliencyOperators = new DefaultListModel();   //based on saliency map constraints. TODO: should these be combined with dependencies ??
  private final DefaultListModel<ImageProcessor> validationConstraints = new DefaultListModel();   //based on validator constraints. TODO: should these be combined with dependencies ??
  private final Map<DetectorType, Detector> availableDetectors = new HashMap<>();
  private final Map<TrackerType, Tracker> availableTrackers = new HashMap<>();
  final boolean standAloneSearch; //part of higher-level composite search manager
  final boolean parallelValidators = false; //serial vs parallel

  /**
   * Constructor is package-private so that the AvailableSearchManagers factory
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
  SimpleSearchManager(long typeId, boolean incrementalImgProcFlag, boolean incrementalDetectorFlag, boolean serialProcessingFlag, boolean singleIterationFlag) {
    super(typeId, incrementalImgProcFlag, incrementalDetectorFlag, serialProcessingFlag, singleIterationFlag);
    this.standAloneSearch = false;
  }

  /**
   * Constructor is package-private so that the AvailableSearchManagers factory
   * is the only way a SearchManager can be instantiated. Use this constructor
   * when this SearchManager is a stand-alone SearchManager (i.e., not part of a
   * hierarchical search).
   *
   * @param incrementalImgProcFlag
   * @param incrementalDetectorFlag
   * @param serialProcessingFlag
   * @param singleIterationFlag
   */
  SimpleSearchManager(boolean incrementalImgProcFlag, boolean incrementalDetectorFlag, boolean serialProcessingFlag, boolean singleIterationFlag) {
    super(incrementalImgProcFlag, incrementalDetectorFlag, serialProcessingFlag, singleIterationFlag);
    this.standAloneSearch = true;
  }

  /**
   * Print the Detector and Tracker introspection statistics in
   * {@code directory/detector_<id>.txt} and {@code directory/tracker_<id>.txt}
   *
   * @param directory
   */
  public synchronized void printStats(String directory) {
    isValid();

    if (detector != null) {
      detector.getPerformanceInfo().print(directory + "detector_" + typeId + ".txt");
    }
    if (tracker != null) {
      tracker.getPerformanceInfo().print(directory + "tracker_" + typeId + ".txt");
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized boolean addConstraint(final Term term,
                                            final boolean incrementalImgProcFlag, final boolean incrementalDetectorFlag,
                                            final boolean singleIterationFlag, final boolean serialProcessingFlag) {
    isValid();

    boolean addedDetector = false;
    boolean addedConstraint = false;
    //prefer a validation process first, and a detector second
    if (Vision.availableValidationProcessors.hasCapableProcessorType(term)) {
      //add image processor
      addedConstraint = addValidationConstraint(term, incrementalImgProcFlag, singleIterationFlag, serialProcessingFlag);
    } else if (Vision.availableDetectors.hasCapableDetector(term)) {
      //add detector
      addedDetector = addDetectorConstraint(term, incrementalDetectorFlag, singleIterationFlag, serialProcessingFlag);
    }
    //always try to get a saliency operator to help guide visual attention
    if (Vision.availableSaliencyProcessors.hasCapableProcessorType(term)) {
      //add saliency processor
      //NOTE: saliency doesn't contribute to whether or not addedConstraint is true.
      //This is because saliency does not contribute to validating what type
      //of object is detected/tracked.
      addSaliencyConstraint(term, incrementalImgProcFlag, singleIterationFlag, serialProcessingFlag);
    }

    //if was added as a detector or validator
    if (addedDetector || addedConstraint) {
      //add all descriptor info to trackers
      if (tracker != null) {
        tracker.addProcessingDescriptor(term, this);
      }

      //fire event on ui event queue to update description
      Vision.availableSearchTypes.updateGuiEntry(this);
      return true;
    } else {
      log.info(String.format("[addConstraint] %s does not map to a valid ValidationProcessor or Detector.", term.toString()));
      return false;
    }

  }

  private boolean addDetectorConstraint(final Term term, final boolean incrementalFlag,
                                        final boolean singleIterationFlag, final boolean serialProcessingFlag) {
    if (detector != null) {
      log.error(String.format("[addConstraint] Detector type has already been selected for SearchManager: %d. Can not reassign Detector.", typeId));
      return false;
    }

    List<Detector> newDetectors = Vision.availableDetectors.getInstances(this, term);
    if (newDetectors.isEmpty()) {
      log.error(String.format("[addDetectorConstraint] No Detectors available for %s.", term.toString()));
      return false;
    }

    //if more than one detector option, just choosing first one for now
    detector = newDetectors.get(0);
    for (Detector d : newDetectors) {
      availableDetectors.put(d.getType(), d);
    }

    //register detector to be notified by its own dependencies
    List<ImageProcessorType> detectorDependencyType = detector.getDependencies();
    for (ImageProcessorType dependencyType : detectorDependencyType) {
      ImageProcessor dependency = detector.getDependency(dependencyType);
      if (dependency == null) {
        log.warn("Dependency not available: {}", dependencyType);
      } else {
        dependency.registerForNotification(detector, typeId);
      }
    }

    //register detector to be notified by existing saliencyOperators
    //if detector has master saliency dependency, register to be notified by
    //master instead of individual saliency processors
    ImageProcessor masterSaliency = detector.getDependency(ImageProcessorType.MASTERSALIENCY);
    if (masterSaliency != null) {
      masterSaliency.registerForNotification(detector, typeId);
      for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
        ImageProcessor existingSaliencyConstraint = e.nextElement();
        existingSaliencyConstraint.registerForNotification(masterSaliency, typeId);
      }
    } else if (saliencyOperators.size() > 0) {
      log.warn(String.format("[addDetectorConstraint] Detector has SaliencyOperators but no MasterSaliency dependency."
              + "Add dependency to Detector config if needed. SearchManager: %d.", typeId));
    }

    // register detector to notify existing validators
    if (!validationConstraints.isEmpty()) {
      if (parallelValidators) {
        for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
          ImageProcessor existingValidationConstraint = e.nextElement();
          detector.registerForNotification(existingValidationConstraint, typeId);
        }
      } else {
        ImageProcessor firstValidationConstraint = validationConstraints.firstElement();
        detector.registerForNotification(firstValidationConstraint, typeId);
      }
    }

    //set flags for all new detectors
    for (Detector d : newDetectors) {
      //set single iteration flag
      d.setSingleIteration(singleIterationFlag);

      //set incremental processing
      d.setIncrementalProcessing(incrementalFlag);

      //set serial processing
      d.setSerialProcessing(serialProcessingFlag);
    }

    //add term to SearchManager description
    if (!term.equals(defaultDetectorPredicate)) {
      description.add(term);
    }
    detector.addProcessingDescriptor(term, this);    //add to detector's java and native side

    //get/add trackers from all detector options
    if (standAloneSearch) {
      setTrackers(newDetectors);
    }

    //start detector
    if (shouldBeRunning()) {
      detector.start(this);
    }

    return true;
  }

  /**
   * Add available trackers from all detector options (avoiding duplicate
   * types), set current Tracker, and make appropriate registrations between
   * Tracker, Detector, and Validators.
   *
   * @param detectors list of detector options
   */
  private void setTrackers(List<Detector> detectors) {
    for (Detector d : detectors) {
      // add all tracker options for each detector option
      for (TrackerType usableTrackerType : d.getUsableTrackers()) {
        // avoiding duplicate types
        if (!availableTrackers.containsKey(usableTrackerType)) {
          Tracker newTracker = Vision.availableTrackers.getInstance(this, usableTrackerType);
          availableTrackers.put(newTracker.getType(), newTracker);
        }
      }

      // set selected detector's tracker
      if (d.equals(detector)) {
        if (!detector.getUsableTrackers().isEmpty() && availableTrackers.containsKey(detector.getUsableTrackers().get(0))) {
          tracker = availableTrackers.get(detector.getUsableTrackers().get(0));

          //add all descriptors to tracker
          for (Term descriptor : description) {
            tracker.addProcessingDescriptor(descriptor, this);
          }

          //if there are any validation processors, register to notify them,
          //otherwise register to notify tracker
          if (validationConstraints.isEmpty()) {
            detector.registerForNotification(tracker, typeId);
          } else {
            //co-register tracker with validation processors
            if (parallelValidators) {
              for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
                ImageProcessor existingValidationConstraint = e.nextElement();
                //tracker.registerForNotification(existingValidationConstraint);    //TODO: add back in?
                existingValidationConstraint.registerForNotification(tracker, typeId);
              }
            } else {
              //ImageProcessor firstValidationConstraint = validationConstraints.firstElement();
              //tracker.registerForNotification(firstValidationConstraint);   //TODO: add back in?
              ImageProcessor lastValidationConstraint = validationConstraints.lastElement();
              lastValidationConstraint.registerForNotification(tracker, typeId);
            }
          }
        } else {
          log.error(String.format("[addDetectorConstraint] No Trackers available for %s detector.", detector.getType().toString()));
        }
      }
    }

    //start tracker
    if (shouldBeRunning() && tracker != null) {
      tracker.start(this);
    }
  }

  private boolean addValidationConstraint(final Term term, final boolean incrementalFlag,
                                          final boolean singleIterationFlag, final boolean serialProcessingFlag) {
    //get ImageProcessorType capable of processing requested term
    ImageProcessorType processorType = Vision.availableValidationProcessors.getCapableProcessorType(term);

    //check if ImageProcessorType already exists
    //if it does, just add term descriptor
    for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
      ImageProcessor currValidationConstraint = e.nextElement();
      if (currValidationConstraint.isType(processorType)) {
        //add additional processing descriptor to existing image processor constraint
        if (currValidationConstraint.addProcessingDescriptor(term, this)) {
          description.add(term);
          return true;
        } else {
          return false;
        }
      }
    }

    //constraint does not yet exist, create it
    ImageProcessor newValidationConstraint = Vision.availableValidationProcessors.getInstance(this, term);
    newValidationConstraint.addProcessingDescriptor(term, this);
    description.add(term);

    //if this is first validation constraint to be added, remove the notification
    //link between detector and tracker. whenever there is a validator, it will
    //sit "in-between" the detector and tracker (added later in this method).
    if (detector != null && tracker != null && validationConstraints.isEmpty()) {
      detector.unregisterForNotification(tracker, typeId);
    }

    if (parallelValidators) {
      //register to be notified by detector
      if (detector != null) {
        detector.registerForNotification(newValidationConstraint, typeId);
      }

      //co-register to be notified by tracker
      if (tracker != null) {
        //tracker.registerForNotification(newValidationConstraint, typeId); //TODO: add this back in!
        newValidationConstraint.registerForNotification(tracker, typeId);
      }
    } else { //serial validators
      if (validationConstraints.isEmpty()) {
        //first validation constraint needs to be notifted by detector
        if (detector != null) {
          detector.registerForNotification(newValidationConstraint, typeId);
        }
        //last validation constraint needs to co-register with tracker
        if (tracker != null) {
          newValidationConstraint.registerForNotification(tracker, typeId);
          //tracker.registerForNotification(newValidationConstraint, typeId);  //TODO: add this back in!
        }
      } else {
        ImageProcessor lastValidationConstraint = validationConstraints.lastElement();
        lastValidationConstraint.registerForNotification(newValidationConstraint, typeId);

        if (tracker != null) {
          lastValidationConstraint.unregisterForNotification(tracker, typeId);
          newValidationConstraint.registerForNotification(tracker, typeId);
        }
      }
    }

    //add to local list
    validationConstraints.addElement(newValidationConstraint);

    //set single iteration flag
    newValidationConstraint.setSingleIteration(singleIterationFlag);

    //set incremental processing
    newValidationConstraint.setIncrementalProcessing(incrementalFlag);

    //set serial processing
    newValidationConstraint.setSerialProcessing(serialProcessingFlag);

    //start new constraint, if requested
    if (shouldBeRunning()) {
      newValidationConstraint.start(this);
    }

    return true;
  }

  private boolean addSaliencyConstraint(final Term term, final boolean incrementalFlag,
                                        final boolean singleIterationFlag, final boolean serialProcessingFlag) {
    //get ImageProcessorType capable of processing requested term
    ImageProcessorType processorType = Vision.availableSaliencyProcessors.getCapableProcessorType(term);

    //check if ImageProcessorType already exists
    //if it does, just add term descriptor
    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor currSaliencyConstraint = e.nextElement();
      if (currSaliencyConstraint.isType(processorType)) {
        //add additional processing descriptor to existing image processor constraint
        if (currSaliencyConstraint.addProcessingDescriptor(term, this)) {
          return true;
        } else {
          return false;
        }
      }
    }

    //constraint does not yet exist, create it
    ImageProcessor newSaliencyConstraint = Vision.availableSaliencyProcessors.getInstance(this, term);
    newSaliencyConstraint.addProcessingDescriptor(term, this);

    //co-register with detector
    if (detector != null) {
      //if detector has master_saliency dependency, register with that instead of detector itself
      ImageProcessor masterSaliency = detector.getDependency(ImageProcessorType.MASTERSALIENCY);
      if (masterSaliency != null) {
        newSaliencyConstraint.registerForNotification(masterSaliency, typeId);
      } else {
        newSaliencyConstraint.registerForNotification(detector, typeId);
      }
    }

    //add to local list
    saliencyOperators.addElement(newSaliencyConstraint);

    //set single iteration flag
    newSaliencyConstraint.setSingleIteration(singleIterationFlag);

    //set incremental processing
    newSaliencyConstraint.setIncrementalProcessing(incrementalFlag);

    //set serial processing
    newSaliencyConstraint.setSerialProcessing(serialProcessingFlag);

    //start new constraint, if requested
    if (shouldBeRunning()) {
      newSaliencyConstraint.start(this);
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

    boolean removedConstraint = false;
    if (removeImageProcessorConstraint(term)) {
      removedConstraint = true;
    } else if (removeDetectorConstraint(term)) {
      removedConstraint = true;
    } else {
      log.error(String.format("[removeConstraint] %s could not be removed or is not part of this SearchManager.", term.toString()));
    }

    //fire event on ui event queue to update description
    if (removedConstraint) {
      //keep all descriptor info in trackers
      if (tracker != null) {
        tracker.removeProcessingDescriptor(term, this);
      }
      Vision.availableSearchTypes.updateGuiEntry(this);
    }

    return removedConstraint;
  }

  private void finishConstructingPipeline() {
    if (detector == null) {
      setDefaultDetector();
    }
  }

  @Override
  public void registerForNotification(VisionProcess processor) {
    finishConstructingPipeline();

    if (tracker == null) {
      log.error("[registerForNotification] trying to register with null Tracker.");
    }
    tracker.registerForNotification(processor, typeId);
  }

  @Override
  public void unregisterForNotification(VisionProcess processor) {
    finishConstructingPipeline();

    if (tracker == null) {
      log.error("[unregisterForNotification] trying to unregister with null Tracker.");
    }
    tracker.unregisterForNotification(processor, typeId);
  }

  /**
   * @param term
   * @return - if was successfully removed. Note: if wasn't contained, returns
   * false.
   */
  private boolean removeImageProcessorConstraint(final Term term) {
    isValid();

    //EAK: TODO: this needs to be reimplemented. all the new saliency, validator notification
    //business has really complicated things, and this needs to be thoroughly thought through
    //when there's actually time to think it through.
    log.warn("[removeImageProcessorConstraint] method not currently available.");

    //remove processing descriptor from existing image processor constraint
        /*
     for (Enumeration<ImageProcessor> e = constraints.elements(); e.hasMoreElements();) {
     ImageProcessor currConstraint = e.nextElement();
     if (currConstraint.removeProcessingDescriptor(term, this)) {
     //remove term from SearchManager description
     description.remove(term);

     //if no more processing descriptors, turn off, and remove image processor
     if (currConstraint.getProcessingDescriptorsSize() <= 0) {
     currConstraint.stop(this);

     //un-register with detector
     if (detector != null) {
     detector.unregisterForNotification(currConstraint);
     currConstraint.unregisterForNotification(detector);
     }

     //un-register with tracker
     if (tracker != null) {
     tracker.unregisterForNotification(currConstraint);
     currConstraint.unregisterForNotification(tracker);
     }

     //un-register with other image processors
     constraints.removeElement(currConstraint);  //remove from list before iterating through rest of constraints
     for (Enumeration<ImageProcessor> e2 = constraints.elements(); e2.hasMoreElements();) {
     ImageProcessor otherConstraint = e2.nextElement();
     otherConstraint.unregisterForNotification(currConstraint);
     currConstraint.unregisterForNotification(otherConstraint);
     }

     //release "ownership" back to factory
     Vision.availableImageProcessors.release(this, currConstraint);
     }

     return true;
     }
     }
     */
    return false;
  }

  private boolean removeDetectorConstraint(final Term term) {
    isValid();

    log.warn("[removeDetectorConstraint] method not currently available.");
    return false;
  }

  /**
   * Set the incremental processing flag for the Detector.
   *
   * @param flag
   */
  @Override
  public void setDetectorIncrementalProcessing(final boolean flag) {
    isValid();

    for (Detector d : availableDetectors.values()) {
      d.setIncrementalProcessing(flag);
    }
  }

  /**
   * Get incremental processing flag for selected Detector. Returns
   * "incrementalDetectorFlag" if no detector exists (as default preference to
   * use incremental processing).
   *
   * @return
   */
  @Override
  public synchronized boolean getDetectorIncrementalProcessing() {
    isValid();

    if (availableDetectors.isEmpty()) {
      return incrementalDetector;
    }

    //get flag of first detector
    final boolean flag = availableDetectors.values().iterator().next().getIncrementalProcessing();

    //verify that all detector are consistent
    for (Detector d : availableDetectors.values()) {
      if (d.getIncrementalProcessing() != flag) {
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

    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor currConstraint = e.nextElement();
      currConstraint.setIncrementalProcessing(flag);
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

    if (saliencyOperators.isEmpty()) {
      return incrementalImgProc;
    }

    //get flag of first constraint
    Enumeration<ImageProcessor> e = saliencyOperators.elements();
    ImageProcessor currConstraint = e.nextElement();
    final boolean flag = currConstraint.getIncrementalProcessing();

    //verify that all saliencyOperators are consistent
    while (e.hasMoreElements()) {
      currConstraint = e.nextElement();
      if (currConstraint.getIncrementalProcessing() != flag) {
        log.error("[getConstraintsIncrementalProcessing]: Incremental Processing flags are not consistent.");
      }
    }

    return flag;
  }

  @Override
  public synchronized void setSingleIteration(final boolean flag) {
    isValid();

    //set for saliency processors
    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor currConstraint = e.nextElement();
      currConstraint.setSingleIteration(flag);
    }

    //set for detectors
    for (Detector d : availableDetectors.values()) {
      d.setSingleIteration(flag);
    }
  }

  @Override
  public synchronized boolean getSingleIteration() {
    isValid();

    if (!saliencyOperators.isEmpty()) {
      //get flag of first constraint
      Enumeration<ImageProcessor> e = saliencyOperators.elements();
      ImageProcessor currConstraint = e.nextElement();
      final boolean flag = currConstraint.getSingleIteration();

      //verify that all saliencyOperators are consistent
      while (e.hasMoreElements()) {
        currConstraint = e.nextElement();
        if (currConstraint.getSingleIteration() != flag) {
          log.error("[getSingleIteration]: Single Iteration flags are not consistent.");
        }
      }

      //verify that detector is consistent
      if (detector != null) {
        if (detector.getSingleIteration() != flag) {
          log.error("[getSingleIteration]: Single Iteration flags are not consistent with Detector.");
        }
      }

      return flag;
    } else if (detector != null) {
      return detector.getSingleIteration();
    } else {
      //no image processors or detectors yet
      return singleIteration;
    }
  }

  @Override
  public synchronized void setSerialProcessing(final boolean flag) {
    isValid();

    //set for saliency processors
    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor currConstraint = e.nextElement();
      currConstraint.setSerialProcessing(flag);
    }

    //set for detectors
    for (Detector d : availableDetectors.values()) {
      d.setSerialProcessing(flag);
    }
  }

  @Override
  public synchronized boolean getSerialProcessing() {
    isValid();

    if (!saliencyOperators.isEmpty()) {
      //get flag of first constraint
      Enumeration<ImageProcessor> e = saliencyOperators.elements();
      ImageProcessor currConstraint = e.nextElement();
      final boolean flag = currConstraint.getSerialProcessing();

      //verify that all saliencyOperators are consistent
      while (e.hasMoreElements()) {
        currConstraint = e.nextElement();
        if (currConstraint.getSerialProcessing() != flag) {
          log.error("[getSerialProcessing]: Serial Processing flags are not consistent.");
        }
      }

      //verify that detector is consistent
      if (detector != null) {
        if (detector.getSerialProcessing() != flag) {
          log.error("[getSerialProcessing]: Serial Processing flags are not consistent with Detector.");
        }
      }

      return flag;
    } else if (detector != null) {
      return detector.getSerialProcessing();
    } else {
      //no image processors or detectors yet
      return serialProcessing;
    }
  }

  /**
   * Get saliency operators as a ListModel to allow the UI to listen when
   * changes to the operators are made.
   *
   * @return
   */
  public synchronized ListModel getSaliencyOperators() {
    isValid();

    return saliencyOperators;
  }

  /**
   * Get validator constraints as a ListModel to allow the UI to listen when
   * changes to the constraints are made.
   *
   * @return
   */
  public synchronized ListModel getValidatorConstraints() {
    isValid();

    return validationConstraints;
  }

  /**
   * Get set of DetectorTypes available for this search.
   *
   */
  public synchronized Set<DetectorType> getDetectorOptions() {
    isValid();

    return availableDetectors.keySet();
  }

  /**
   * Get set of TrackerTypes available for this search.
   */
  public synchronized Set<TrackerType> getTrackerOptions() {
    isValid();

    return availableTrackers.keySet();
  }

  /**
   * Get Detector option, if it exists.
   *
   * @param detectorType
   * @return Detector, or null
   */
  public synchronized Detector getDetector(DetectorType detectorType) {
    isValid();

    return availableDetectors.get(detectorType);
  }

  /**
   * Get Tracker, if it exists.
   *
   * @param trackerType
   * @return Tracker, or null
   */
  public synchronized Tracker getTracker(TrackerType trackerType) {
    isValid();

    return availableTrackers.get(trackerType);
  }

  /**
   * Find which available Detector is currently selected for use.
   *
   * @return Detector
   */
  public synchronized Detector getSelectedDetector() {
    isValid();

    return detector;
  }

  /**
   * Find which available Tracker is currently selected for use.
   *
   * @return
   */
  public synchronized Tracker getSelectedTracker() {
    isValid();

    return tracker;
  }

  @Override
  public VisionProcess getFirstVisionProcessor() {
    if (detector == null) {
      setDefaultDetector();
    }
    return detector;
  }

  @Override
  public VisionProcess getLastVisionProcessor() {
    if (tracker != null) {
      return tracker;
    } else if (!validationConstraints.isEmpty()) {
      return validationConstraints.lastElement();
    } else {
      return detector;
    }
  }

  @Override
  public Set<VisionProcess> getAllVisionProcessors() {
    Set<VisionProcess> results = new HashSet();
    // Guaranteed to return the detector
    VisionProcess first = getFirstVisionProcessor();
    results.add(first);
    addAllDependencies(first, results);

    if (!validationConstraints.isEmpty()) {
      for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
        ImageProcessor currValidationConstraint = e.nextElement();
        results.add(currValidationConstraint);
        addAllDependencies(currValidationConstraint, results);
      }
    }
    if (!saliencyOperators.isEmpty()) {
      for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
        ImageProcessor operator = e.nextElement();
        results.add(operator);
        addAllDependencies(operator, results);
      }
    }
    if (tracker != null) {
      results.add(tracker);
      addAllDependencies(tracker, results);
    }

    return results;
  }

  /**
   * Will populate a set with the dependencies of the passed-in visionprocess
   *
   * @param current
   * @param results
   */
  private void addAllDependencies(VisionProcess current, Set<VisionProcess> results) {
    if (current == null)
      return;

    List<ImageProcessorType> detectorDependencyType = current.getDependencies();
    for (ImageProcessorType dependencyType : detectorDependencyType) {
      ImageProcessor dependency = detector.getDependency(dependencyType);
      results.add(dependency);
      addAllDependencies(dependency, results);
    }
  }

  private void setDefaultDetector() {
    // make sure detector has same variable name as validators
    if (!validationConstraints.isEmpty()) {
      // making assumption that validator's first arg of first descriptor is variable we want
      Symbol var = validationConstraints.firstElement().getProcessingDescriptors(this).get(0).get(0);
      defaultDetectorPredicate = new Predicate(defaultDetectorPredicate.getName(), var);
    }

    // try to get default detector
    if (!addDetectorConstraint(defaultDetectorPredicate, incrementalDetector, singleIteration, serialProcessing)) {
      log.warn(String.format("[setDefaultDetector] No Detector selected, and the default \"%s\" Detector was unavailable.", defaultDetectorPredicate.toString()));
      return;
    } else {
      log.info(String.format("[setDefaultDetector] No Detector selected, so default \"%s\" Detector was selected.", defaultDetectorPredicate.toString()));

      //fire event on ui event queue to update entry
      Vision.availableSearchTypes.updateGuiEntry(this);
    }
  }

  /**
   * Switch the Detector to be used. If successful, also switches to a suitable
   * Tracker.
   *
   * @param detectorType
   * @return bool - if switch was successful
   */
  public synchronized boolean setDetector(DetectorType detectorType) {
    isValid();

    // don't allow changes while running, or in the pipeline to start running
    if (isRunning()) {
      return false;
    }

    if (detector != null && detector.isType(detectorType)) {
      // already using specified detector
      return true;
    }

    // see if the requested detectorType is an option
    Detector newDetector = availableDetectors.get(detectorType);

    if (newDetector == null) {
      log.warn(String.format("DetectorType %s not available for SearchManager %d.", detectorType.toString(), typeId));
    }

    // check/get tracker
    Tracker newTracker = null;
    if (newDetector != null && standAloneSearch) {
      //check if currently selected tracker is valid with detector
      if (newDetector.canUseTracker(tracker.getType())) {
        newTracker = tracker;
      } else {
        //find a usable tracker for dType
        for (Tracker currTrackerOption : availableTrackers.values()) {
          if (newDetector.canUseTracker(currTrackerOption.getType())) {
            newTracker = currTrackerOption;                                //set tracker
            break;
          }
        }
      }
      if (newTracker == null) {
        log.warn(String.format("SearchManager %d does not have an available tracker for DetectorType %s.", typeId, detectorType.toString()));
      }
    }

    // final detector and tracker check
    if (newDetector == null || (newTracker == null && standAloneSearch)) {
      return false;
    }

    log.warn("[setDetector] notification registration has not been updated!");
    detector = newDetector;
    tracker = newTracker;
    return true;
  }

  /**
   * Switch the Tracker to be used. If successful, also switches to a suitable
   * Detector.
   *
   * @param trackerType
   * @return bool - if switch was successful
   */
  public synchronized boolean setTracker(TrackerType trackerType) {
    isValid();

    // don't allow changes while running, or in the pipeline to start running, or
    // if search manager isn't using a tracker
    if (isRunning() || !standAloneSearch) {
      return false;
    }

    if (tracker != null && tracker.isType(trackerType)) {
      //already using specified tracker
      return true;
    }

    // see if the requested trackerType is an option
    Tracker newTracker = availableTrackers.get(trackerType);

    Detector newDetector = null;
    if (newTracker != null) {
      //check if currently selected detector is valid with tracker
      if (detector.canUseTracker(trackerType)) {
        newDetector = detector;
      } else {
        //find a usable detector for tType
        for (Detector currDetectorOption : availableDetectors.values()) {
          if (currDetectorOption.canUseTracker(trackerType)) {
            newDetector = currDetectorOption;
            break;
          }
        }
      }
    }

    if (newTracker == null) {
      log.warn(String.format("TrackerType %s not available for SearchManager %d.", trackerType.toString(), typeId));
    } else if (newDetector == null) {
      log.warn(String.format("SearchManager %d does not have an available detector for TrackerType %s.", typeId, trackerType.toString()));
    }

    log.warn("[setTracker] notification registration has not been updated!");
    if (newTracker == null || newDetector == null) {
      return false;
    }

    detector = newDetector;
    tracker = newTracker;
    return true;
  }

  /**
   * Does the search meet the expectation of its set analysis policy. This is
   * for vision introspection.
   *
   * @return
   */
  @Override
  public synchronized Collection<TaskAnalysisPolicy.Result> meetsExpectations() {
    isValid();
    return analysisPolicy.analyse(performanceInfo);
  }

  /**
   * Get all available detector types, leaving out the current detector type.
   *
   * @return set of DetectorTypes
   */
  public synchronized Set<DetectorType> getAlternativeDetectorTypes() {
    isValid();

    Set<DetectorType> detectorTypes = new HashSet<>(availableDetectors.keySet());
    detectorTypes.remove(detector.getType());

    return detectorTypes;
  }

  /**
   * Get all available tracker types, leaving out the current tracker type.
   *
   * @return set of TrackerTypes
   */
  public synchronized Set<TrackerType> getAlternativeTrackerTypes() {
    isValid();

    Set<TrackerType> trackerTypes = new HashSet<>(availableTrackers.keySet());
    trackerTypes.remove(tracker.getType());

    return trackerTypes;
  }

  /**
   * Add a new Detector and Tracker option for this memory object type.
   *
   * @param detectorType - new Detector type
   * @param trackerType - new Tracker type
   */
//    synchronized void addDetectorTrackerPair(DetectorType detectorType, TrackerType trackerType) {
//        isValid();
//
//        if (!containsDetectorType(detectorType)) {
//            Detector newDetector = Vision.availableDetectors.getInstances(this, detectorType);
//            availableDetectors.put(newDetector.getType(), newDetector);
//        }
//
//        if (!containsTrackerType(trackerType)) {
//            Tracker newTracker = Vision.availableTrackers.getInstance(this, trackerType);
//            availableTrackers.add(newTracker);
//        }
//
//        //set task analysis policy
//        //TODO: HACKY - how should this actually be done??
//        if (availableDetectors.size() == 2 && containsDetectorType(DetectorType.BLOB) && containsDetectorType(DetectorType.SIFT3D)) {
//            analysisPolicy = new TaskAnalysisPolicy_BlobBlort();
//        } else {
//            analysisPolicy = new TaskAnalysisPolicyDefault();
//        }
//    }

  /**
   * Can this search use the specified detector type.
   *
   * @param detectorType
   * @return
   */
  private boolean containsDetectorType(DetectorType detectorType) {
    return (getDetector(detectorType) != null);
  }

  /**
   * Can this search use specified tracker type.
   *
   * @param trackerType
   * @return
   */
  private boolean containsTrackerType(TrackerType trackerType) {
    return (getTracker(trackerType) != null);
  }

  /**
   * Is this search currently running.
   *
   * @return
   */
  @Override
  public synchronized boolean isRunning() {
    isValid();

    //check detector and tracker
    if (detector == null || !detector.isStartCaller(this)) {
      return false;
    }
    if (standAloneSearch && (tracker == null || !tracker.isStartCaller(this))) {
      return false;
    }

    //check validation constraints
    for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
      ImageProcessor constraint = e.nextElement();
      if (!constraint.isStartCaller(this)) {
        return false;
      }
    }

    //check saliency operators
    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor operator = e.nextElement();
      if (!operator.isStartCaller(this)) {
        return false;
      }
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

    if (detector == null) {
      setDefaultDetector();
    }

    if (standAloneSearch && tracker == null) {
      log.warn("Can not start. No Tracker selected.");
      return;
    }

    if (standAloneSearch) {
      tracker.start(this);
    }
    for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
      ImageProcessor existingConstraint = e.nextElement();
      existingConstraint.start(this);
    }
    if (standAloneSearch) {
      detector.registerForCaptureNotification();
    }
    detector.start(this);
    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor existingConstraint = e.nextElement();
      existingConstraint.start(this);
    }
  }

  /**
   * Stop constraints, detector, and tracker if they are running, and return
   * immediately without waiting for processors to stop.
   *
   * @param wait
   */
  @Override
  final protected synchronized void stop(boolean wait) {
    isValid();

    log.debug(String.format("[stop] wait: %b.", wait));

    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor existingConstraint = e.nextElement();
      log.trace(String.format("[stop] %d stopping saliency constraint: %s.", typeId, existingConstraint.toString()));
      existingConstraint.stop(this, wait);
    }
    for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
      ImageProcessor existingConstraint = e.nextElement();
      log.trace(String.format("[stop] %d stopping validation constraint: %s.", typeId, existingConstraint.toString()));
      existingConstraint.stop(this, wait);
    }

    if (detector != null) {

      if (standAloneSearch) {
        detector.unregisterForCaptureNotification();
      }
      detector.stop(this, wait);
    } else {
      log.warn("[stop] Detector is null. Could not stop.");
    }

    if (standAloneSearch && tracker != null) {
      log.trace(String.format("[stop] %d stopping tracker.", typeId));
      tracker.stop(this, wait);
    } else {
      log.warn("[stop] Tracker is null. Could not stop.");
    }

    log.trace(String.format("[stop] %d stopped.", typeId));
  }

  @Override
  public synchronized boolean hasIterationCompleted() {
    isValid();

    if (tracker != null) {
      return tracker.hasIterationCompleted(typeId);
    }
    return false;
  }

  @Override
  public synchronized void setTimeOfLastClientUse(long time) {
    isValid();

    performanceInfo.setTimeOfLastClientUse(time);

    if (tracker != null) {
      tracker.getPerformanceInfo().setTimeOfLastClientUse(time);
    }
    if (detector != null) {
      detector.getPerformanceInfo().setTimeOfLastClientUse(time);
    }

    //TODO: also set saliency and constraints?
  }

  @Override
  synchronized void terminate() {
    isValid();

    //should already be stopped, but just in case...
    //stop();
    validFlag = false;

    //"release" constraints
    for (Enumeration<ImageProcessor> e = saliencyOperators.elements(); e.hasMoreElements(); ) {
      ImageProcessor existingConstraint = e.nextElement();
      Vision.availableSaliencyProcessors.release(this, existingConstraint);
    }
    saliencyOperators.clear();
    for (Enumeration<ImageProcessor> e = validationConstraints.elements(); e.hasMoreElements(); ) {
      ImageProcessor existingConstraint = e.nextElement();
      Vision.availableValidationProcessors.release(this, existingConstraint);
    }
    validationConstraints.clear();

    //"release" Trackers
    for (Tracker currTrackerOption : availableTrackers.values()) {
      Vision.availableTrackers.release(this, currTrackerOption);
    }
    availableTrackers.clear();

    //"release" Detectors
    for (Detector currDetectorOption : availableDetectors.values()) {
      Vision.availableDetectors.release(this, currDetectorOption);
    }
    availableDetectors.clear();
  }
}
