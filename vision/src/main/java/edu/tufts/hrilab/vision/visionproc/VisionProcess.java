/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.visionproc;

import edu.tufts.hrilab.vision.reflection.TaskPerformanceInformation;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.visionproc.swig.NativeVisionProcess;
import edu.tufts.hrilab.vision.visionproc.swig.VectorLong;
import edu.tufts.hrilab.util.IdGenerator;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.stm.SearchManager;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeoutException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Base class for Detector, Tracker, and ImageProcessor classes.
 *
 * ============ IMPORTANT ==================== (1) terminate() must be called
 * when a VisionProcess is no longer used. (2) Every public class method must be
 * synchronized and call isValid() at the beginning of the method.
 * ===========================================
 *
 * @author Evan Krause evan.krause@tufts.edu
 * @param <T> SWIG wrapped enum class (ImageProcessorType, DetectorType,
 * @param <Detail> VisionProcessDetail
 * @param <NVP> NativeVisionProcess
 */
public abstract class VisionProcess<T extends Enum<T>, Detail extends VisionProcessDetail<T>, NVP extends NativeVisionProcess> {

  /**
   * Log4j logger.
   */
  protected Logger log = LoggerFactory.getLogger(getClass());
  /**
   * To generate unique IDs for each VisionProcessor.
   */
  static final protected IdGenerator id_generator = new IdGenerator();
  /**
   * If this VisionProcessor is still valid.
   */
  private boolean validFlag = true;
  /**
   * Unique processor ID.
   */
  protected final long processId;
  /**
   * Basic information about how to instantiate a particular kind of Vision
   * Processor.
   */
  protected final Detail detail;
  /**
   * Processor's image processing dependencies (e.g. sift, etc).
   */
  protected HashMap<ImageProcessorType, ImageProcessor> dependencies = new HashMap();
  /**
   * Native processor (used via SWIG) that does the actual vision processing
   * work.
   */
  protected final NVP processor;
  /**
   * Handles thread that executes native processor loop.
   */
  private final ExecutorService executor = Executors.newSingleThreadExecutor();
  /**
   * If this processor is currently running.
   */
  protected volatile boolean runFlag = false;
  /**
   * Run future from starting the vision processor.
   */
  private Future runFuture;
  /**
   * Run future from stopping the vision processor.
   */
  private Future stopFuture;
  /**
   * Keeps track of all objects that have called start. A single processor can
   * be servicing multiple searches simultaneously.
   */
  private final HashSet<Object> startCallers = new HashSet();
  /**
   * Predicate description that processor is trying to process. Hashed by caller
   * object.
   */
  protected HashMap<Object, List<Term>> description = new HashMap();
  /**
   * Runtime stats and metrics.
   */
  protected TaskPerformanceInformation performanceInfo = null;
  /**
   * If true, only complete one processing iteration and then stop.
   */
  protected boolean singleIteration = false;

  public VisionProcess(final Detail typeDetail, final NVP nativeProcessor) {
    detail = typeDetail;
    processor = nativeProcessor;
    processId = nativeProcessor.getProcessorId();
    performanceInfo = new TaskPerformanceInformation(detail.toString());

    //get instantiated ImageProcessor dependencies
    for (ImageProcessorType dependencyType : detail.getDependencies()) {
      //need to look in all three imgProc factories for dependencies
      //TODO: make this cleaner ?
      ImageProcessor dependency;
      if (Vision.availableSaliencyProcessors.hasCapableProcessorType(dependencyType)) {
        dependency = Vision.availableSaliencyProcessors.getInstance(this, dependencyType);
      } else if (Vision.availableValidationProcessors.hasCapableProcessorType(dependencyType)) {
        dependency = Vision.availableValidationProcessors.getInstance(this, dependencyType);
      } else if (Vision.availableImageProcessors.hasCapableProcessorType(dependencyType)) {
        dependency = Vision.availableImageProcessors.getInstance(this, dependencyType);
      } else {
        log.error("[constructor] at least one dependency could not be found!");
        continue;
      }

      dependencies.put(dependencyType, dependency);

      // register VisionProcess to be notified by its own dependencies
      // EAK: using "this" in constr is dangerous but i think it should be ok here
      // NOTE: using -1 here for typeId because all dependencies of this vision processor
      // are the same regardless of the typeId using this vision processor
      dependency.registerForNotification(this, -1L);
    }
  }

  @Override
  public synchronized String toString() {
    return "VisionProcess: " + detail.toString() + ". ID: " + processId;
  }

  /**
   * Get ID of vision processor.
   *
   * @return processor Id
   */
  final public long getId() {
    isValid();
    return processId;
  }

  /**
   * Check if VisionProcess is of certain enum type.
   *
   * @param processType - enum Type.
   * @return
   */
  public synchronized boolean isType(T processType) {
    isValid();

    return detail.getType().equals(processType);
  }

  /**
   * Get VisionProcess enum Type
   *
   * @return
   */
  public synchronized T getType() {
    isValid();

    return detail.getType();
  }

  /**
   * Get underlying native VisionProcess. Mainly used to register processors
   * with each other.
   *
   * @return
   */
  public synchronized NVP getNativeProcessor() {
    isValid();

    return processor;
  }

  /**
   * Get performance info for introspection tasks.
   *
   * @return
   */
  public synchronized TaskPerformanceInformation getPerformanceInfo() {
    isValid();

    return performanceInfo;
  }

  /**
   * Get ImageProcessor dependencies.
   *
   * @return
   */
  public synchronized List<ImageProcessorType> getDependencies() {
    isValid();

    return detail.getDependencies();
  }

  /**
   * Get specific instantiated ImageProcessor dependency. Null if dependency
   * doesn't exist.
   *
   * @param type
   * @return
   */
  public synchronized ImageProcessor getDependency(ImageProcessorType type) {
    isValid();

    return dependencies.get(type);
  }

  /**
   * Configure native VisionProcess.
   *
   * @param config filename
   */
  public synchronized void loadConfig(final String config) {
    isValid();

    processor.loadConfig(config);
  }

  /**
   * Register a VisionProcess to be interrupted or notified every time a
   * "perform" iteration has been completed. The notification is sent and
   * received on that native side.
   *
   * @param processor to be notified.
   */
  public synchronized void registerForNotification(VisionProcess processor, Long typeId) {
    isValid();

    this.processor.registerForNotification(processor.getNativeProcessor(), typeId);
  }

  /**
   * Unregister a previously registered VisionProcess to no longer be notified.
   *
   * @param processor to be notified.
   */
  public synchronized void unregisterForNotification(VisionProcess processor, Long typeId) {
    isValid();

    this.processor.unregisterForNotification(processor.getNativeProcessor(), typeId);
  }

  public synchronized List<Long> getRegisteredProcessorIds(Long typeId) {
    List<Long> neighboring_ids = new ArrayList();
    VectorLong proc_ids = this.processor.getRegisteredProcessorIds(typeId);
    for (int i = 0; i < proc_ids.size(); i++) {
        neighboring_ids.add(proc_ids.get(i));
    }
    return neighboring_ids;
  }

  /**
   * Register to be notified every time a "capture" iteration has been
   * completed. The notification is sent and received on that native side. This
   * method is needed because the capture processor does not derive from the
   * VisionProccess base class.
   */
  public synchronized void registerForCaptureNotification() {
    isValid();

    this.processor.registerForCaptureNotification();
  }

  /**
   * Un-register from the "capture" notifications. This method is needed because
   * the capture processor does not derive from the VisionProccess base class.
   */
  public synchronized void unregisterForCaptureNotification() {
    isValid();

    this.processor.unregisterForCaptureNotification();
  }

  /**
   * Add descriptor to VisionProcess (e.g., add "red" to color processor)
   *
   * @param descriptor - Predicate to add
   * @param caller - the object adding the descriptor (almost always a
   * SearchManager)
   * @return - if add was successful
   */
  final public synchronized boolean addProcessingDescriptor(final Term descriptor, final SearchManager caller) {
    isValid();

    //add descriptor locally -- don't yet pass it to native side
    List<Term> descriptionById = description.get(caller);
    if (descriptionById == null) {
      descriptionById = new ArrayList();
      description.put(caller, descriptionById);
    }
    descriptionById.add(descriptor);

    //pass desciptor to native side if caller has called start
    if (startCallers.contains(caller)) {
      if (!addProcessingDescriptorToNative(descriptor, caller.getTypeId())) {
        log.error(String.format("[addProcessingDescriptor] %s descriptor could not be added to native processor.", descriptor.toString()));
        descriptionById.remove(descriptor);
        return false;
      }
    }

    return true;
  }

  /**
   * Add descriptor to native VisionProcess. This is a separate method so it can
   * be overridden in super classes.
   *
   * @param descriptor - to add to native side
   * @return if addition was successful
   */
  protected synchronized boolean addProcessingDescriptorToNative(final Term descriptor, final Long callerId) {
    // START TEST //
//    com.vision.common.fol.swig.Symbol nativeSymbol = new com.vision.common.fol.swig.Symbol("test_symbol");
//    com.vision.common.fol.swig.Variable nativeVariable = new com.vision.common.fol.swig.Variable("test_var_name", "test_var_type");
//    com.vision.common.fol.swig.Predicate nativePredicate = new com.vision.common.fol.swig.Predicate("test_predicate", nativeSymbol);
//    com.vision.common.fol.swig.Symbol arg0 = new com.vision.common.fol.swig.Symbol("arg1");
//    com.vision.common.fol.swig.Symbol arg1 = new com.vision.common.fol.swig.Variable("arg2", "arg2type");
//    com.vision.common.fol.swig.PredicateBuilder predicateBuilder = new com.vision.common.fol.swig.PredicateBuilder();
//    predicateBuilder.setName("test_predicate");
//    predicateBuilder.addArgument(arg0);
//    predicateBuilder.addArgument(arg1);
//    processor.addProcessingDescriptor(predicateBuilder.build(), callerId);
//    System.gc();
//    System.runFinalization();
//    
//    processor.printPredicate();

    // END TEST //
    String processingDescriptor = parsePredicate(descriptor);
    return processor.addProcessingDescriptor(processingDescriptor, callerId);
  }

  /**
   * Remove descriptor to VisionProcess (e.g., remove "red" from color
   * processor)
   *
   * @param descriptor - Predicate to remove
   * @param caller - the object removing the descriptor (almost always a
   * SearchManager)
   * @return - if removal was successful
   */
  final public synchronized boolean removeProcessingDescriptor(final Term descriptor, final SearchManager caller) {
    isValid();

    if (removeProcessingDescriptorFromNative(descriptor, caller.getTypeId())) {
      if (description.get(caller) != null && description.get(caller).remove(descriptor)) {
        return true;
      } else {
        log.error("[removeProcessingDescriptor] WARNING: inconsistent descriptor data.");
      }
    }
    return false;
  }

  /**
   * Remove descriptor from native VisionProcess. This is a separate method so
   * it can be overridden in super classes.
   *
   * @param descriptor - to remove from native side
   * @return if addition was successful
   */
  protected synchronized boolean removeProcessingDescriptorFromNative(final Term descriptor, final Long callerId) {
    isValid();

    String processingDescriptor = parsePredicate(descriptor);
    return processor.removeProcessingDescriptor(processingDescriptor, callerId);
  }

  public synchronized List<Term> getProcessingDescriptors(Object caller) {
    return new ArrayList<>(description.get(caller));
  }

  /**
   * Parse predicate for appropriate descriptor to send to C++ processor.
   * Detectors, ImageProcessors, etc, should override this method if it's not
   * general enough.
   *
   * @param descriptor
   * @return
   */
  public String parsePredicate(Term descriptor) {
    isValid();

    return PredicateHelper.getNativeStringRepresentation(descriptor);
  }

  /**
   * Get the number of processing descriptors.
   *
   * @return
   */
  public synchronized int getProcessingDescriptorsSize() {
    isValid();

    return description.size();
  }

  /**
   * Check if underlying native vision processor has learned.
   * @return
   */
  public synchronized boolean hasLearned() {
    isValid();

    return processor.hasLearned();
  }

  /**
   * Reset underlying native vision processor's has learned flag, so that
   * learning can happen repeatedly.
   */
  public synchronized void resetHasLearned() {
    isValid();

    processor.resetHasLearned();
  }

  /**
   * Get whether or not VisionProcess display flag is set.
   *
   * @return
   */
  public synchronized boolean getDisplayFlag() {
    isValid();

    return processor.getDisplayFlag();
  }

  /**
   * Turn on VisionProcess display.
   *
   * @param flag
   */
  public synchronized void setDisplayFlag(boolean flag) {
    isValid();

    if (flag == processor.getDisplayFlag()) {
      return;
    }

    if (flag) {
      processor.turnDisplayOn(toString());
    } else {
      processor.turnDisplayOff();
    }
  }

  /**
   * Get incremental processing flag.
   *
   * @return
   */
  public synchronized boolean getIncrementalProcessing() {
    isValid();

    return processor.getIncrementalProcessing();
  }

  /**
   * Turn on/off incremental processing.
   *
   * @param flag
   */
  public synchronized void setIncrementalProcessing(boolean flag) {
    isValid();

    processor.setIncrementalProcessing(flag);
  }

  /**
   * Get serial processing flag.
   *
   * @return
   */
  public synchronized boolean getSerialProcessing() {
    isValid();

    return processor.getSerialProcessing();
  }

  /**
   * Turn on/off serial processing.
   *
   * @param flag
   */
  public synchronized void setSerialProcessing(boolean flag) {
    isValid();

    processor.setSerialProcessing(flag);
  }

  /**
   * Get single iteration flag.
   *
   * @return
   */
  public synchronized boolean getSingleIteration() {
    isValid();

    return singleIteration;
  }

  /**
   * Turn on/off single iteration processing.
   *
   * @param flag
   */
  public synchronized void setSingleIteration(boolean flag) {
    isValid();

    singleIteration = flag;
  }

  /**
   * Is VisionProcess running.
   *
   * @return
   */
  public synchronized boolean isRunning() {
    isValid();

    return (runFuture == null ? false : !runFuture.isDone());
  }

  /**
   * Check if caller has called start on this VisionProcessor.
   * @return
   */
  public synchronized boolean isStartCaller(final Object caller) {
    isValid();

    return startCallers.contains(caller);
  }

  /**
   * Run VisionProcess.
   *
   * @param caller
   */
  public synchronized void start(final Object caller) {
    isValid();

    log.trace("[start] called.");

    //keep track of start/stop callers (in case this is shared by multiple Objects)
    if (startCallers.contains(caller)) {
      log.warn(String.format("[start] called: %s. already contains caller.", toString()));
      return;
    } else {
      //System.out.println("start called: " + toString() + ". adding caller to list.");
      //add previously added descriptor(s) from caller to native side
      List<Term> callerDescriptors = description.get(caller);
      if (null != callerDescriptors) {
        Long callerId = -1L;
        if (caller instanceof SearchManager) {
          callerId = ((SearchManager) caller).getTypeId();
        }
        for (Term descriptor : callerDescriptors) {
          boolean result = addProcessingDescriptorToNative(descriptor, callerId);
          log.debug(String.format("[start] add descriptor to native side: %s. result: %s.", descriptor, (result ? "true" : "false")));
        }
      }
      startCallers.add(caller);
    }

    //if already stopped, or in the process of stopping, it is safe to add another task to the executor
    //if NOT stopped or NOT in the process of stopping, don't add another task
    if (runFuture != null && !(runFuture.isCancelled() || runFuture.isDone())) {
      log.debug(String.format("[start] called: %s CANT (RE)START.", toString()));
      return;
    }

    log.trace(String.format("[start] called: %s. submitting start to executor.", toString()));

    runFlag = true;
    final Object thisProcessor = this;
    runFuture = executor.submit(new Runnable() {
      @Override
      public void run() {
        log.trace(String.format("[start] called: %s. running job.", thisProcessor.toString()));
        try {
          //initialze native processor
          processor.init();

          //set running status to true
          processor.setRunningStatus(true);

          //add entry to FPS GUI
          Vision.FPSpanel.addEntry(thisProcessor.toString());

          //main processing loop
          perform();
        } catch (Exception e) {
          log.error("[start] Exception caught in perform.", e);
        }
      }
    });
  }

  /**
   * Stop running VisionProcess and destroy native data.
   *
   * @param caller
   * @param wait If method should wait on stop to complete or return
   * immediately.
   */
  public synchronized void stop(final Object caller, final boolean wait) {
    isValid();
    log.trace("stop called.");

    if (runFuture == null) {    //never started
      return;
    }

    //keep track of stop/start callers
    if (!startCallers.contains(caller)) {
      log.warn(String.format("stop called: %s. caller did not start.", toString()));
      return;
    } else {
      //System.out.println("stop called: " + toString() + ". caller did start.");
      //remove previously added descriptor(s) from caller from native side
      List<Term> callerDescriptors = description.get(caller);
      if (null != callerDescriptors) {
        Long callerId = -1L;
        if (caller instanceof SearchManager) {
          callerId = ((SearchManager) caller).getTypeId();
        }
        for (Term descriptor : callerDescriptors) {
          removeProcessingDescriptorFromNative(descriptor, callerId);
        }
      }
      startCallers.remove(caller);
    }

    //only stop if all callers of start have called stop
    if (startCallers.isEmpty()) {
      log.trace(String.format("stop called: %s. stopping.", toString()));

      //stop vision process
      runFlag = false;

      //do any memory clean up here!! (added to executor queue)
      if (wait) {
        stopHelper();
      } else {
        Thread stopThread = new Thread() {
          @Override
          public void run() {
            stopHelper();
          }
        };
        stopThread.setName("VisionProcStopThread" + processId);
        stopThread.start();
      }
    }
  }

  /**
   * Stop helper ensures the vision processing job is complete and submits a new
   * job to finish any work and cleanup the native vision processor when stop or
   * terminate has been called. This method blocks until both the vision
   * processing job and cleanup job finish execution.
   */
  private void stopHelper() {
    log.trace("[stopHelper] called.");

    //can't stop vision process (i.e., interruptNotificationWait) from stop job 
    //on executor bc that job will only be executed *after* processing job
    waitUntilProcessorIsDone();

    final Object thisRef = this;
    final String thisString = thisRef.toString(); //get this here, to prevent deadlock
    if (stopFuture == null || stopFuture.isDone()) {
      stopFuture = executor.submit(new Runnable() {
        @Override
        public void run() {
          log.trace("[stopHelper] running.");
          //stop image processing dependecies
          for (ImageProcessor dependency : dependencies.values()) {
            dependency.stop(thisRef, true);
          }

          //set running status to false
          log.trace("[stopHelper] set run status.");
          processor.setRunningStatus(false);

          //close display
          log.trace("[stopHelper] turnDisplayOff.");
          processor.turnDisplayOff();

          //finish up any unfinished introspection work
          log.trace("[stopHelper] finish.");
          performanceInfo.finish();

          //any cleanup that needs to be done before native destructor is called
          //(guaranteed to run in same thread as "perform")
          log.trace("[stopHelper] cleanup.");
          processor.cleanup();

          log.trace("[stopHelper] done.");

          Vision.FPSpanel.removeEntry(thisString);
        }
      });
    }

    //wait for helper cleanup job to finish
    log.trace("Waiting for stop helper to complete.");
    while (!runFuture.isDone()) {
      try {
        stopFuture.get(1000, TimeUnit.MILLISECONDS);
      } catch (InterruptedException | ExecutionException | TimeoutException e) {
        log.error("Interrupted while waiting for stop to complete.");
      }
    }
  }

  /**
   * Ensure main perform job has finished.
   */
  private void waitUntilProcessorIsDone() {
    //ensure main perform job has finished
    while (!runFuture.isDone()) {
      log.debug(String.format("[stopHelper] waiting for %s to finish.", toString()));
      processor.interruptNotificationWait();
      try {
        //wait for runFuture to finish current job
        runFuture.get(1000, TimeUnit.MILLISECONDS);
      } catch (InterruptedException | ExecutionException | TimeoutException e) {
        log.debug("[stopHelper] still waiting...");
      }
    }
    log.debug("[stopHelper] runFuture finished.");
  }

  /**
   * This method must be called when a VisionProcess is "deleted" so that its
   * ImageProcessor dependencies (if any) can be "freed" from the
   * AvailableImageProcessors factory. An VisionProcess is no longer valid after
   * this method is called.
   */
  protected synchronized void terminate() {
    isValid();

    //should already be stopped
    boolean isRunning = isRunning();
    if (isRunning() || !startCallers.isEmpty()) {
      log.error(String.format("Being terminated without being properly stopped! isRunning: %b. num startCallers: %d. Attempting to terminate anyways.",
              isRunning, startCallers.size()));

      //try to stop vision process
      runFlag = false;

      //submit stop/cleanup task to executor
      stopHelper();
    }

    //make instance invalid
    validFlag = false;

    //cleanup and shutdown
    executor.shutdown();    //disallow additional tasks to be submitted
    try {
      // Wait a while for existing tasks to terminate
      if (!executor.awaitTermination(10, TimeUnit.SECONDS)) {
        executor.shutdownNow(); // Cancel currently executing tasks
        // Wait a while for tasks to respond to being cancelled
        if (!executor.awaitTermination(10, TimeUnit.SECONDS)) {
          log.error(String.format("[terminate] %s did not terminate.", toString()));
        }
      }
    } catch (InterruptedException ie) {
      // (Re-)Cancel if current thread also interrupted
      executor.shutdownNow();
      // Preserve interrupt status
      Thread.currentThread().interrupt();
    }

    //"release" dependencies
    for (ImageProcessor dependency : dependencies.values()) {
      ImageProcessorType dependencyType = dependency.getType();
      if (Vision.availableSaliencyProcessors.hasCapableProcessorType(dependencyType)) {
        Vision.availableSaliencyProcessors.release(this, dependency);
      } else if (Vision.availableValidationProcessors.hasCapableProcessorType(dependencyType)) {
        Vision.availableValidationProcessors.release(this, dependency);
      } else if (Vision.availableImageProcessors.hasCapableProcessorType(dependencyType)) {
        Vision.availableImageProcessors.release(this, dependency);
      } else {
        log.error("At least one dependency could not be released!");
      }
    }

    //delete native procesor (this would be deleted by GC, but we may as well do it now)
    processor.delete();
  }

  /**
   * Every public class method, even subclass methods, needs to call this to
   * ensure that {@code terminate} has not been called.
   */
  protected void isValid() {
    if (!validFlag) {
      throw new IllegalStateException("VisionProcess has been terminated and is no longer valid."
              + " VisionProcess: " + toString());
    }
  }

  /**
   * Main processing method to be written by sub-class.
   */
  protected abstract void perform();
  // DEBUGGING ONLY !!! ========================================
//    @Override
//    protected void finalize() throws Throwable {
//        System.out.println("Finalizer: " + toString());
//    }
  // DEBUGGING ONLY !!! ========================================
}
