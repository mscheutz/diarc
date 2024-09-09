/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
/**
 * Defines the set of remotely available methods.
 */
package edu.tufts.hrilab.vision;

import java.awt.Dimension;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;

import java.util.List;
import java.util.Map;

/**
 * Interface for the DIARC Vision Component. This adds functionality that doesn't
 * exist in edu.tufts.hrilab.interfaces.VisionComponent and is only available in the "real"
 * VisionComponent.
 */
public interface VisionInterface extends edu.tufts.hrilab.interfaces.VisionInterface {
  @TRADEService
  @Action
  void pauseCapture();

  @TRADEService
  @Action
  void resumeCapture();

  @TRADEService
  @Action
  void stopAllSearches() ;

  @TRADEService
  @Action
  void restartAllStoppedSearches() ;


  //=============================================================
  //======================= START: move these into interface =====
  //=============================================================

  /**
   * Name the collection of Predicate descriptors. The descriptors must belong
   * to a single SearchManager that already exists, or an attempt to create a
   * new SearchManager matching the descriptors will be made. The description
   * must match the SearchManager description exactly.
   *
   * @param descriptors Predicate description of object
   * @param typeName Predicate name of object (name will be bound to typeId in
   * VisionComponent)
   * @return true if descriptors map to an existing typeId, false otherwise
   */
  @TRADEService
  boolean nameDescriptors(List<? extends Symbol> descriptors, Symbol typeName);

  //=============================================================
  //======================= END: move these into interface =====
  //=============================================================

  /**
   * Find what part(s) of the requested descriptors vision doesn't know how to process.
   *
   * @param descriptors
   * @return list of (possibly empty) unsatisfiable constraints
   */
  @TRADEService
  @Action
  List<Symbol> getUnsatisfiableConstraints(List<? extends Symbol> descriptors);

  /**
   * Find what part(s) of the requested descriptor vision doesn't know how to process.
   *
   * @param descriptor
   * @return list of (possibly empty) unsatisfiable constraints
   */
  @TRADEService
  @Action
  List<Symbol> getUnsatisfiableConstraints(Symbol descriptor);

  /**
   * Use the passed in term to learn a new descriptor. The term must have two
   * arguments, the first arg 'describing' the new thing-to-learn, and the
   * second arg 'naming' the new thing-to-learn. The description must be
   * composed entirely of things vision already knows about.
   *
   * @param learningTerm term specifying the kind of learning to do (e.g., instanceOf(X,Y), definitionOf(X,Y))
   * @param bindings variable bindings to any variables in query term
   * @return if thing-to-learn was successfully learned
   */
  @TRADEService
  boolean learn(Term learningTerm, List<Map<Variable, Symbol>> bindings);

  /**
   * Un-learns things that have been learned by the learn method.
   * @param learningTerm
   * @param bindings
   * @return
   */
  @TRADEService
  boolean unlearn(Term learningTerm, List<Map<Variable, Symbol>> bindings);

  /**
   * Get List of all SearchManager IDs in long term memory.
   *
   * @return List of MemoryObjectType IDs
   */
  @TRADEService
  List<Long> getAvailableTypeIds();

  /**
   * Start detecting/tracking specified SearchManager.
   *
   * @param typeId
   */
  @TRADEService
  @Action
  void startType(long typeId);

  /**
   * Stop detecting/tracking specified SearchManager.
   *
   * @param typeId
   */
  @TRADEService
  @Action
  void stopType(long typeId);

  /**
   * Return the camera's last frame in a byte array.
   *
   * @return the image in a byte array
   */
  @TRADEService
  byte[] getFrame();

  /**
   * Return the camera's last disparity frame in a byte array.
   * @return
   */
  @TRADEService
  byte[] getDisparityFrame();

  /**
   * Return the camera's last depth frame in a byte array.
   * @return
   */
  @TRADEService
  byte[] getDepthFrame();

  /**
   * Save the current frame to file (e.g., for logging).
   *
   * @param filename
   */
  @TRADEService
  void takeSnapshot(String filename);

  /**
   * Returns the size of the image being used.
   *
   * @return Dimension containing the size
   */
  @TRADEService
  Dimension getImageSize();

  /**
   * Entry point in vision for making world observations.
   * @param observation single term specifying observation to make
   * @return bindings for any free variables in observation term
   */
  @TRADEService
//  @Observes({"touching(mug,table)"})
  @Observes({"touching(X,Y)"})
  List<Map<Variable, Symbol>> makeObservations(Term observation);

  @TRADEService
  List<String> getMyGroups();
}
