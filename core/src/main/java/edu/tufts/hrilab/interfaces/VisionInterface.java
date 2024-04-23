/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import java.util.List;

/**
 * Core interface for vision servers.
 */
public interface VisionInterface {

	/**
	 * Find all SearchManagers containing at least one token in STM. One each,
	 * regardless of how many tokens of a particular type are in STM.
	 *
	 * @param conf minimum confidence of tokens to be included
	 * @return SearchManager IDs
	 */
    @TRADEService
    List<Long> getTypeIds(final double conf);

	/**
	 * Get the SearchManager ID (search ID) based on a Predicate description of
	 * the object. The Predicate description must match the SearchManager
	 * description exactly. If an existing SearchManager does not match the
	 * description, a new SearchManager will be attempted to be built. This
	 * assumes that all descriptors refer to the same object. Also
	 * starts the SearchManager if not running.
	 * NOTE: currently just ignores descriptors that vision doesn't know how
	 * to handle.
	 *
	 * @param descriptors Predicate description of object
	 * @return SearchManager Id (-1 if there is no match)
	 */
    @TRADEService
    @Action
    Long getTypeId(final List<? extends Symbol> descriptors);

	/**
	 * Get the SearchManager ID (search ID) based on a Predicate description of
	 * the object. The Predicate description must match the SearchManager
	 * description exactly. If a current SearchManager does not match the
	 * description a new SearchManager will be attempted to be built. Also
	 * starts the SearchManager if not running.
	 * NOTE: This is only a convenience method so a List doesn't need to be
	 * passed when only a single Predicate is used.
	 *
	 * @param descriptor Predicate description of object
	 * @return SearchManager Id (-1 if there is no match)
	 */
    @TRADEService
    @Action
	//TODO: should this exist as an action independent of the identical signature action in VisionConsultantInterface?
    Long getTypeId(final Symbol descriptor);

	/**
	 * Get the descriptors for a particular SearchManager.
	 *
	 * @param typeId
	 * @return
	 */
    @TRADEService
    @Action
    List<Term> getDescriptors(final Long typeId);

	/**
	 * Name the collection of Predicate descriptors referred to by the
	 * SearchManager ID (search ID).
	 *
	 * @param typeId SearchManager ID (search ID)
	 * @param typeName Predicate name of object (name will be bound to typeId in
	 * VisionComponent)
	 * @return true if typeId exists, false otherwise
	 */
    @TRADEService
    boolean nameDescriptors(final Long typeId, final Symbol typeName);

	/**
	 * Get MemoryObject IDs in STM with a confidence level over the specified
	 * threshold. Only considers results from SearchManager searches that are
	 * currently running.
	 *
	 * @param conf minimum confidence of tokens to include
	 * @return List of STM MemoryObject IDs
	 */
    @TRADEService
    List<Long> getTokenIds(final double conf);

	/**
	 * Get MemoryObject IDs that have specified SearchManager ID with a
	 * confidence level above the specified threshold. If the specified
	 * SearchManager exists but is not running, it will be started before
	 * returning results.
	 *
	 * @param typeId SearchManager ID of tracked objects
	 * @param conf minimum confidence of tokens to include
	 * @return List of STM MemoryObject IDs
	 */
    @TRADEService
    @Action
    List<Long> getTokenIds(final long typeId, final double conf);

	/**
	 * Get MemoryObject IDs for all tokens *exactly* matching descriptors in STM
	 * with confidence greater than threshold. If a matching type is found, it
	 * will be started (if not already running). If no match exists, a
	 * SearchManager will attempt to be built and started before returning.
	 * This is largely a convenience method and {@code getTypeId} should be used
	 * whenever possible.
	 *
	 * @param descriptors list of Symbol descriptors
	 * @param conf the minimum confidence level
	 * @return List of STM MemoryObject IDs
	 */
    @TRADEService
    @Action
    List<Long> getTokenIds(final List<? extends Symbol> descriptors, final double conf);

	/**
	 * Get MemoryObjects in STM with a confidence over the specified threshold.
	 * Only considers results from SearchManager searches that are currently
	 * running.
	 *
	 * @param conf minimum confidence of tokens to include
	 * @return List of MemoryObjects
	 */
    @TRADEService
    List<MemoryObject> getTokens(final double conf);

	/**
	 * Get MemoryObjects of the specified SearchManager ID with confidence
	 * above specified threshold. If the specified SearchManager exists but is
	 * not running, it will be started before returning results.
	 *
	 * @param typeId SearchManager ID of tracked objects
	 * @param conf minimum confidence of tokens to include
	 * @return List of MemoryObjects
	 */
    @TRADEService
    @Action
    List<MemoryObject> getTokens(final long typeId, final double conf);

	/**
	 * Get MemoryObjects *exactly* matching descriptors in STM with confidence
	 * greater than threshold. If a matching type is found, it will be started (if
	 * not already running). If no match exists, a SearchManager will attempt
	 * to be built and started before returning. This is largely a convenience
	 * method and {@code getTypeId} should be used whenever possible.
	 *
	 * @param descriptors list of Symbol descriptors
	 * @param conf the minimum confidence level
	 * @return List of STM MemoryObjects
	 */
    @TRADEService
    @Action
    List<MemoryObject> getTokens(final List<? extends Symbol> descriptors, final double conf);

	/**
	 * Get the MemoryObject with the specified id.
	 *
	 * @param tokenId MemoryObject ID in STM
	 * @param conf minimum confidence of tokens to include
	 * @return MemoryObject token (Null if doesn't exist)
	 */
    @TRADEService
    @Action
    MemoryObject getToken(final long tokenId, final double conf);

	/**
	 * Confirms that the object is still in STM.
	 *
	 * @param tokenId MemoryObject ID of the object to be confirmed
	 * @return true if the MemoryObject is present in STM
	 */
    @TRADEService
    @Action
    boolean confirmToken(final long tokenId);

	/**
	 * Confirms that the object is still in STM.
	 *
	 * @param token MemoryObject to be confirmed
	 * @return true if the object is present in STM
	 */
    @TRADEService
    boolean confirmToken(final MemoryObject token);

    // ============== START Incremental Search Methods ====================================
	/**
	 * This instantiates a new MemoryObjecType (search manager) which can be
	 * incrementally configured via {@code addDescriptor}.
	 *
	 * @return SearchManager ID (search id)
	 */
    @TRADEService
    Long createNewType();
    
	/**
	 * Add new search constraint (i.e., ImageProcessor) to an existing
	 * SearchManager (specified by searchID).
	 *
	 * @param typeId - unique ID returned by {@code createNewType}
	 * @param descriptor - predicate describing visual search attribute (e.g.,
	 * "red", "round")
	 */
    @TRADEService
    boolean addDescriptor(final long typeId, final Symbol descriptor);
    
	/**
	 * Remove search constraint (i.e., processing descriptor and/or
	 * ImageProcessor) to an existing SearchManager (specified by searchID).
	 *
	 * @param typeId
	 * @param descriptor - to remove
	 * @return - if removal was successful
	 */
    @TRADEService
    boolean removeDescriptor(final long typeId, final Symbol descriptor);

	/**
	 * Signal the end of constraint addition. Descriptors/constraints can no
	 * longer be added to the SearchManager after this has been called. This
	 * also starts the Detector and Tracker and will choose the "object" Detector
	 * if one hasn't already been selected.
	 *
	 * @param typeId
	 */
    @TRADEService
    void endDescriptorChanges(final long typeId);
	// ============== END Incremental Search Methods ====================================

	/**
	 * Turns off search for typeId, and removes SearchManager.
	 *
	 * @param typeId
	 */
    @TRADEService
    @Action
    void stopAndRemoveType(final long typeId);

}
