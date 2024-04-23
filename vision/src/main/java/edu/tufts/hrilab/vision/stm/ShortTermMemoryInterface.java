/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.vision.stm.swig.ShortTermMemoryModule;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class calls into native code to retrieve requested info/data from
 * tracked MemoryObjects. It does not store any memory objects, it only hands
 * off the requested data. The getShortTermMemory() can populate an entire STM
 * and hand it off, but this is only a snapshot of the STM at the time it was
 * requested, and is not updated.
 *
 * TODO: have the Java side STM continually updated to reflect most recent
 * native STM.
 *
 * @author Evan Krause
 */
public class ShortTermMemoryInterface {

  static {
    System.loadLibrary("tracker");
  }
  private static Logger log = LoggerFactory.getLogger(ShortTermMemoryInterface.class);

  /**
   * Get all MemoryObjectType IDs with at least one object in short term memory
   * with a confidence level above the specified threshold.
   *
   * @param conf
   * @return list of MemoryObjectType IDs
   */
  public static List<Long> getTypeIds(final double conf) {
    ArrayList<Long> toReturn = new ArrayList();
    long[] results = ShortTermMemoryModule.getMemoryObjectTypeIds(conf);

    //update introspection data - only update types that are being returned
    long currTime = System.currentTimeMillis();
    for (long currTypeId : results) {
      toReturn.add(currTypeId);
      SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, currTypeId);
      if (currSearchType != null) {
        currSearchType.setTimeOfLastClientUse(currTime);
      }
    }

    return toReturn;
  }

  /**
   * Get all MemoryObject IDs in the short term memory with a confidence level
   * over the specified threshold.
   *
   * @param conf
   * @return
   */
  public static List<Long> getTokenIds(final double conf) {
    log.trace("[getTokenIds] method entered.");
    ArrayList<MemoryObject> resultMOs = ShortTermMemoryModule.getMemoryObjects(conf);
    log.trace("[getTokenIds] got tokenIds from native side.");
    ArrayList<Long> resultIds = new ArrayList();

    //update introspection data - only update types that are being returned
    ListIterator<MemoryObject> moItr = resultMOs.listIterator();
    long currTime = System.currentTimeMillis();
    while (moItr.hasNext()) {
      MemoryObject currMO = moItr.next();
      resultIds.add(currMO.getTokenId());
      SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, currMO.getTypeId());
      if (currSearchType != null) {
        currSearchType.setTimeOfLastClientUse(currTime);
      }
    }

    return resultIds;
  }

  /**
   * Get all MemoryObject IDs of the specified MemoryObjectType in the short
   * term memory with a confidence level over the specified threshold.
   *
   * @param typeId
   * @param conf
   * @return
   */
  public static List<Long> getTokenIds(final long typeId, final double conf) {
    //System.out.println("in getAllByType");
    ArrayList<Long> result = new ArrayList<Long>();
    long[] intResult = ShortTermMemoryModule.getMemoryObjectIds(typeId, conf);
    //System.out.println("got results in getAllByType. num elements: " + intResult.length);
    for (long currId : intResult) {
      result.add(currId);
    }
    //System.out.println("getAllByType results: " + intResult);

    //update introspection data
    SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, typeId);
    if (currSearchType != null) {
      long currTime = System.currentTimeMillis();
      currSearchType.setTimeOfLastClientUse(currTime);
    }

    return result;
  }

  /**
   * Get all MemoryObjects in the short term memory with a confidence level over
   * the specified threshold.
   *
   * @param conf
   * @return
   */
  public static List<MemoryObject> getTokens(final double conf) {
    log.trace("[getTokens] method entered.");
    List<MemoryObject> result = (ArrayList) ShortTermMemoryModule.getMemoryObjects(conf);
    log.trace("[getTokens] got tokens from native side.");

    // apply any relavant definitions to memory object scene graph
    List<NamedDescription> definitions = Vision.availableSearchTypes.getDefinitions();
    for (MemoryObject mo : result) {
      for (NamedDescription definition : definitions) {
        MemoryObjectUtil.applyDefinition(mo, definition.getName(), definition.getDescriptors());
      }
    }

    //update introspection data - only update types that are being returned
    ListIterator<MemoryObject> moItr = result.listIterator();
    long currTime = System.currentTimeMillis();
    log.trace("[getTokens] updating introspection data...");
    while (moItr.hasNext()) {
      MemoryObject currMO = moItr.next();
      SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, currMO.getTypeId());
      if (currSearchType != null) {
        currSearchType.setTimeOfLastClientUse(currTime);
      }
    }
    log.trace("[getTokens] done updating introspection data.");

    // TODO: EAK: this doesn't belong here and also doesn't match the semantics in assertActivatedEntities
//    ArrayList<MemoryObject> removedMOs = ShortTermMemoryModule.getRemovedMemoryObjects();
//    // log.trace(removedMOs.size());
//    for (MemoryObject mo : removedMOs) {
//
//      // Should always be size 1, but handle for more than 1 case
//      for (Symbol ref : Vision.consultant.getReferences(mo.getTypeId())) {
//        Term seePred = new Term("see", "self", ref.toString());
//        // log.trace(ref);
//        // log.trace(seePred);
//        Vision.consultant.retractBelief(seePred);
//      }
//    }

    return result;
  }

  /**
   * Get all MemoryObjects of the specified MemoryObjectType in the short term
   * memory with a confidence level over the specified threshold.
   *
   * @param typeId
   * @param conf
   * @return
   */
  public static List<MemoryObject> getTokens(final long typeId, final double conf) {
    //System.out.println("stmi getTokensByTypeId: " + typeId);
    ArrayList<MemoryObject> result = new ArrayList(ShortTermMemoryModule.getMemoryObjectsByTypeId(typeId, conf));
    //System.out.println("stmi results: " + result);

    // apply any relevant definitions to memory object scene graph
    List<NamedDescription> definitions = Vision.availableSearchTypes.getDefinitions();
    for (MemoryObject mo : result) {
      for (NamedDescription definition : definitions) {
        MemoryObjectUtil.applyDefinition(mo, definition.getName(), definition.getDescriptors());
      }
    }

    //set introspection data
    //update introspection data
    SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, typeId);
    if (currSearchType != null) {
      long currTime = System.currentTimeMillis();
      currSearchType.setTimeOfLastClientUse(currTime);
    }

    return result;
  }

  /**
   * Get the MemoryObject with the MemoryObject type ID and token ID if the
   * confidence level is above the specified threshold.
   *
   * @param tokenId
   * @param conf
   * @return
   */
  public static MemoryObject getToken(final long tokenId, final double conf) {
    //System.out.println("getMemoryObject id: " + id);
    MemoryObject mo = ShortTermMemoryModule.getMemoryObject(tokenId, conf);
    if (mo.getTokenId() == -1) {
      return null;
    }

    // apply any relavant definitions to memory object scene graph
    List<NamedDescription> definitions = Vision.availableSearchTypes.getDefinitions();
    for (NamedDescription definition : definitions) {
      MemoryObjectUtil.applyDefinition(mo, definition.getName(), definition.getDescriptors());
    }

    //update introspection data
    SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, mo.getTypeId());
    if (currSearchType != null) {
      long currTime = System.currentTimeMillis();
      currSearchType.setTimeOfLastClientUse(currTime);
    }

    return mo;
  }

  /**
   * Check if the MemoryObject with specified MemoryObject ID is in short term
   * memory.
   *
   * @param tokenId
   * @return
   */
  public static boolean confirmObject(final long tokenId) {
    if (tokenId == -1) {
      return false;
    }

    MemoryObject mo = ShortTermMemoryModule.getMemoryObject(tokenId, 0.0);

    //update introspection data
    SearchManager currSearchType = Vision.availableSearchTypes.getInstance(null, mo.getTypeId());
    if (currSearchType != null) {
      long currTime = System.currentTimeMillis();
      currSearchType.setTimeOfLastClientUse(currTime);
    }

    return (mo.getTokenId() == tokenId);
  }

  /**
   * Check if the MemoryObject is in short term memory.
   *
   * @param token
   * @return
   */
  public static boolean confirmObject(final MemoryObject token) {
    return confirmObject(token.getTokenId());
  }

  public static void clearMemory() {
    log.info("Clearing Memory.");
    ShortTermMemoryModule.clearMemory();
  }
}
