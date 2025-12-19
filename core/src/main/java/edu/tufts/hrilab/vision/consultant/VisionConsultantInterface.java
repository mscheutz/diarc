/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.consultant;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.ConsultantInterface;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import java.util.List;

public interface VisionConsultantInterface extends ConsultantInterface {

  /**
   * Get the underlying vision typeId associated with the object reference.
   * @param objectRef
   * @return
   */
  Long getTypeId(Symbol objectRef);

  /**
   * Get tokens for an object reference. For now, this method just calls the
   * getTokens method that takes in a typeId. In the future, this should
   * probably not automatically start a visual search, but instead look through LTM
   * which currently doesn't exist.
   * @param objectRef
   * @return
   */
  List<MemoryObject> getTokens(Symbol objectRef);

  /**
   * Get token IDs for an object reference. For now, this method just calls the
   * getTokenIds method that takes in a typeId. In the future, this should
   * probably not automatically start a visual search, but instead look through LTM
   * which currently doesn't exist.
   * @param objectRef
   * @return
   */
  List<Long> getTokenIds(Symbol objectRef);

}
