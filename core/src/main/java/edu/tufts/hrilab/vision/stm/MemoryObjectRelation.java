/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Term;
import java.io.Serializable;

/**
 *
 * @author Evan Krause evan.krause@tufts.edu
 */
public class MemoryObjectRelation implements Serializable {

  /**
   * Detection or validation confidence [0 1].
   */
  private float confidence;
  /**
   * Descriptor (e.g., on(X,Y), near(X,Z), etc).
   */
  private Term descriptor;
  /**
   * Other MemoryObject that relationship is between.
   */
  private MemoryObject relatedObject;

  /**
   * MemoryObjectRelation constructor.
   *
   * @param confidence
   * @param descriptor
   * @param relatedObject
   */
  MemoryObjectRelation(float confidence, Term descriptor, MemoryObject relatedObject) {
    this.confidence = confidence;
    this.descriptor = descriptor;
    this.relatedObject = relatedObject;
  }

  /**
   * Get confidence of this relation.
   *
   * @return
   */
  public float getConfidence() {
    return confidence;
  }

  /**
   * Get relation in predicate form. Returns a direct reference, not a copy.
   *
   * @return
   */
  public Term getDescriptor() {
    return descriptor;
  }

  /**
   * Get other MemoryObject that relationship is between. Returns a direct
   * reference, not a copy.
   *
   * @return
   */
  public MemoryObject getRelatedObject() {
    return relatedObject;
  }

}
