/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.consultant;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import java.util.ArrayList;
import java.util.List;

public class VisionReference extends Reference {
  /**
   * Search type id in vision component.
   */
  public Long typeId;
  /**
   * List of detected objects' ids for this reference.
   */
  public List<Long> tokenIds;

  public VisionReference(Symbol objectRef, Variable variable) {
    super(objectRef, variable);
    this.typeId = -1L;
    this.tokenIds = new ArrayList<>();
  }

  protected VisionReference(Symbol objectRef, Variable variable, List<Term> properties) {
    super(objectRef, variable, properties);
    this.typeId = -1L;
    this.tokenIds = new ArrayList<>();
  }

  protected VisionReference(Symbol objectRef, Variable variable, Long typeId, List<Long> tokenIds, List<Term> properties) {
    super(objectRef, variable, properties);
    this.typeId = typeId;
    this.tokenIds = new ArrayList<>(tokenIds);
  }

  public VisionReference(VisionReference other) {
    super(other.refId, other.variable, new ArrayList<>());
    // convert all properties to string and then re-recreate pred so that proper Variable type is added and correct
    // FOL class is used when loading from file (e.g., creating Variables instead of Symbols because JSON can't handle
    // derived classes when parsing so will just use the java type explicitly listed and truncate the derived class)
    other.properties.forEach(prop -> properties.add(PredicateHelper.createPredicate(prop.toString())));
    this.typeId = other.typeId;
    this.tokenIds = new ArrayList<>(other.tokenIds);
  }
}
