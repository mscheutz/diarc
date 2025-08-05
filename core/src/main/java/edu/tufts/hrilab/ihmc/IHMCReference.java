/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ihmc;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;

public class IHMCReference extends Reference {
  private String ihmcName;

  public IHMCReference(Symbol ref, Variable variable) {
    super(ref, variable);
  }

  public IHMCReference(Symbol ref, Variable variable, List<Term> properties) {
    super(ref, variable, properties);
  }

  public String getIHMCName() {
    return ihmcName;
  }

  public void setIHMCName(String ihmcName) {
    this.ihmcName = ihmcName;
  }
}
