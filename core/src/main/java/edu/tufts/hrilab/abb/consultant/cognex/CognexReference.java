/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.cognex;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

public class CognexReference extends Reference {
  public CognexResult result = null; //token
  public CognexJob cognexJob; //type

  @Override
  public String toString() {
    return "CognexReference{" +
            "result=" + result +
            ", cognexJob=" + cognexJob +
            '}';
  }

  public CognexReference(Symbol objectRef, Variable variable) {
    super(objectRef, variable);
  }

  public CognexReference(Symbol objectRef, Variable variable, CognexJob cognexJob) {
    super(objectRef, variable);
    this.cognexJob = cognexJob;
  }

  public CognexReference(Symbol objectRef, Variable variable, CognexResult result, CognexJob cognexJob) {
    super(objectRef, variable);
    this.result = result;
    this.cognexJob = cognexJob;
  }

  public void setCognexJob(CognexJob cognexJob) {
    this.cognexJob = cognexJob;
  }

  public void setResult(CognexResult result) {
    this.result = result;
  }

  public CognexJob getJob() {
    return cognexJob;
  }
}
