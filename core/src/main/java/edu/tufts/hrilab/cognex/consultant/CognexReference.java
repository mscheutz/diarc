/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.cognex.consultant;

import edu.tufts.hrilab.abb.AbbCognexResult;
import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

public class CognexReference extends Reference {
  public AbbCognexResult result = null; //token
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

  public CognexReference(Symbol objectRef, Variable variable, AbbCognexResult result, CognexJob cognexJob) {
    super(objectRef, variable);
    this.result = result;
    this.cognexJob = cognexJob;
  }

  public void setCognexJob(CognexJob cognexJob) {
    this.cognexJob = cognexJob;
  }

  public void setResult(AbbCognexResult result) {
    this.result = result;
  }

  public CognexJob getJob() {
    return cognexJob;
  }
}
