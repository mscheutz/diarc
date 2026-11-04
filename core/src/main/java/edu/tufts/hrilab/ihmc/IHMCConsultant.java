/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ihmc;

import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.List;

public class IHMCConsultant extends Consultant<IHMCReference> {
  public IHMCConsultant(Class<IHMCReference> refClass, String kbName) {
    super(refClass, kbName);
  }

  public IHMCConsultant(Class<IHMCReference> refClass, String kbName, List<String> properties) {
    super(refClass, kbName, properties);
  }

  @Override
  protected <U> U localConvertToType(Symbol refId, Class<U> type) {
    return null;
  }

  @Override
  protected <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    return null;
  }
}
