/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift;

import java.util.List;

import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.consultant.VisionReference;

public class AITVisionConsultant extends Consultant<VisionReference> {

  public AITVisionConsultant(Class<VisionReference> refClass, String kbName) {
    super(refClass, kbName);
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
