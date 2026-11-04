/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.location;

import ai.thinkingrobots.trade.TRADEService;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.ArrayList;
import java.util.List;

public class ABBLocationConsultant extends Consultant<ABBLocationReference> {

  public ABBLocationConsultant() {
    super(ABBLocationReference.class, "location");
  }

  @Override
  protected <U> U localConvertToType(Symbol refId, Class<U> type) {
    return localConvertToType(refId, type, new ArrayList<>());
  }

  @Override
  protected <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    log.error("[convertToType] Can not handle conversions to type: " + type);
    return null;
  }

  @TRADEService
  public Symbol saveLocation(Symbol descriptor) {
    String descriptorName = descriptor.getName();
    List<Term> props = new ArrayList<>();
    props.add(Factory.createPredicate(descriptorName, "X:" + getKBName()));

    Symbol newRefID = getNextReferenceId();
    ABBLocationReference newLocationRef = new ABBLocationReference(
            newRefID,
            Factory.createVariable("X", getKBName()),
            props
    );

    addReference(newLocationRef);

    return newRefID;
  }
}
