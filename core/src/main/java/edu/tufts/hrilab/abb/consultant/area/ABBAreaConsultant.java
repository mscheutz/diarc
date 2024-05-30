/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.area;

import ai.thinkingrobots.trade.TRADEService;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.ArrayList;
import java.util.List;

public class ABBAreaConsultant extends Consultant<ABBAreaReference> {

  private static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

  public ABBAreaConsultant() {
    super(ABBAreaReference.class, "area");
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
  public Symbol addArea(Symbol descriptor) {
    String descriptorName = descriptor.getName();
    List<Term> props = new ArrayList<>();
    props.add(Factory.createPredicate(descriptorName, "X:" + getKBName()));

    Symbol newRefID = getNextReferenceId();
    ABBAreaReference newAreaRef = new ABBAreaReference(
            newRefID,
            Factory.createVariable("X", getKBName()),
            props
    );

    addReference(newAreaRef);

    return newRefID;
  }
}
