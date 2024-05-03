/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.item;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.List;

public class ItemConsultant extends Consultant<ItemReference> {

  public ItemConsultant() {
    super(ItemReference.class, "item");
  }

  @Override
  protected <U> U localConvertToType(Symbol refId, Class<U> type) {
    return type.cast(references.get(refId));
  }

  @Override
  protected <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    return localConvertToType(refId, type);
  }

  @Override
  public boolean addPropertiesHandled(List<Term> properties) {
    boolean propertiesAdded = false;
    for (Term property : properties) {
      if (propertiesHandled.stream().noneMatch(p -> edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(p, property))) {
        propertiesHandled.add(property);
        propertiesAdded = true;
        String propName = property.getName();
        String strippedName = stripQuotesFromMorpheme(propName);

        // TODO: this imposes an ordering constraint on diarc components (i.e., RR needs to be up first)
        try {
          TRADEServiceInfo injectTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("injectDictionaryEntry"));
          injectTSI.call(void.class, strippedName, "RN", propName + ":" + kbName, "VAR");
          injectTSI.call(void.class, strippedName, "REF", propName + ":" + kbName, "DEFINITE");
          injectTSI.call(void.class, strippedName, "DESC", propName, "");
          injectTSI.call(void.class, strippedName, "DN", propName, "");
          injectTSI.call(void.class, strippedName, "VAL", "item(" + propName + ")", "");
        } catch (TRADEException e) {
          log.error("[addPropertiesHandled] unable to add dictionary entry for " + propName, e);
          propertiesAdded = false;
        }
      }
    }
    if (propertiesAdded) {
      notifyNewPropertySubscribers();
    }
    return propertiesAdded;
  }

  @TRADEService
  @Action
  public Symbol addItem(Symbol name) {
    String newName = name.getName();
    Term newProperty = Factory.createPredicate(newName, Factory.createVariable("X", getKBName()));
    List<Term> newProperties = List.of(newProperty);
    addPropertiesHandled(newProperties);
    return Factory.createSymbol(newName);
  }
}
