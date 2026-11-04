/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.dialogue;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.interfaces.ConsultantInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

//TODO:brad: I'm not sure if we should continue to use this class for what it is being used for with PA. I think a re-implementation using the ContextConsultant would make more sense. If that happens this class no-longer needs to exist.
public class DialogueConsultant implements ConsultantInterface {
  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  //TODO:brad: should this be dialogue?
  private String KBname="dialog";
  protected final DiaLog diaLog;
  List<Term> defaultConsultantProperties = Arrays.asList(
          PredicateHelper.createPredicate("doit(X:"+KBname+")",KBname),
          PredicateHelper.createPredicate("dothis(X"+KBname+")",KBname),
          PredicateHelper.createPredicate("dothat(X"+KBname+")",KBname),
          PredicateHelper.createPredicate("that(X"+KBname+")",KBname)
  );
  Map<Symbol,Term> dialogrefs;
  private HashSet<TRADEServiceInfo> propertyNotificationSubscribers = new HashSet<>();

  public DialogueConsultant(DiaLog d) {
      log.trace("Entering Constructor");
      dialogrefs = new HashMap<>();
      diaLog = d;
      log.trace("Running...");
  }

  @Override
  public java.util.Map<Symbol, Double> getActivatedEntities() {
    Map<Symbol,Double> domain = new HashMap<>();

    // Generate an iterator. Start just after the last element.
    ListIterator<Utterance> li = diaLog.getDialogueHistory().listIterator(diaLog.getDialogueHistory().size());
    while(li.hasPrevious()) {
      Utterance u = li.previous();
      if(!u.getBoundSemantics().isEmpty()) {
        List<Symbol> predicates = new ArrayList<>();
        predicates.add(u.getBoundSemantics().get(0));
        while (!predicates.isEmpty()) {
          List<Symbol> args = new ArrayList<>();
          for (Symbol s : predicates) {
            if (s.isTerm()) {
              if (s.getName().equals("itk")) {
                //get relevant arguments about the action
                Symbol question = ((Term) s).get(1);
                //remove variable
                List<Symbol> measures = ((Term) question).getArgs();
                measures.remove(measures.size() - 1);
                Symbol refid = Factory.createSymbol("dialog_" + dialogrefs.size(), KBname);
                dialogrefs.put(refid, new Term("dialogref", measures));
                domain.put(refid, 1.0);
                return domain;
              }
              args.addAll(((Term) s).getArgs());
            }
          }
          predicates = args;
        }
      }
    }
    return domain;
//        return consultant.getActivatedEntities();
  }


  @Override
  public String getKBName() {
    //  if (consultant == null)
	  //return "";
    //  else {
	  //return consultant.name();
    //  }
    return KBname;
  }

  @Override
  public List<Term> getPropertiesHandled() {
    //return consultant.propertiesHandledJava();
//    try {
//      //TODO:brad: can we just make this take in the dialogue history object directly?
//      dialogHistory = (List<Utterance>) TRADE.callThe("getDialogueHistory");
//    } catch (TRADEException e) {
//      log.warn("Could not call getDialogueHistory.", e);
//    }

    return defaultConsultantProperties;

  }

  @Override
  public java.util.List<Symbol> getInitialDomain(List<Term> query) {
    //TODO:brad: this isn't how this should work
    //return consultant.getInitialDomain(query);
    List<Symbol> domain = new ArrayList<>();
    domain.add(Factory.createSymbol("dialog_"+(dialogrefs.size()-1),getKBName()));
    return domain;
    //return new ArrayList<>(dialogrefs.keySet());
  }

  @Override
  public Double process(Term constraint, Map<Variable, Symbol> bindings) {
    //log.debug("Going to call process. Consultant is ready, right? " + consultant);
    //return consultant.process(constraint, bindings);

    log.debug("Being asked how probable it is that " + constraint + " holds under bindings " + bindings);

    // convert property to Term
    Term property = constraint;
    // check against default vision consultant properties (e.g., it, that, ...)
    for (Term defaultProperty : defaultConsultantProperties) {
      if (Utilities.predicatesMatch(defaultProperty, property)) {
        return 1.0;
      }
    }

    log.debug("visionRef does NOT contains property: " + property);
    return 0.0;
  }

  @Override
  public Map<Variable, Symbol> createReferences(List<Variable> vars) {
    //TODO:brad: is this going to break stuff?
    return new HashMap<>();
  }

  @Override
  public boolean assertProperties(java.util.Map<Variable, Symbol> bindings, Double prob, java.util.List<Term> properties) {
    //return consultant.assertProperties(vars, bindings, prob, properties);
    //TODO:brad: is this going to break stuff?
    return false;
  }

  @Override
  public List<Term> getAssertedProperties(Symbol refId) {
    //return consultant.getAssertedProperties(refId);
    List<Term> lst = new ArrayList<>();
    if(dialogrefs.containsKey(refId)) {
      lst.add(dialogrefs.get(refId));
    }else{
      log.warn("[getAssertedProperties] trying to get properties for refId that doesn't have corresponding reference: "+refId);
    }
    return lst;
  }

  @Override
  public boolean assertProperties(Symbol refId, List<Term> properties) {
    return false;
  }

  @Override
  public <U> U convertToType(Symbol refId, Class<U> type) {
    return null;
  }

  @Override
  public <U> U convertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    return null;
  }

  //todo:(pete) this duplicates code in Consultant. Shbould we be doing something else with the interface/Consultant implementation distinction?
  @Override
  public void registerForNewPropertyNotification(TRADEServiceInfo callback) {
    propertyNotificationSubscribers.add(callback);
  }

  @Override
  public void unregisterForNewPropertyNotification(TRADEServiceInfo callback) {
    propertyNotificationSubscribers.remove(callback);
  }

  @Override
  public void notifyNewPropertySubscribers() {
    for (TRADEServiceInfo subscriber : propertyNotificationSubscribers) {
      try {
        subscriber.call(void.class);
      } catch (TRADEException e) {
        log.error("Could not make callback to: " + subscriber, e);
      }
    }
  }
}
