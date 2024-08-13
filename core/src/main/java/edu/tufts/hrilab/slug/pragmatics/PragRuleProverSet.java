/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.pragmatics;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.fol.util.PragUtil;
import edu.tufts.hrilab.util.Util;

import java.util.*;

public class PragRuleProverSet extends HashSet<PragRuleProver> {

  protected Logger log = LoggerFactory.getLogger(this.getClass());

  private List<Map<Integer, Utterance>> dialogContext;
  private Set<Term> envContext;
  private List<List<Term>> propertyContext;

  ///////////////////////////////////////////
  public PragRuleProverSet() {
    super();
    dialogContext = new ArrayList<>();
    envContext = new HashSet<>();
    propertyContext = new ArrayList<>();
  }
  
  ///////////////////////////////////////////
  public Set<Term> getEnvContext() {
    return envContext;
  }

  public List<List<Term>> getPropertyContext() {
    return propertyContext;
  }

  public List<Map<Integer, Utterance>> getDialogContext() {
    return dialogContext;
  }

  ///////////////////////////////////////////
  public void parseContext(String line) {
    int opar = line.indexOf('(');
    int cpar = line.lastIndexOf(')');
    String argStr = line.substring(opar + 1, cpar);
    //TODO: at some point replace Util.tokenize args with PragUtil.splitToplevel()
    List<String> tokens = Util.tokenizeArgs(argStr.trim());
    log.debug("tokens: ", tokens);

    assert tokens != null;
    if (tokens.isEmpty()) {
      return;
    } else if (tokens.get(0).equalsIgnoreCase("utterance")) {
      if (tokens.size() > 2) {
        Integer i = Integer.valueOf(tokens.get(1));
        log.debug(tokens.get(2));
        Utterance u = UtteranceUtil.createUtterance(tokens.get(2));
        Map<Integer,Utterance> contextmap = new HashMap<>();
        contextmap.put(i,u);
        dialogContext.add(contextmap);
      }
    } else if (tokens.get(0).equals("property")) {
      List<Term> list = new ArrayList<>();
      for (int i=1;i<tokens.size();i++) {
        Term t = Factory.createPredicate(tokens.get(i));
        list.add(t);
      }
      propertyContext.add(list);
    } else {
      for (String token : tokens) {
        log.debug("token " + token);
        Term t = Factory.createPredicate(token);
        envContext.add(t);
      }
    }
  }

  public List<Map<Variable,Symbol>> checkPropertyApplicability(List<Map<Variable,Symbol>> bindings, Utterance utt, List<List<Term>> contextList) {
    //check if there are reference bindings
    if (contextList.isEmpty()) {
      return bindings;
    }
    List<Map<Variable,Symbol>> totalbindings = new ArrayList<>();
    try {
      for (Map<Variable, Symbol> binding : bindings) {
        List<Term> supplementalsemantics = new ArrayList<>();
        List<Term> propertyList = contextList.get(0);
        if (!propertyList.isEmpty() && propertyList.get(0).get(0).isVariable()) {
          if (propertyList.size() == 1) {
            Term context = propertyList.get(0);  //call getProperties
            //this only works for single argument properties, i.e. not on(X,Y)\
            context = context.copyWithNewBindings(binding);
            Symbol reference = context.get(0);
            if (reference != null) {
              if (!context.getName().equals("propertyOf")) {
                //convert from "Y(X)" to "propertyOf(X,Y)"
                context = new Term("propertyOf", context.get(0).getName(), context.getName());
              }
              if (reference.getName().contains("_")) {
                //is there a reference?
                List<Map<Variable, Symbol>> contextBindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("observeProperties").argTypes(Symbol.class,Term.class))
                        .call(List.class, utt.getAddressee(), context);
                if (!contextBindings.isEmpty()) {
                  for (Map<Variable, Symbol> contextBinding : contextBindings) {
                    contextBinding.putAll(binding);
                  }
                  totalbindings.addAll(checkPropertyApplicability(contextBindings, utt, contextList.subList(1, contextList.size())));
                } else {
                  //context is not fulfilled, try the next binding map
                  break;
                }
              } else {
                //there isn't a reference here so the context cannot apply
                break;
              }
            }
          } else {
            //if there's unbound vars in there, we need to resolve them with refresolution
            for (Term context : propertyList) {
              context = context.copyWithNewBindings(binding);
              if (context.getName().equals("propertyOf")) {
                supplementalsemantics.add(new Term(context.get(1), context.get(0)));
              } else {
                supplementalsemantics.add(context);
              }
            }
            //another call to reference resolution to find a reference for a new variable. this doesn't work with more than one possible reference.
            //TODO:brad: this is suspect
            Variable var = propertyList.get(0).getOrderedVars().get(0);
            Map<Variable, Symbol> ghtierMap = new HashMap<>();
            ghtierMap.put(var, Factory.createSymbol("DEFINITE"));

            utt = new Utterance.Builder()
                    .setSpeaker(utt.getSpeaker())
                    .setAddressee(utt.getAddressee())
                    .addListeners(utt.getListeners())
                    .setSemantics(supplementalsemantics.get(0))
                    .setSupplementalSemantics(supplementalsemantics)
                    .setTierAssignments(ghtierMap)
                    .setUtteranceType(UtteranceType.UNKNOWN)
                    .setIsInputUtterance(utt.isInputUtterance()).build();

            Utterance resolvedUtt = TRADE.getAvailableService(new TRADEServiceConstraints().name("resolveReferences").argTypes(Utterance.class)).call(Utterance.class, utt);
            List<Map<Variable, Symbol>> newreferences = resolvedUtt.getBindings();
            if (!newreferences.isEmpty()) {
              for (Map<Variable, Symbol> newreference : newreferences) {
                for (Variable key : newreference.keySet()) {
                  Symbol ref = newreference.get(key);
                  newreference.remove(key);
                  newreference.put(new Variable(key.getName()),ref);
                }
                newreference.putAll(binding);
              }
              totalbindings.addAll(checkPropertyApplicability(newreferences, utt, contextList.subList(1, contextList.size())));
            } else {
              //couldn't find any references for variable
              break;
            }
          }
        }
      }
    } catch (TRADEException e) {
      log.error("Failed to call reference resolution:", e);
    }
    return totalbindings;
  }

  public List<Map<Variable,Symbol>> checkEnvironmentalApplicability(Set<Term> env) {

    Map<Variable, Symbol> binding = new HashMap<>();

    // Environmental Context
    if (!envContext.isEmpty()) {
      // TODO handle multiple possible maps
      for (Symbol s : envContext) {
        for (Symbol s2 : env) {
          Map<Variable, Symbol> tmpBinding = PragUtil.getSymbolBindings(s, s2);
          if (tmpBinding != null) {
            binding.putAll(tmpBinding);
          }
        }
      }
      if (binding.isEmpty()) {
        return new ArrayList<>();
      } else {
        List<Map<Variable,Symbol>> currentBinding = new ArrayList<>();
        currentBinding.add(binding);
        return currentBinding;
      }
    } else {
      List<Map<Variable,Symbol>> currentBinding = new ArrayList<>();
      currentBinding.add(new HashMap<>());
      return currentBinding;
    }
  }


  public List<Map<Variable,Symbol>> checkDialogueApplicability(List<Utterance> dialogHistory) {
    Map<Variable, Symbol> binding = new HashMap<>();
    if (!dialogContext.isEmpty() && dialogHistory.size() >dialogContext.size()) {
      for (Map<Integer,Utterance> dialog : dialogContext) {
        for (Integer i : dialog.keySet()) {
          if (i >= 0 && dialogHistory.size() - i >= 0) {
            Utterance u1 = dialog.get(i);
            Utterance u2 = dialogHistory.get(dialogHistory.size() - i - 1);
            binding = UtteranceUtil.getUtteranceBindings(binding, u1, u2);
          }
          //TODO:brad: was this supposed to do anything?
//        else if (i < 0) {
//          boolean foundApplicable = false;
//          for (int j = dialogHistory.size() - 1; j >= 0; j--) {
//            Utterance u1 = dialogContext.get(i);
//            Utterance u2 = dialogHistory.get(j);
//            Map<Variable, Symbol> tmpBinding = UtteranceUtil.getUtteranceBindings(binding, u1, u2);
//            if (tmpBinding != null) {
//              binding = tmpBinding;
//              break;
//            }
//          }
//          if (!foundApplicable) {
//            return false;
//          }
//        }
        }
      }

      if (binding==null || binding.isEmpty()) {
        return new ArrayList<>();
      } else {
        List<Map<Variable,Symbol>> currentBinding = new ArrayList<>();
        currentBinding.add(binding);
        return currentBinding;
      }
    } else {
      List<Map<Variable,Symbol>> currentBinding = new ArrayList<>();
      currentBinding.add(new HashMap<>());
      return currentBinding;
    }
  }

//  public ArrayList<PragRuleProver> findApplicableRules(Set<Term> propsToConvey) {
//    ArrayList<PragRuleProver> retList = new ArrayList<PragRuleProver>();
//    for (PragRuleProver pr : this) {
//      //log.debug("PragRuleSet.findApplicableRules().propsToConvey = " + propsToConvey);
//      ArrayList<Map<Variable, Symbol>> maps = pr.rhsApplicableMaps(propsToConvey);
//      for (Map<Variable, Symbol> map : maps) {
//        retList.add(pr.getBoundVersion(map));
//      }
//    }
//    return retList;
//  }
//
//  public ArrayList<PragRuleProver> findApplicableRules(Term propToConvey) {
//    Set<Term> singleton = new HashSet<Term>();
//    singleton.add(propToConvey);
//    return findApplicableRules(singleton);
//  }

  public ArrayList<PragRuleProver> findApplicableRules(Term propToConvey) {
    ArrayList<PragRuleProver> retList = new ArrayList<>();
    for (PragRuleProver pr : this) {
      ArrayList<Map<Variable, Symbol>> maps = pr.rhsApplicableMap(propToConvey);
      for (Map<Variable, Symbol> map : maps) {
        retList.add(pr.getBoundVersion(map));
      }
    }
    return retList;
  }

  public ArrayList<PragRuleProver> findApplicableRules(Set<Term> propsToConvey) {
    ArrayList<PragRuleProver> applicableRules = new ArrayList<>();
    for(Term t: propsToConvey){
      applicableRules.addAll(findApplicableRules(t));
    }
    return applicableRules;
  }

}
