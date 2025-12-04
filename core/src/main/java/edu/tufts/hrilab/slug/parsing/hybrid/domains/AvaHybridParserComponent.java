package edu.tufts.hrilab.slug.parsing.hybrid.domains;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;

import java.util.ArrayList;
import java.util.List;

public class AvaHybridParserComponent extends HybridParserComponent {
  private List<Long> modifiedUtteranceUids = new ArrayList<>();

  @Override
  public Utterance parseUtterance(Utterance incoming) {
    //TODO: AvaDemo "hack" to prevent duplicate parsing when checking for false positive wakeword triggers
    if (incoming.getSemantics() != null) {
      log.info("[parseUtterance] receiving utterance with set semantics. Please unset before calling parseUtterance if desiring a new result");
      return incoming;
    }

    Utterance output = super.parseUtterance(incoming);
    output = applyContextToOutput(output);
    return output;
  }

  private Utterance applyContextToOutput(Utterance utterance) {
    if (utterance.getSemantics() == null) {
      return utterance;
    }
    Predicate semantics = (Predicate) utterance.getSemantics();

    //Copying Utterance so we don't modify what was put into the cache (if not using validation)
    Utterance output = new Utterance.Builder(utterance).build();

    //TODO: should this always be last question? Or should look for non-validation question specifically?
    //Get current question expected response forms
    List<Term> expectedResponseForms = new ArrayList<>();
    try {
      expectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestionResponseForm").argTypes(Symbol.class)).call(List.class, output.getAddressee());
    } catch (TRADEException e) {
      log.error("[applyContextToOutput] unable to get last question response form", e);
    }

    //If getting endActionLearning while a pending question exists seeking location(X), swap to location(none)
    if (semantics.getName().equals("endActionLearning")) {
      if (expectedResponseForms.contains(Factory.createPredicate("location","X"))) {
        output.setSemantics(Factory.createPredicate("location","none"));
        output.setType(UtteranceType.REPLY);
        modifiedUtteranceUids.add(output.getUid());
        return output;
      }
    }

    //If getting location(none) while action learning is present and no question for location(X) exists, swap to endActionLearning
    if (semantics.equals(Factory.createPredicate("location","none"))) {
      if (!expectedResponseForms.contains(Factory.createPredicate("location","X"))) {
        output.setSemantics(Factory.createPredicate("endActionLearning", output.getAddressee()));
        output.setType(UtteranceType.INSTRUCT);
        modifiedUtteranceUids.add(output.getUid());
        return output;
      }
    }

    //If a question exists asking for a taskPredicate, coerce direct action parses into the corresponding REPLY form
    // (relevant for: scheduled goal task, tour action along the path, kiosk action)
    //  Do not include SKIPPENDING goals
    if (!expectedResponseForms.isEmpty()) {
      if (expectedResponseForms.contains(Factory.createPredicate("taskPredicate","X"))) {
        if (!semantics.getName().equals("taskPredicate") && output.getType().equals(UtteranceType.INSTRUCT)) {
          try {
            PriorityTier priorityTier = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPriorityTierForGoal").argTypes(Predicate.class)).call(PriorityTier.class, semantics);
            if (priorityTier != PriorityTier.SKIPPENDING) {
              output.setSemantics(Factory.createPredicate("taskPredicate",semantics));
              output.setType(UtteranceType.REPLY);
              modifiedUtteranceUids.add(output.getUid());
              //TODO: Do we want to set needsValidation to true here?
              return output;
            }
          } catch (TRADEException e) {
            log.error("[applyContextToOutput] unable to get priority tier for output goal", e);
          }
        }
      }
    }

    return utterance;
  }

  @Override
  public void cacheParse(Utterance utterance) {
    // Don't cache entries whose output were modified due to context
    if (modifiedUtteranceUids.contains(utterance.getUid())) {
      return;
    }

    // If semantics are taskName, then just return. Do not cache
    if (utterance.getSemantics() != null && utterance.getSemantics().getName().equals("taskName")) {
      return;
    }

    super.cacheParse(utterance);
  }
}

