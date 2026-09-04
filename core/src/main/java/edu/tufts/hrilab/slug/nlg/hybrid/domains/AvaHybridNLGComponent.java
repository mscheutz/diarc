package edu.tufts.hrilab.slug.nlg.hybrid.domains;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent;

import java.util.List;

public class AvaHybridNLGComponent extends HybridNLGComponent {
  @Override
  public void cacheNLGOutput(Utterance utterance) {
    if (utterance.getSemantics() != null) {
      String command = utterance.getSemantics().getName();
      if (command.equals("mean")) {
        //TODO: revisit - seems to give context dependent validation here which is bad for caching
        Symbol semantics = utterance.getSemantics();
        if (semantics.isPredicate()) {
          Predicate pred = (Predicate) semantics;
          List<Symbol> args = pred.getArgs();
          if (args.size() > 1) {
           String name = args.get(1).getName();
           if (name.equals("modifyExistingTaskStatus") || name.equals("learnTaskSequence") || name.equals("scheduleTask") || name.equals("scheduled") || name.equals("submitTaskSequenceDefinition")) {
             //Know this is definitely desired for modifyExistingTaskStatus, is this similarly desired for all focused contexts?
             return;
           }
          }
        }
      }
    }
    super.cacheNLGOutput(utterance);
  }
}
