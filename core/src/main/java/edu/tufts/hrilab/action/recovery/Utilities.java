/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Utilities {
  private static Logger log = LoggerFactory.getLogger(Utilities.class);

  /**
   * Bring Belief up to date with state of the world after novelty detection by
   * retracting incorrect beliefs and asserting correct beliefs.
   *
   * NOTE: this only needs to happen for discrepancy preds, as observations automatically update belief, but
   * discrepancy(X,Y) observations don't explicitly retract X and assert Y.
   * @param novelties
   */
  static public void correctBeliefState(Set<Predicate> novelties) {
    Set<Term> assertions = new HashSet<>();
    Set<Term> retractions = new HashSet<>();
    for (Predicate novelty : novelties) {
      if (novelty.getName().equals("discrepancy")) {
        Term retraction = (Term) novelty.get(0);
        if (retraction.isNegated()) {
          // ignore -- don't need to retract negations
        } else {
          retractions.add(retraction);
        }

        Term assertion = (Term) novelty.get(1);
        if (assertion.isNegated()) {
          // actually an un-negated retraction
          retractions.add(assertion.toUnnegatedForm());
        } else {
          assertions.add(assertion);
        }
      }
    }

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBeliefs").argTypes(Set.class)).call(void.class, retractions);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class,MemoryLevel.class)).call(void.class, assertions, MemoryLevel.EPISODIC);
    } catch (TRADEException e) {
      log.error("Error trying to bring Belief up to date after novelty detection.",e);
    }
  }

  /**
   * Get ActionDBEntry from action signature.
   * @param actionSignature
   * @return
   */
  public static ActionDBEntry getActionBySignature(Predicate actionSignature) {
    // get broken action db entry
    ActionDBEntry brokenAction = null;
    List<ActionDBEntry> brokenActionOptions = null;
    try {
      brokenActionOptions = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActionsBySignature").argTypes(Predicate.class)).call(List.class, actionSignature);
    } catch (TRADEException e) {
      log.error("Error calling getActionsBySignature for: " + actionSignature, e);
    }
    if (brokenActionOptions.size() > 1) {
      log.warn("More than one option found for action: " + actionSignature);
      brokenAction = brokenActionOptions.get(0);
    } else if (!brokenActionOptions.isEmpty()) {
      brokenAction = brokenActionOptions.get(0);
    }

    return brokenAction;
  }
}
