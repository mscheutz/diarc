/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.recovery.FailureInformation;
import edu.tufts.hrilab.action.recovery.PolicyConstraints;
import edu.tufts.hrilab.action.recovery.PostRecoveryExecutionType;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public class RecoveryPolicyDatabase {
  private final static Logger log = LoggerFactory.getLogger(RecoveryPolicyDatabase.class);
  /**
   * A map of all recovery policies hashed by policy constraints.
   */
  private final Map<PolicyConstraints, List<ActionDBEntry>> recoveryPolicyDB = new HashMap<>();

  /**
   * Protected so that only the Database can instantiate.
   */
  protected RecoveryPolicyDatabase() {
  }

  /**
   * Add a recovery policy to the database. This should only be called
   * internally when an ActionDBEntry is added.
   *
   * @param action the recovery policy
   */
  protected synchronized void addRecoveryPolicyToDB(ActionDBEntry action) {
    if (action.isRecoveryPolicy()) {
      List<Class<?>> argTypes = action.getRequiredInputRolesTypes();
      List<Class<?>> returnTypes = action.getReturnRolesTypes();
      // recoverySignature(Predicate failedActionPredicate, List<Predicate> failureReasons, Predicate failedGoal)
      if (argTypes.size() == 3 && (returnTypes.size() == 1 || returnTypes.isEmpty())
              && List.class.isAssignableFrom(argTypes.get(0))
              && List.class.isAssignableFrom(argTypes.get(1))
              && Predicate.class.isAssignableFrom(argTypes.get(2))) {

        // optional return value of PostRecoveryExecutionType
        if (returnTypes.size() == 1 && !PostRecoveryExecutionType.class.isAssignableFrom(returnTypes.get(0))) {
          log.warn("The action '" + action.getName() + "' cannot be a recovery policy as the return value is not a PostRecoveryExecutionType. "
                  + " The action will not be available as a recovery policy.");
          return;
        }

        for (PolicyConstraints constraints : action.getPolicyConstraints()) {
          if (constraints.alwaysApplicable() && constraints.recoveryType.isAction() && constraints.postRecoveryExecutionType == PostRecoveryExecutionType.CONTINUE) {
            log.warn("The recovery specification for action {} is not permitted (empty constraints with action recovery and CONTINUE post-recovery execution type). " +
                    "The recovery policy will not be available.", action.getName());
            continue;
          }

          log.debug("Adding recovery policy to DB: " + action);

          List<ActionDBEntry> policies = recoveryPolicyDB.get(constraints);

          if (policies == null) {
            // this policy hasn't been seen before, add it so this action can be looked up
            policies = new ArrayList<>();
            policies.add(action);
            recoveryPolicyDB.put(constraints, policies);
          } else {
            // this observation has been seen before, add DBEntry to the list if it is not already present
            if (!policies.contains(action)) {
              policies.add(action);
            }
          }
        }
      } else {
        log.warn("The action '" + action.getName() + "' cannot be a recovery policy as it is not of the form: "
                + "() = " + action.getName() + "(List failedActionPredicates, List failureReasons, Predicate failedGoal)."
                + " The action will not be available as a recovery policy.");
      }
    }
  }

  /**
   * Remove an action's observations from the database. Should only be used
   * internally.
   *
   * @param entry the action for which the observations should be removed
   */
  protected synchronized void removeRecoveryPolicyFromDB(ActionDBEntry entry) {
    if (!entry.isRecoveryPolicy()) {
      log.warn("[removeRecoveryPolicyFromDB] supplied entry {} is not a recovery policy, not removing", entry);
      return;
    }

    log.debug("Removing recovery policy from DB: " + entry);
    for (PolicyConstraints constraints : entry.getPolicyConstraints()) {
      List<ActionDBEntry> policies = recoveryPolicyDB.get(constraints);
      if (policies != null && !policies.isEmpty()) {
        policies.remove(entry);
      }
    }
  }

  /**
   * Get all Recovery Policies (i.e., actions) that can be for a particular action failure.
   * TODO: these probably need to be filtered per ?actor
   *
   * @param failureInformation
   * @return
   */
  public final synchronized List<ActionDBEntry> getRecoveryPolicies(FailureInformation failureInformation) {
    return getRecoveryPolicies(getMatchingPolicyConstraints(failureInformation));
  }

  public final synchronized List<ActionDBEntry> getRecoveryPolicies(List<PolicyConstraints> orderedConstraintsList) {
    List<ActionDBEntry> applicablePolicies = new ArrayList<>();
    orderedConstraintsList.forEach(constraints -> applicablePolicies.addAll(recoveryPolicyDB.get(constraints)));
    return applicablePolicies;
  }

  public final synchronized List<PolicyConstraints> getMatchingPolicyConstraints(FailureInformation failureInformation) {
    Map<PolicyConstraints,Integer> applicableConstraintsScores = new HashMap<>();

    recoveryPolicyDB.forEach((constraints, policies) -> {
      int score = failureInformation.getNumberOfConstraintMatches(constraints);
      if (score >= 0) {
        applicableConstraintsScores.put(constraints, score);
      }
    });

    // sort applicable policies by score (i.e., number of matching constraints)
    Stream<Map.Entry<PolicyConstraints,Integer>> sorted =
            applicableConstraintsScores.entrySet().stream()
                    .sorted(Collections.reverseOrder(Map.Entry.comparingByValue()));

    List<PolicyConstraints> applicableConstraints = new ArrayList<>();
    sorted.forEach(entry -> applicableConstraints.add(entry.getKey()));

    //Cannot have RETRY post recovery if no specific child action at this level failed (possible in the case of FAIL_OVERALLCONDITIONS, FAIL_POSTCONDITIONS)
    //Is practically also an issue with FAIL_NOTFOUND because there is no current functionality to reattempt to parse the previous eventSpec.
    // Q: should this ^ case be allowed? (recovery when an action isn't known/its service isn't available)
    if (failureInformation.contextType.isAction() && failureInformation.failedActions.isEmpty()) {
      applicableConstraints.removeIf(e -> e.postRecoveryExecutionType == PostRecoveryExecutionType.RETRY);
    }

    return applicableConstraints;
  }

}
