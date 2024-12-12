package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;

public interface RLInterface {

  /**
   * Calls a policy created by RL.
   *
   * @param action Name associated with the policy
   * @return Success/failure justification
   */
  @TRADEService
  @Action
  public Justification callPolicy(String action);

  /**
   * Creates a new policy via RL and assigns it the given name
   *
   * @param action Name to be associated with the new policy
   * @return Success/failure justification
   */
  @TRADEService
  @Action
  public Justification learnPolicy(String action);

  /**
   * Updates
   *
   * @param failedOperator Name of the operator we want to update
   * @return Success/failure justification
   */
  @TRADEService
  @Action
  public Justification updatePolicy(String failedOperator);
}
