/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.justification.Justification;

public class DefaultRecovery extends Recovery {

    @Override
    public void updateRecoveryStatus(Context context) {
    }

    @Override
    public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, Context context) {
        return false;
    }

    @Override
    public ParameterizedAction selectRecoveryPolicy(Context context) {
        return null;
    }

}
