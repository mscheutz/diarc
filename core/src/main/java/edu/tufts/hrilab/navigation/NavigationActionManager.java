/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.navigation;

import edu.tufts.hrilab.action.ActionStatus;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 *
 * @author evankrause
 */

public class NavigationActionManager {

        //main executor
        private ExecutorService executor = Executors.newSingleThreadExecutor();
        private Future runFuture;
        private NavigationAction currAction;

        /**
         * Submit new navigation action. Automatically cancels any existing action,
         * and executes new action.
         * 
         * @param newAction 
         */
        public synchronized void initiateAction(NavigationAction newAction) {
            //cancel current action
            if (currAction != null) {
                currAction.cancel();
		runFuture.cancel(false);
            }
            currAction = newAction;
            runFuture = executor.submit(currAction);
        }

        /**
         * Get status of currently running action. Returns UNKNOWN if there is no
         * current action.
         * @param aid - action ID
         * @return ActionStatus
         */
        public synchronized ActionStatus getStatus(long aid) {
            if (currAction != null && currAction.getID() == aid) {
                return currAction.getStatus();
            } else {
                return ActionStatus.UNKNOWN;
            }
        }

        /**
         * Cancel currently running action, if one exists.
         */
        public synchronized void cancel() {
            if (currAction != null) {
                currAction.cancel();
            }
        }

        /**
         * Cancel action with specified ID.
         * @param aid - action ID
         * @return - if currently executing action has specified ID and was canceled.
         */
        public synchronized boolean cancel(long aid) {
            if ((currAction != null) && (currAction.getID() == aid)) {
                return currAction.cancel();
            } else {
                return false;
            }
        }
    
}
