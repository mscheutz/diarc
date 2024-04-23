/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

import edu.tufts.hrilab.supermarket.GameAction;

public class Cancel extends Active {
    public Cancel() { this.maxResponseWait = 1000; }

    @Override
    public String getCommand() {
        return "CANCEL";
    }
}

