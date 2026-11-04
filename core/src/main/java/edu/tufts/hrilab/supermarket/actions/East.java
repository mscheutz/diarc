/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

public class East extends Active {
    public East() { this.maxResponseWait = 1000; }

    @Override
    public String getCommand() {
        return "EAST";
    }
}
