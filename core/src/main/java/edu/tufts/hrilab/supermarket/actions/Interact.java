/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

public class Interact extends Active {
    public Interact() { this.maxResponseWait = 1000; }

    @Override
    public String getCommand() {
        return "INTERACT";
    }
}
