/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

public class Nop extends Active {
    public Nop() { this.maxResponseWait = 1000; }

    @Override
    public String getCommand() {
        return "NOP";
    }
}
