/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

public class Toggle extends Active {
    public Toggle() { this.maxResponseWait = 1000; }

    @Override
    public String getCommand() {
        return "TOGGLE_CART";
    }
}
