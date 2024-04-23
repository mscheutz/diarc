/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

public class North extends Active {

    public North() { this.maxResponseWait = 1000; }

    @Override
    public String getCommand() {
        return "NORTH";
    }
}
