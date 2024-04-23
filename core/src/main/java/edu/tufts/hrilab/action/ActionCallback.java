/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import java.io.Serializable;

public class ActionCallback implements Runnable,/* Remote,*/ Serializable {
    @Override
    public void run() {
        System.out.println("Running runme.");
    }
}
