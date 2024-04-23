/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;


/**
 * A thread-safe way to assign unique IDs
 * @author evankrause
 */

public class IdGenerator {
    private volatile long id = 0;

    public synchronized long getNext() {
        return id++;
    }
}
