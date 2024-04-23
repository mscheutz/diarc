/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.lock;

/**
 *
 * @author willie
 */
public class ResourceUnavailableException extends RuntimeException {
    
    public ResourceUnavailableException(String lock, String cmd) {
        super("Lock for " + lock + " is currently owned by " + cmd);
    }
}
