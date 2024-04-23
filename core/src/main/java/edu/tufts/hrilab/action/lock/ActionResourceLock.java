/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.lock;

import edu.tufts.hrilab.action.ActionInterpreter;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.*;
import java.util.Stack;
import java.io.Serializable;


/**
 * <code>ActionResourceLock</code> is the base class for Action's resource
 * locks.
 */
public abstract class ActionResourceLock implements Serializable {
    public String lockName;
    public Stack<ActionInterpreter> owner;
    final Lock lock = new ReentrantLock();
    final Condition released = lock.newCondition();
    
    static List<ActionResourceLock> resources = new ArrayList<>();
    static public ActionResourceLock faceLock; // facial expression
    static public ActionResourceLock blinkLock; // eyelid 
    static public ActionResourceLock headLock; // head and eye pose
    static public ActionResourceLock motionLock;
    static public ActionResourceLock transmitLock;
    static public ActionResourceLock visionLock;
    static public ActionResourceLock speechLock;
    static public ActionResourceLock lightLock;
    
    /**
     * Get a resource lock.
     *
     * @param lockName the name of the lock to get
     * @return the lock
     */
    static public ActionResourceLock getLock(String lockName) {
        if (lockName.equalsIgnoreCase("motionLock")) {
            return motionLock;
        } else if (lockName.equalsIgnoreCase("transmitLock")) {
            return transmitLock;
        } else if (lockName.equalsIgnoreCase("visionLock")) {
            return visionLock;
        } else if (lockName.equalsIgnoreCase("speechLock")) {
            return speechLock;
        } else if (lockName.equalsIgnoreCase("lightLock")) {
            return lightLock;
        } else if (lockName.equalsIgnoreCase("blinkLock")) {
            return blinkLock;
        } else if (lockName.equalsIgnoreCase("headLock")) {
            return headLock;
        } else if (lockName.equalsIgnoreCase("faceLock")) {
            return faceLock;
        } else {
            return null;
        }
    }

    /**
     * Must be implemented by subclass.
     **/
    abstract public boolean acquire(ActionInterpreter actionInt, int count);
    
    public String getLockName() {
      return lockName;
    }

    /**
     * Acquire the resource lock, waiting indefinitely for it to become
     * available.
     * @param actionInt the ActionInterpreter requesting the lock
     * @return true if the lock is acquired
     */
    public boolean blockingAcquire(ActionInterpreter actionInt) {
        return acquire(actionInt, -1);
    }

    /**
     * Attempt to acquire the resource lock, but do not block if it is
     * unavailable.
     * @param actionInt the ActionInterpreter requesting the lock
     * @return true if the lock is acquired, false otherwise
     */
    public boolean nonBlockingAcquire(ActionInterpreter actionInt) {
        String cmd = actionInt.getGoal().toString();
        boolean retval = acquire(actionInt, 1);

        return retval;
    }

    /**
     * Release the resource lock, ensuring that the current AI is the
     * current holder.  This version releases only one level of recursive
     * acquire; if an enclosing script also acquired the lock, actionInt will
     * still hold it.
     * @param actionInt the ActionInterpreter releasing the lock
     */
    public void release(ActionInterpreter actionInt) {
        Stack<ActionInterpreter> uncover = new Stack<ActionInterpreter>();
        String cmd = actionInt.getGoal().toString();
        lock.lock();

        try {
            if (!owner.empty() && (owner.peek() == actionInt)) {
                owner.pop();
                if(owner.empty()) {
                    //System.out.println(cmd + " released " + lockName);
                    released.signal();
                } else {
                    //System.out.println(cmd + " popped " + lockName);
                }
            } else if (!owner.empty()) {
                uncover = new Stack<>();
                while (!owner.empty() && (owner.peek() != actionInt)) {
                    uncover.push(owner.pop());
                }
                if (!owner.empty()) {
                    //System.out.println(cmd + " must've had " + lockName +
                    //	    " preempted");
                    owner.pop();
                }
                while (!uncover.empty()) {
                    owner.push(uncover.pop());
                }
            } else {
                //System.out.println(cmd + " must've had " + lockName +
                //	" preempted, but nobody owns it now!");
            }
        } finally {
            lock.unlock();
        }
    }

    /**
     * Release the resource lock, ensuring that the current AI is the
     * current holder.  This version ensures the lock will be released,
     * regardless of how many recursive acquires the owner has made.
     * @param actionInt the ActionInterpreter releasing the lock
     */
    public void deepRelease(ActionInterpreter actionInt) {
        lock.lock();

        try {
            if (!owner.empty() && (owner.peek() == actionInt)) {
                while (!owner.empty())
                    owner.pop();
                //System.out.println(cmd + " (deeply) releasing " + lockName);
                released.signal();
            } else {
            }
        } finally {
            lock.unlock();
        }
    }

    /**
     * Get the interpreter currently holding the lock.
     * @return the owner
     */
    public ActionInterpreter getOwner() {
        ActionInterpreter o = null;
        if (!owner.empty()) {
            o = owner.peek();
        }
        return o;
    }

    @Override
    public String toString() {
        String o = "unowned";
        if (!owner.isEmpty()) {
            o = owner.peek().getGoal().toString();
        }
        return lockName + " (" + o + ")";
    }

    /**
     * Constructor for a resource lock.
     * @param name the lock's name
     */
    public ActionResourceLock(String name) {
        lockName = name;
        owner = new Stack<ActionInterpreter>();
    }
    
    public static void deepReleaseAll(ActionInterpreter ai) {
      for (ActionResourceLock r : resources) {
        r.deepRelease(ai);
      }
    }
}
