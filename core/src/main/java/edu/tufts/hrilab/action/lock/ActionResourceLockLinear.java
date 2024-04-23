/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action.lock;

import edu.tufts.hrilab.action.ActionInterpreter;

/**
 * <code>ActionResourceLockLinear</code> implements FCFS locks--no preemption
 * based on priority.
 */
public class ActionResourceLockLinear extends ActionResourceLock {

  /**
   * Make <code>count</code> attempts to acquire this resource lock.
   *
   * @param actionInt the ActionInterpreter requesting the lock
   * @param count the maximum number of attempts to acquire the lock
   * @return true if the lock is successfully acquired, false otherwise
   */
  @Override
  public boolean acquire(ActionInterpreter actionInt, int count) {
    int i = 0;
    String cmd = actionInt.getGoal().toString();
    boolean retval = false;

    // This is stupid, but I might as well preserve the semantics...
    if (count == 0) {
      return false;
    }

    lock.lock();

    try {
      while (true) {
                // PWS: Should we allow AIs access if their spawners have the
        // lock?
        // TODO: priority checking.
        // Right now this just checks to see if the lock is free or I
        // already have it; need to check if my priority is high
        // enough to override and take the lock away.
        if (owner.empty() || (owner.peek() == actionInt)) {
          owner.push(actionInt);
          //System.out.println(cmd + " acquired " + lockName);
          retval = true;
          break;
        } else if (++i == count) {
          break;
        }
        try {
          //System.out.println(cmd + " awaiting " + lockName);
          released.await();
        } catch (InterruptedException ie) {
          break;
        }
      }
    } finally {
      lock.unlock();
    }
    return retval;
  }

  /**
   * Constructor for a resource lock.
   *
   * @param name the lock's name
   */
  public ActionResourceLockLinear(String name) {
    super(name);
  }

  public static void initResources() {
    ActionResourceLock.faceLock = new ActionResourceLockLinear("faceLock");
    resources.add(faceLock);
    ActionResourceLock.blinkLock = new ActionResourceLockLinear("blinkLock");
    resources.add(blinkLock);
    ActionResourceLock.headLock = new ActionResourceLockLinear("headLock");
    resources.add(headLock);
    ActionResourceLock.motionLock = new ActionResourceLockLinear("motionLock");
    resources.add(motionLock);
    ActionResourceLock.transmitLock = new ActionResourceLockLinear("transmitLock");
    resources.add(transmitLock);
    ActionResourceLock.visionLock = new ActionResourceLockLinear("visionLock");
    resources.add(visionLock);
    ActionResourceLock.speechLock = new ActionResourceLockLinear("speechLock");
    resources.add(speechLock);
    ActionResourceLock.lightLock = new ActionResourceLockLinear("lightLock");
    resources.add(lightLock);
  }
}
