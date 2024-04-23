/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2;

@FunctionalInterface
public interface SubscriberCallback<T> {
  /**
   * Every time the subscriber gets a message, execute the callback on it
   * @param t
   */
  void callback(T t);
}
