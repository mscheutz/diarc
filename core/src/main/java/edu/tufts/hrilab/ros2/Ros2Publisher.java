/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2;

/**
 * Generic Ros2 Publisher, which takes in a specific message type
 *
 * @param <T>
 */
public interface Ros2Publisher<T> {
  /**
   * Body of the publisher that is submitted every iteration.
   */
  void publish();

  /**
   *
   * @return Returns the message object internal to this publisher.
   * This object is mutable, and can be altered to change the published message.
   */
  T getMessage();

//  void setMessageData(T message);
}
