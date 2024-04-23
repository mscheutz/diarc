/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2;

import edu.tufts.hrilab.ros2.ros2jrosclient.*;
import id.jrosmessages.Message;

/**
 * Factory class to allow users to generate ROS 2 objects without needing to know a specific
 * underlying implementation. The main purpose of this class is so we can implement different
 * ROS2 Java implementations without changing classes that use ROS2
 */
public class Ros2Factory {

  /**
   * Returns an instance of a Ros2 Node with the default implementation
   *
   * @return Generic Ros2 Node
   */
  public static Ros2Node getDefaultNode() {
    return new JRosClientNode();
  }

  /**
   * @param domainId Domain ID to create the node in
   * @return Generic Ros2 Node
   * @see #getDefaultNode()
   */
  public static Ros2Node getDefaultNode(int domainId) {
    return new JRosClientNode(domainId);
  }

  /**
   * Returns an instance of a Ros2 publisher with the default implementation
   *
   * @param messageType Type of the message this publisher will publish, e.g. StringMessage
   * @param topicName   Topic on which to publish
   * @param <T>         Message type
   * @return Generic ROS2 Publisher
   */
  public static <T extends Message> Ros2Publisher<T> getDefaultPublisher(Class<T> messageType,
                                                                         String topicName) {
    return new JRosClientPublisher<>(messageType, topicName);
  }

  /**
   * Returns an instance of a Ros2 Node with the default implementation, with a pre-initialized
   * message.
   *
   * @param messageType Type of the message this publisher will publish, e.g. StringMessage
   * @param topicName   Topic on which to publish
   * @param msg         Initialized message to publish
   * @param <T>         Message type
   * @return Generic ROS2 Publisher
   */
  public static <T extends Message> Ros2Publisher<T> getDefaultPublisher(Class<T> messageType,
                                                                         String topicName, T msg) {
    return new JRosClientPublisher<>(messageType, topicName, msg);
  }

  /**
   * @param messageType Type of the message this publisher will publish, e.g. StringMessage
   * @param topic       Topic on which to publish
   * @param callback    Function to execute on received message
   * @param <T>         Message type
   * @return Generic ROS2 Subscriber
   */
  public static <T extends Message> Ros2Subscriber<T> getDefaultSubscriber(Class<T> messageType,
                                                                           String topic,
                                                                           SubscriberCallback<T> callback) {
    return new JRosClientSubscriber<T>(messageType, topic, callback);
  }
}
