/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient;

import edu.tufts.hrilab.ros2.Ros2Publisher;
import id.jrosclient.TopicSubmissionPublisher;
import id.jrosmessages.Message;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.InvocationTargetException;

/**
 * Wrapper on JROSClient's TopicSubmissionPublisher. Both for convenience and interactions with
 * the BaseNode class. May eventually be extracted to an interface for different ros2 java
 * implementations.
 *
 * @param <T> message type
 * @author Marlow Fawn
 * @see JRosClientNode
 */
public class JRosClientPublisher<T extends Message> implements Ros2Publisher<T> {
  protected static Logger log = LoggerFactory.getLogger(JRosClientPublisher.class);
  public TopicSubmissionPublisher<T> publisher;
  private T message;

  public JRosClientPublisher(Class<T> messageType, String topic) {
    publisher = new TopicSubmissionPublisher<>(messageType, topic);
    try {
      this.message = messageType.getDeclaredConstructor().newInstance();
    } catch (InstantiationException | IllegalAccessException | InvocationTargetException |
             NoSuchMethodException e) {
      throw new RuntimeException(e);
    }
  }

  public JRosClientPublisher(Class<T> messageType, String topic, T message) {
    publisher = new TopicSubmissionPublisher<>(messageType, topic);
    this.message = message;
  }

  @Override
  public void publish() {
    publisher.submit(message);
  }

  @Override
  public T getMessage() {
    return message;
  }

//  @Override
//  public void setMessageData(T message) {
//    this.message = message;
//  }

}
