/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient;

import edu.tufts.hrilab.ros2.Ros2Subscriber;
import edu.tufts.hrilab.ros2.SubscriberCallback;
import id.jrosclient.TopicSubscriber;
import id.jrosmessages.Message;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Wrapper on JROSClient's TopicSubscriber. Both for convenience and interactions with
 * the BaseNode class. May eventually be extracted to an interface for different ros2 java
 * implementations.
 *
 * @param <T>
 * @author Marlow Fawn
 * @see JRosClientNode
 */
public class JRosClientSubscriber<T extends Message> implements Ros2Subscriber<T> {
  protected static Logger log = LoggerFactory.getLogger(JRosClientSubscriber.class);
  public TopicSubscriber<T> subscriber;


  public JRosClientSubscriber(Class<T> messageType, String topic, SubscriberCallback<T> callback) {

    subscriber = new TopicSubscriber<>(messageType, topic) {
      @Override
      public void onNext(T t) {
        callback.callback(t);
        getSubscription().get().request(1);
      }

      @Override
      public void onError(Throwable throwable) {
        log.error("Error in subscriber", throwable);
      }
    };
  }
}
