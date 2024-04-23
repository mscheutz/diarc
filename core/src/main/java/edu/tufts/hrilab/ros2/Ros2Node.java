/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2;

import id.jrosmessages.Message;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import pinorobotics.jros2actionlib.JRos2ActionClient;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2Definition;
import pinorobotics.jros2services.JRos2ServiceClient;
import pinorobotics.jrosservices.msgs.ServiceDefinition;

import javax.vecmath.Matrix4d;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Todo: Remove service dependencies
 */
public abstract class Ros2Node {

  protected static Logger log = LoggerFactory.getLogger(Ros2Node.class);

  /**
   * Thread manager. TODO: Figure out thread pool size. Also see if we can incorporate cache.
   */
  private final ScheduledExecutorService asyncExecutor = Executors.newScheduledThreadPool(8);

  /**
   * This is started from the thread pool, and is used to shut down the node.
   */
  private Future<?> nodeFuture;

  /**
   * Tracks all the publishers on this node, and their remaining iterations.
   * Value is -1 for indefinite iterations.
   */
  protected Map<Ros2Publisher<?>, Integer> publisherList = new HashMap<>();

  /**
   * Tracks all the services on this node, and their associated names
   * //TODO: Make generic
   */
  protected Map<String, JRos2ServiceClient<?, ?>> serviceList = new HashMap<>();


  /**
   * Tracks all the services on this node, and their associated names
   * //TODO: Make generic
   */
  protected Map<String, JRos2ActionClient<?, ?>> actionList = new HashMap<>();


  /**
   * Spins the node up at the default rate of 1.
   *
   * @see #spin(Integer)
   */
  public void spin() {
    spin(1);
  }

  /**
   * Spins the node up at the specified rate. All publishers publish at this
   * rate.
   *
   * @param rate Rate at which publishers are published.
   */
  public void spin(Integer rate) {
    nodeFuture = asyncExecutor.scheduleAtFixedRate(() -> {
      for (Map.Entry<Ros2Publisher<?>, Integer> entry : publisherList.entrySet()) {
        if (entry.getValue() > 0) { // Run for a set amount of iterations
          entry.getKey().publish();
          entry.setValue(entry.getValue() - 1);
        } else {
          entry.getKey().publish();
        }
      }
    }, 0, rate, TimeUnit.MILLISECONDS);
  }

  /**
   * Spins down the node. All publishers will stop publishing.
   */
  public void end() {
    nodeFuture.cancel(true);
  }

  /**
   * Adds the specified subscriber to the node. Immediately starts listening.
   *
   * @param sub Subscriber to add to the node.
   */
  public abstract void addAndStartSubscriber(Ros2Subscriber<?> sub);

  /**
   * Removes the specified subscriber from the node. TODO: Currently no
   * jrosclient functionality for this.
   *
   * @param sub Subscriber to remove from the node.
   */
  public abstract void removeSubscriber(Ros2Subscriber<?> sub);

  /**
   * Adds the specified publisher to the node. Does not start publishing
   * until the publisher is started.
   *
   * @param pub Publisher to add to the node.
   */
  public abstract void addPublisher(Ros2Publisher<?> pub);

  /**
   * Starts publishing the specified publisher indefinitely (assuming node is
   * spinning). The node must have been added to the node.
   *
   * @param pub Existing publisher to be started
   */
  public abstract void startPublisher(Ros2Publisher<?> pub);

  /**
   * Starts publishing the specified publisher for the given iterations
   * (assuming node is pinning). The node must have been added to the node.
   *
   * @param pub   Existing publisher to be started
   * @param iters Iterations the publisher publishes for
   */
  public abstract void startPublisher(Ros2Publisher<?> pub, Integer iters);

  /**
   * Convenience method to publish a message one time for a given publisher.
   * Removed after use.
   *
   * @param pub Publisher to publish once
   */
  public abstract void oneTimePublish(Ros2Publisher<?> pub);

  /**
   * Convenience method to add and start a publisher indefinitely.
   *
   * @param pub Publisher to be added and started
   * @see #addPublisher(Ros2Publisher)
   * @see #startPublisher(Ros2Publisher)
   */
  public abstract void addAndStartPublisher(Ros2Publisher<?> pub);

  /**
   * Conveinece method to add and start a publisher for the given number of
   * iterations.
   *
   * @param pub   Publisher to be added and started
   * @param iters Number of iterations to publish for
   * @see #addPublisher(Ros2Publisher)
   * @see #startPublisher(Ros2Publisher, Integer)
   */
  public abstract void addAndStartPublisher(Ros2Publisher<?> pub, Integer iters);

  /**
   * Stops the given publisher from publishing. Can be started again.
   *
   * @param pub Publisher to be stopped.
   */
  public abstract void stopPublisher(Ros2Publisher<?> pub);

  /**
   * Stops and removes the publisher from the node. must be re-added to start
   * again.
   *
   * @param pub Publisher to be stopped and removed
   */
  public abstract void removePublisher(Ros2Publisher<?> pub);

  /**
   * Creates a service... This is definition is a nightmare. Should we just let users make this
   * manually?
   *
   * @param definition
   * @param <REQ>
   * @param <RES>
   * @param <DEF>
   */
  public abstract <REQ extends Message, RES extends Message, DEF extends ServiceDefinition<REQ,
      RES>> void addService(DEF definition, String serviceName);

  public abstract <REQ extends Message, RES extends Message> RES callService(String serviceName,
                                                                             REQ message);

  // TODO: Discuss if these should be native (but still optional) node functions, or moved elsewhere
  public abstract void createTf();

  public abstract Matrix4d lookupTransform(String from, String to);



  public abstract <GOAL extends Message, RES extends Message, DEF extends Action2Definition<GOAL,
      RES>> void addAction(DEF def, String actionName);

  public abstract <GOAL extends Message, RES extends Message> RES callAction(String actionName,
                                                                             GOAL goal);

}
