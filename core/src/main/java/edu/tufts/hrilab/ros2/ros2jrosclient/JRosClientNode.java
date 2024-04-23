/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient;

import edu.tufts.hrilab.ros2.Ros2Node;
import edu.tufts.hrilab.ros2.Ros2Publisher;
import edu.tufts.hrilab.ros2.Ros2Subscriber;
import id.jros2client.JRos2Client;
import id.jros2client.JRos2ClientConfiguration;
import id.jros2client.JRos2ClientFactory;
import id.jrosmessages.Message;
import id.jrosmessages.geometry_msgs.QuaternionMessage;
import id.jrosmessages.geometry_msgs.Vector3Message;
import pinorobotics.jros2actionlib.JRos2ActionClient;
import pinorobotics.jros2actionlib.JRos2ActionClientFactory;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2Definition;
import pinorobotics.jros2services.JRos2ServiceClient;
import pinorobotics.jros2services.JRos2ServiceClientFactory;
import pinorobotics.jros2tf2.JRos2Tf2;
import pinorobotics.jros2tf2.JRos2Tf2Factory;
import pinorobotics.jros2tf2.tf2_msgs.LookupTransformResultMessage;
import pinorobotics.jrosservices.msgs.ServiceDefinition;
import pinorobotics.jrostf2.exceptions.JRosTf2Exception;
import pinorobotics.rtpstalk.RtpsTalkConfiguration;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.concurrent.*;

/**
 * Generic ROS2 node implementation using JROSClient. This may eventually be extracted to an
 * interface so we can swap out different ros2 java implementations.
 *
 * @author Marlow Fawn
 */
public class JRosClientNode extends Ros2Node {
  /**
   * Pub/sub manager
   */
  protected JRos2Client client;
  public JRos2Tf2 tf;

  public JRosClientNode() {
    client = new JRos2ClientFactory().createClient();
  }

  public JRosClientNode(int domain_id) {
    RtpsTalkConfiguration.Builder builder = new RtpsTalkConfiguration.Builder();
    builder.domainId(domain_id);
    client = new JRos2ClientFactory().createClient(new JRos2ClientConfiguration(builder.build()));
  }

  @Override
  public void addAndStartSubscriber(Ros2Subscriber<?> sub) {
    JRosClientSubscriber<?> subscriber = (JRosClientSubscriber<?>) sub;
    client.subscribe(subscriber.subscriber);
  }

  @Override
  public void end() {
    super.end();
    client.close();
  }

  @Override
  public void removeSubscriber(Ros2Subscriber<?> sub) {
    log.warn("removeSubscriber not yet functional!");
  }

  @Override
  public void addPublisher(Ros2Publisher<?> pub) {
    publisherList.put(pub, 0);
  }

  @Override
  public void startPublisher(Ros2Publisher<?> pub) {
    Integer iterations = publisherList.get(pub);
    if (iterations == null) {
      log.warn("Publisher not started because it has not yet been added to the node. Did you " +
              "mean" + " to call [addAndStartPublisher]?");
    } else {
      publisherList.replace(pub, -1); // Publishers with a value of -1 will
      // run indefinitely.
      //todo: This can probably be changed with generics.
      JRosClientPublisher<?> publisher = (JRosClientPublisher<?>) pub;
      client.publish(publisher.publisher);
    }
  }

  @Override
  public void startPublisher(Ros2Publisher<?> pub, Integer iters) {
    publisherList.replace(pub, iters);
    JRosClientPublisher<?> publisher = (JRosClientPublisher<?>) pub;
    client.publish(publisher.publisher);
  }

  @Override
  public void oneTimePublish(Ros2Publisher<?> pub) {
    addAndStartPublisher(pub, 1);
    removePublisher(pub);
  }

  @Override
  public void addAndStartPublisher(Ros2Publisher<?> pub) {
    addPublisher(pub);
    startPublisher(pub);
  }

  @Override
  public void addAndStartPublisher(Ros2Publisher<?> pub, Integer iters) {
    addPublisher(pub);
    startPublisher(pub, iters);
  }

  @Override
  public void stopPublisher(Ros2Publisher<?> pub) {
    publisherList.replace(pub, 0);
  }

  @Override
  public void removePublisher(Ros2Publisher<?> pub) {
    stopPublisher(pub);
    JRosClientPublisher<?> publisher = (JRosClientPublisher<?>) pub;
    client.unpublish(publisher.publisher);
    publisherList.remove(pub);
  }

  @Override
  public <REQ extends Message, RES extends Message, DEF extends ServiceDefinition<REQ, RES>> void addService(DEF definition, String serviceName) {
    JRos2ServiceClient<REQ, RES> serviceClient = new JRos2ServiceClientFactory().createClient(client, definition, serviceName);

    serviceList.put(serviceName, serviceClient);
  }

  @Override
  public <REQ extends Message, RES extends Message> RES callService(String serviceName, REQ message) {
    JRos2ServiceClient<REQ, RES> service = (JRos2ServiceClient<REQ, RES>) serviceList.get(serviceName);
    try {
      return service.sendRequestAsync(message).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void createTf() {
    tf = new JRos2Tf2Factory().createTf2Client(client);
  }

  public Matrix4d lookupTransform(String from, String to) {
    try {
      LookupTransformResultMessage res = tf.lookupTransform(from, to);
      QuaternionMessage quatMsg = res.transform.transform.rotation;
      Quat4d quat = new Quat4d(quatMsg.x, quatMsg.y, quatMsg.z, quatMsg.w);
      Vector3Message transMsg = res.transform.transform.translation;
      Vector3d trans = new Vector3d(transMsg.x, transMsg.y, transMsg.z);
      return new Matrix4d(quat, trans, 1);
    } catch (JRosTf2Exception e) {
      throw new RuntimeException(e);
    }
  }

  public <GOAL extends Message, RES extends Message, DEF extends Action2Definition<GOAL, RES>> void addAction(DEF def, String actionName) {
    log.error("ERROR: This functionality is currently broken due to library");
    try {
      JRos2ActionClient<GOAL, RES> actionClient =
              new JRos2ActionClientFactory().createClient(client, def, actionName);
      actionList.put(actionName, actionClient);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public <GOAL extends Message, RES extends Message> RES callAction(String actionName, GOAL goal) {
    log.error("ERROR: This functionality is currently broken due to library");

    JRos2ActionClient<GOAL, RES> actionClient =
            (JRos2ActionClient<GOAL, RES>) actionList.get(actionName);
    try {
      return actionClient.sendGoalAsync(goal).get();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

  }
}
