/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.ros2.ros2jrosclient.generated.*;
import id.jros2client.JRos2ClientFactory;
import id.jrosmessages.Message;
import id.jrosmessages.std_msgs.Int32Message;
import id.jrosmessages.std_msgs.StringMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2Definition;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalMessage;
import pinorobotics.jros2services.JRos2ServiceClientFactory;

import java.util.concurrent.ExecutionException;

public class TestGeneric extends DiarcComponent {

  public TestGeneric() {
    Ros2Node pubNode = Ros2Factory.getDefaultNode();
    pubNode.spin(100);

    //When publishing and subscribing on the same thread, make a separate node for each.
    Ros2Node subNode = Ros2Factory.getDefaultNode();
    pubNode.spin(100);

    // Initialize a new publisher of type StringMessage on topic chatter
    Ros2Publisher<StringMessage> dynamicPub = Ros2Factory.getDefaultPublisher(StringMessage.class,
        "/chatter");
    // Set the initial data (there are many ways to do this, as we will see!)
    StringMessage pubmsg = dynamicPub.getMessage();
    pubNode.addAndStartPublisher(dynamicPub);

    //Here we demonstrate that the message changes as the message object is updated.
    pubmsg.data = "Hello 1";
    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    pubmsg.data = "Hello 2";

    // You can also start a publisher with an already populated message
    Ros2Publisher<StringMessage> pub = Ros2Factory.getDefaultPublisher(StringMessage.class,
        "/chatter2"
        , new StringMessage().withData("Hello World!"));
    pubNode.addAndStartPublisher(pub);

    // Start a new subscriber on chatter.
    Ros2Subscriber<StringMessage> sub = Ros2Factory.getDefaultSubscriber(StringMessage.class,
        "/chatter", stringMessage -> log.info(stringMessage.data));
    subNode.addAndStartSubscriber(sub);

    // Just to demonstrate you can add make the callback object independently
    SubscriberCallback<StringMessage> callback = o -> log.info(o.data);
    Ros2Subscriber<StringMessage> sub2 = Ros2Factory.getDefaultSubscriber(StringMessage.class,
        "/chatter2", callback);
    subNode.addAndStartSubscriber(sub2);

//    Service definition - still a work in progress to make generic! But technically works. Need
//     to figure out if we can RECEIVE service requests, and not just send them.
//    pubNode.addService(new AddTwoIntsServiceDefinition(), "add_two_ints");
//    AddTwoIntsRequestMessage testMsg = new AddTwoIntsRequestMessage(2, 3);
//    AddTwoIntsResponseMessage testResp = pubNode.callService("add_two_ints", testMsg);
//    log.info("Received " + testResp.sum);

    // FIXME: Whenever this gets updated
//    node.addAction(new FibonacciActionDefinition(),"fibonacci");
//    FibonacciActionGoalMessage goal =
//        new FibonacciActionGoalMessage().withGoal(new FibonacciGoalMessage().withOrder(new Int32Message().withData(13)));
//    Message res = node.callAction("fibonacci", goal);
//    log.info(res.toString());
  }
}
