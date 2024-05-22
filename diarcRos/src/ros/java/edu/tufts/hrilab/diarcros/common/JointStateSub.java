/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.common;

import org.ros.address.InetAddressFactory;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.*;
import org.ros.node.topic.Subscriber;
import sensor_msgs.JointState;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class JointStateSub {
    private String namespace = "/";
    private Map<GraphName, GraphName> remappings = new HashMap<>();

    private NodeMainExecutor nodeMainExecutor;
    private Subscriber<sensor_msgs.JointState> JointStateSub;
    private sensor_msgs.JointState jointState;
    private boolean jointStateIsReady = false;
    private final int MAX_ATTEMPTS = 5;

    public ConnectedNode node;
    private boolean newMessageAvailable;

    public JointStateSub(String namespace, Map<String, String> remappings) {
        this.namespace = namespace;
        remappings.forEach((k,v) -> this.remappings.put(GraphName.of(k), GraphName.of(v)));
        initialize();
    }

    public JointStateSub() {
        initialize();
    }

    private void initialize() {
        NodeMain nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/joint_states");
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                JointStateSub = connectedNode.newSubscriber(GraphName.of("joint_states"), sensor_msgs.JointState._TYPE);
                JointStateSub.addMessageListener(msg -> {jointState = msg; newMessageAvailable = true;}); // Add a callback: every time a new /joint_states message shows up, save it to our local jointState variable and make it known we have a new bit of data
                jointStateIsReady = true;
            }
        };

        try {
            URI ros_master_uri = new URI(System.getenv("ROS_MASTER_URI"));
            String host = InetAddressFactory.newNonLoopback().getHostAddress();
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, ros_master_uri);
            nodeConfiguration.setParentResolver(NameResolver.newFromNamespaceAndRemappings(namespace, remappings));
            nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
            nodeMainExecutor.execute(nodeMain, nodeConfiguration);
        } catch (URISyntaxException e) {
            System.err.println("Error trying to create URI: " + e);
            jointStateIsReady = false;
        }
        waitUntilReady();
    }


    public void shutdown() {
        nodeMainExecutor.shutdown();
    }

    private boolean waitUntilReady() {
        try { // Loop exits when jointStateIsReady is true, or if we've checked too many times. A delay of 1 second between checks gives more time for things to start up.
            for(int attempts = 0; attempts < MAX_ATTEMPTS && !jointStateIsReady; ++attempts, TimeUnit.SECONDS.sleep(1));
            return jointStateIsReady;
        } catch (InterruptedException e) {
            e.printStackTrace();
            return false;
        }
    }

    public edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getCurrentJointState() {
        if(!waitUntilReady()) {
            System.err.println("Failed to connect to JointState publisher. Returning incorrect joint state.");
            return new edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState(); // at least it's not null
        }

        newMessageAvailable = false;
        return edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(jointState);
    }

    public edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getCurrentJointStateBlocking() { // Blocking in the sense that we don't return until we get a new joint state.
        while(!newMessageAvailable)
            continue;

        return getCurrentJointState();
    }
}
