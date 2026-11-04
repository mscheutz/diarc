/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.tf;

import edu.tufts.hrilab.diarcros.common.RosConfiguration;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.TransformStamped;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Transform;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;

import edu.tufts.hrilab.diarcros.util.Convert;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.address.InetAddressFactory;
import tf2_msgs.TFMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import edu.tufts.hrilab.diarcros.msg.std_msgs.Header;

/**
 * @author Evan Krause
 */
public class TF {
  private RosConfiguration rc;
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // Local state of ROS data
  private edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage tf;
  private ConcurrentMap<String, TransformStamped> transformMap = new ConcurrentHashMap<>();

  // ROS node ready/wait
  private volatile boolean tfReceived = false;
  private volatile boolean staticTfReceived = true;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  private Logger log = LoggerFactory.getLogger(TF.class);

  public TF(RosConfiguration rc) {
    this.rc = rc;
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of(rc.namespace +"/diarc/tf_handler_"+ rc.uniqueID);
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        System.err.println("on start");
        node = connectedNode;
        // Subscribers
        Subscriber<TFMessage> tfSub = node.newSubscriber("/tf", TFMessage._TYPE);
        Subscriber<TFMessage> tfStaticSub = node.newSubscriber("/tf_static", TFMessage._TYPE);

        tfSub.addMessageListener(msg -> {
          edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage tf = edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toAde(msg);
          for (TransformStamped t : tf.getTransforms()) {
            transformMap.put(t.getChildFrameId(), t);

            // make sure root node gets added to "tree"
            if (!transformMap.containsKey(t.getHeader().getFrameId())) {
              TransformStamped rootTransform = new TransformStamped();
              rootTransform.getHeader().setFrameId("null");
              rootTransform.setChildFrameId(t.getHeader().getFrameId());
              rootTransform.getTransform().setRotation(new Quaternion(0,0,0,1));
              rootTransform.getTransform().setTranslation(new Vector3(0,0,0));
              transformMap.putIfAbsent(t.getHeader().getFrameId(), rootTransform);
            }
          }

          // notify of node ready (only needs to happen once)
          if (!tfReceived) {
            //nodeReadyLock.lock();
            tfReceived = true;
            //try {
            //  nodeReadyCond.signalAll();
            //} finally {
            //  nodeReadyLock.unlock();
            //}
          }
        });

        tfStaticSub.addMessageListener(msg -> {
          edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage tf = edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toAde(msg);
          for (TransformStamped t : tf.getTransforms()) {
            transformMap.put(t.getChildFrameId(), t);

            // make sure root node gets added to "tree"
            if (!transformMap.containsKey(t.getHeader().getFrameId())) {
              TransformStamped rootTransform = new TransformStamped();
              rootTransform.getHeader().setFrameId("null");
              rootTransform.setChildFrameId(t.getHeader().getFrameId());
              rootTransform.getTransform().setRotation(new Quaternion(0,0,0,1));
              rootTransform.getTransform().setTranslation(new Vector3(0,0,0));
              transformMap.putIfAbsent(t.getHeader().getFrameId(), rootTransform);
            }
          }

          // notify of node ready (only needs to happen once)
          if (!staticTfReceived) {
            //nodeReadyLock.lock();
            staticTfReceived = true;
            //try {
            //  nodeReadyCond.signalAll();
            //} finally {
            //  nodeReadyLock.unlock();
            //}
          }
        });
      }
    };
    String host = InetAddressFactory.newNonLoopback().getHostAddress();
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, rc.rosMasterUri);
    nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
    nodeMainExecutor.execute(nodeMain, nodeConfiguration);
  }

  public TF() {
      this(new RosConfiguration());
  }

  /**
   * Wait for node to be connected and ready.
   */
  public void waitForNode() {
    nodeReadyLock.lock();
    try {
      while (!tfReceived || !staticTfReceived) {
        //nodeReadyCond.awaitUninterruptibly();
        Thread.sleep(250);
      }
    } catch(InterruptedException e) {
      log.warn("waitForNode sleep interrupted");
    } finally {
      nodeReadyLock.unlock();
    }
  }

  /**
   * Is node connected and ready.
   * @return
   */
  public boolean isReady() {
    return tfReceived && staticTfReceived;
  }

  public void addLocalStaticTransform(String frame_id, String child_frame_id, Vector3 translation, Quaternion rotation) {
    TransformStamped t = new TransformStamped(child_frame_id, new Transform(translation, rotation));
    t.setHeader(new Header(0, new edu.tufts.hrilab.diarcros.msg.Time(), frame_id));
    transformMap.put(child_frame_id, t);
  }

  public Set<String> getTransformNames() {
    return transformMap.keySet();
  }

  public void printTransformTree() {
    StringBuilder sb = new StringBuilder("Transform tree:\n");
    for (Map.Entry<String, TransformStamped> entry : transformMap.entrySet()) {
      sb.append(" child: ").append(entry.getValue().getChildFrameId())
              .append(" header: ").append(entry.getValue().getHeader().getFrameId()).append("\n");
    }
    log.info(sb.toString());
  }

  public TransformStamped getTransform(List<TransformStamped> transforms, String link) {
    for (TransformStamped t : transforms) {
      if (t.getChildFrameId().equals(link)) {
        return t;
      }
    }
    return null;
  }

  /**
   * Get the chain of transforms to transform from sourceFrame to targetFrame.
   * @param sourceFrame
   * @param targetFrame
   * @return
   */
  private List<Matrix4d> getTransformChain(String sourceFrame, String targetFrame) {
    //log.debug(String.format("[getTransformChain] number of transforms: %d.", transformMap.size()));

    // Get targetFrame link
    TransformStamped targetFrameLink = transformMap.get(targetFrame);
    if (targetFrameLink == null) {
      log.error("[getTransformChain] link not found: " + targetFrame);
      return null;
    }
    // Get kinematic chain from targetFrame --> root
    List<Matrix4d> targetFrameLinks = new ArrayList<>();
    boolean foundSourceFrame = false;
    TransformStamped currLink = targetFrameLink;
    while (!foundSourceFrame && currLink != null) {
      if (currLink.getChildFrameId().equals(sourceFrame)) {
        foundSourceFrame = true;
      } else {
        targetFrameLinks.add(Convert.convertToMatrix4d(currLink));
        currLink = transformMap.get(currLink.getHeader().getFrameId());
      }
    }

    // reverse order of links
    Collections.reverse(targetFrameLinks);

    // if found source frame, return results, we're done!
    if (foundSourceFrame) {
      return targetFrameLinks;
    }

    //////////////////////////////////////////////////////////
    // Get targetFrame link
    TransformStamped sourceFrameLink = transformMap.get(sourceFrame);
    if (sourceFrameLink == null) {
      log.error("[getTransformChain] link not found: " + sourceFrameLink);
      return null;
    }

    // Get kinematic chain from sourceFrame --> root
    List<Matrix4d> sourceFrameLinks = new ArrayList<>();
    boolean foundTargetFrame = false;
    currLink = sourceFrameLink;
    while (!foundTargetFrame && currLink != null) {
      if (currLink.getChildFrameId().equals(targetFrame)) {
        foundTargetFrame = true;
      } else {
        Matrix4d tempTransform = Convert.convertToMatrix4d(currLink);
        tempTransform.invert();
        sourceFrameLinks.add(tempTransform);
        currLink = transformMap.get(currLink.getHeader().getFrameId());
      }
    }

    // do not reserve links in this case

    // if found source frame, return results, we're done!
    if (foundTargetFrame) {
      return sourceFrameLinks;
    }

    //////////////////////////////////////////////////////////
    // if search from target --> root and source --> root didn't result
    // in connecting chain, combine the two chains into one
    log.warn("Combining transform chains.");
    List<Matrix4d> combinedLinks = new ArrayList<>(sourceFrameLinks);
    combinedLinks.addAll(targetFrameLinks);
    return combinedLinks;
  }

  /**
   * This transform returned is to transform a pose in the refLink coordinate frame
   * to a pose in the targetLink coordinate frame.
   * @param sourceFrame
   * @param targetFrame
   * @return
   */
  public Matrix4d getTransform(String sourceFrame, String targetFrame) {
    List<Matrix4d> transforms = getTransformChain(sourceFrame, targetFrame);
    if (transforms == null) {
      return null;
    }

    // combine transform chain into single transform
    Matrix4d resultingTransform = new Matrix4d();
    resultingTransform.setIdentity();
    for (Matrix4d currTransform : transforms) {
      resultingTransform.mul(currTransform);
    }

    //if (log.isDebugEnabled()) {
    //  Vector3d translation = new Vector3d();
    //  resultingTransform.get(translation);
    //  Quat4d orientation = new Quat4d();
    //  resultingTransform.get(orientation);
    //  StringBuilder sb = new StringBuilder();
    //  sb.append("[getTransform]").append("\n");
    //  sb.append("     Translation: ").append(translation).append("\n");
    //  sb.append("     Rotation RPY (radian): ").append(RotationHelpers.quatToXYZRotations(orientation));
    //  log.debug(sb.toString());
    //}
    return resultingTransform;
  }
}
