package edu.tufts.hrilab.vision;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import diarc_vision_ros.*;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarcros.common.RosConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import geometry_msgs.PoseWithCovariance;
import org.ros.address.InetAddressFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import std_msgs.Bool;
import std_msgs.Int64;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

public class RosVisionComponent extends DiarcComponent {
  private RosConfiguration rc = new RosConfiguration();

  // ROS connection

  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  public RosVisionComponent() {
  }

  @Override
  public void init() {
    initializeRosNode(rc);
  }

  public void initializeRosNode(RosConfiguration rc) {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of(rc.namespace + "/diarc/vision");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;

        // register start visual search service
        connectedNode.newServiceServer("StartVisualSearch", StartVisualSearch._TYPE,
                (ServiceResponseBuilder<StartVisualSearchRequest, StartVisualSearchResponse>) (request, response) -> {
          boolean result = startVisualSearch(request.getSearchId());
          response.setStatus(result);
        });

        // register stop visual search service
        connectedNode.newServiceServer("StopVisualSearch", StopVisualSearch._TYPE,
                (ServiceResponseBuilder<StopVisualSearchRequest, StopVisualSearchResponse>) (request, response) -> {
                  boolean result = stopVisualSearch(request.getSearchId());
                  response.setStatus(result);
                });


        // register get visual search results service
        connectedNode.newServiceServer("GetVisualSearchResults", GetVisualSearchResults._TYPE,
                (ServiceResponseBuilder<GetVisualSearchResultsRequest, GetVisualSearchResultsResponse>) (request, response) -> {
                  vision_msgs.Detection3DArray results = getVisualSearchResults(request.getSearchId());
                  response.setDetections(results);
                });

        // register get ALL visual search results service
        connectedNode.newServiceServer("GetAllVisualSearchResults", GetAllVisualSearchResults._TYPE,
                (ServiceResponseBuilder<GetAllVisualSearchResultsRequest, GetAllVisualSearchResultsResponse>) (request, response) -> {
                  vision_msgs.Detection3DArray results = getAllVisualSearchResults();
                  response.setDetections(results);
                });

        // register get visual search ID service
        connectedNode.newServiceServer("GetVisualSearchId", GetVisualSearchId._TYPE,
                (ServiceResponseBuilder<GetVisualSearchIdRequest, GetVisualSearchIdResponse>) (request, response) -> {
                  long searchId = getVisualSearchId(request.getDescriptors());
                  response.setSearchId(searchId);
                });

        // register get visual search descriptors service
        connectedNode.newServiceServer("GetVisualSearchDescriptors", GetVisualSearchDescriptors._TYPE,
                (ServiceResponseBuilder<GetVisualSearchDescriptorsRequest, GetVisualSearchDescriptorsResponse>) (request, response) -> {
                  List<String> descriptors = getVisualSearchDescriptors(request.getSearchId());
                  response.setDescriptors(descriptors);
                });
      }
    };

    try {
      URI ros_master_uri = new URI(System.getenv("ROS_MASTER_URI"));
      String host = InetAddressFactory.newNonLoopback().getHostAddress();
      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, ros_master_uri);
      nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(nodeMain, nodeConfiguration);
    } catch (URISyntaxException e) {
      System.err.println("Error trying to create URI: " + e);
    }
  }

  /**
   * Start execution the visual search pipeline.
   *
   * @param searchId search pipeline ID or type ID
   * @return
   */
  private boolean startVisualSearch(long searchId) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("startType").argTypes(Long.class)).call(void.class, searchId);
    } catch (TRADEException e) {
      log.error("Error calling startType service.", e);
      return false;
    }

    return true;
  }

  /**
   * Stop execution of the visual search pipeline.
   *
   * @param searchId search pipeline ID or type ID
   * @return
   */
  private boolean stopVisualSearch(long searchId) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("stopType").argTypes(Long.class)).call(void.class, searchId);
    } catch (TRADEException e) {
      log.error("Error calling stopType service.", e);
      return false;
    }

    return true;
  }

  private vision_msgs.Detection3DArray getVisualSearchResults(long searchId) {
    List<MemoryObject> mos = new ArrayList<>();
    try {
      mos = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTokens").argTypes(Long.class)).call(List.class, searchId);
    } catch (TRADEException e) {
      log.error("Error calling getTokens service.", e);
    }

    return convertToRosMsg(mos);
  }

  private vision_msgs.Detection3DArray getAllVisualSearchResults() {
    List<MemoryObject> mos = new ArrayList<>();
    try {
      mos = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTokens").argTypes()).call(List.class);
    } catch (TRADEException e) {
      log.error("Error calling getTokens service.", e);
    }

    return convertToRosMsg(mos);
  }

  private long getVisualSearchId(List<String> descriptors) {
    // convert string descriptors to Symbols
    // TODO: handle relational descriptors
    List<Symbol> folDescriptors = new ArrayList<>();
    descriptors.forEach(desc -> folDescriptors.add(Factory.createPredicate(desc, "X")));

    try {
      return  TRADE.getAvailableService(new TRADEServiceConstraints().name("getTypeId").argTypes(List.class)).call(Long.class, folDescriptors);
    } catch (TRADEException e) {
      log.error("Error calling getTypeId service.", e);
      return -1L;
    }

  }

  private List<String> getVisualSearchDescriptors(long searchId) {
    try {
      List<Symbol> folDescriptors = TRADE.getAvailableService(new TRADEServiceConstraints().name("getDescriptors").argTypes(Long.class)).call(List.class, searchId);
      List<String> descriptors = new ArrayList<>();
      folDescriptors.forEach(fol -> descriptors.add(fol.getName()));
      return descriptors;
    } catch (TRADEException e) {
      log.error("Error calling getDescriptors service.", e);
      return new ArrayList<>();
    }

  }

  private vision_msgs.Detection3DArray convertToRosMsg(List<MemoryObject> mos) {
    vision_msgs.Detection3DArray detections = node.getTopicMessageFactory().newFromType("vision_msgs/Detection3DArray");
    detections.getHeader().setFrameId("base_link");
    detections.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));

    for (MemoryObject mo : mos) {
      // transform detection into the base_link frame
      mo.transformToBase();

      vision_msgs.Detection3D detection3D = node.getTopicMessageFactory().newFromType("vision_msgs/Detection3D");

      // set bbox
      geometry_msgs.Point position = detection3D.getBbox().getCenter().getPosition();
      position.setX(mo.getLocation().x);
      position.setY(mo.getLocation().y);
      position.setZ(mo.getLocation().z);

      geometry_msgs.Quaternion orientation = detection3D.getBbox().getCenter().getOrientation();
      orientation.setX(0);
      orientation.setY(0);
      orientation.setZ(0);
      orientation.setW(1);

      geometry_msgs.Vector3 size = detection3D.getBbox().getSize();
      size.setX(mo.getDimensions().x);
      size.setY(mo.getDimensions().y);
      size.setZ(mo.getDimensions().z);

      // set object hypotheses
      // TODO: for now object hypothesis uses the same pose as the bbox and does not set the covariance matrix
      vision_msgs.ObjectHypothesisWithPose hypothesis = node.getTopicMessageFactory().newFromType("vision_msgs/ObjectHypothesisWithPose");
      detection3D.getResults().add(hypothesis);
      hypothesis.setId(mo.getTokenId());
      hypothesis.setScore(mo.getDetectionConfidence());
      PoseWithCovariance poseWithCovar = hypothesis.getPose();
      poseWithCovar.setPose(detection3D.getBbox().getCenter());

      // add detection to list
      detections.getDetections().add(detection3D);
    }

    return detections;
  }
}
