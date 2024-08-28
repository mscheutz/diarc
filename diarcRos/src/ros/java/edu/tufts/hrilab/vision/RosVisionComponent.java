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
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
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
import org.ros.node.topic.Publisher;
import sensor_msgs.JointState;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;
import std_msgs.ColorRGBA;
import vision_msgs.Detection3D;
import vision_msgs.Detection3DArray;
import vision_msgs.ObjectHypothesisWithPose;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RosVisionComponent extends DiarcComponent {
  private RosConfiguration rc = new RosConfiguration();

  // ROS connection

  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // TODO: add services to turn on/off publishing visualization data
  private boolean publishVisData = false;

  private Publisher<MarkerArray> visualizationPub;
  private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(1);

  @Override
  public void init() {
    initializeRosNode(rc);
    if (publishVisData) {
      executor.scheduleAtFixedRate(this::publishVisualizationMarkers, 0, 3, TimeUnit.SECONDS);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("pub_vis").desc("publish visualization data").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("pub_vis")) {
      publishVisData = true;
    }
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

        visualizationPub = connectedNode.newPublisher("detection_visualization_markers", MarkerArray._TYPE);
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

  private int visPubId = 0;
  private void publishVisualizationMarkers() {

    List<MemoryObject> mos;
    try {
      mos = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTokens").argTypes()).call(List.class);
    } catch (TRADEException e) {
      log.error("Error calling getTokens service.", e);
      return;
    }

    MarkerArray markerArray = node.getTopicMessageFactory().newFromType(MarkerArray._TYPE);
    List<Marker> markers = new ArrayList<>();
    for (MemoryObject mo : mos) {
      Marker marker = node.getTopicMessageFactory().newFromType(Marker._TYPE);
      marker.setType(Marker.CUBE);
      marker.setAction(Marker.ADD);
      marker.setId(visPubId++);
      Pose pose = marker.getPose();
      Point point = pose.getPosition();
      point.setX(mo.getLocation().x);
      point.setY(mo.getLocation().y);
      point.setZ(mo.getLocation().z);
      Quaternion orient = pose.getOrientation();
      orient.setX(mo.getOrientation().x);
      orient.setY(mo.getOrientation().y);
      orient.setZ(mo.getOrientation().z);
      orient.setW(mo.getOrientation().w);
      ColorRGBA color = marker.getColor();
      color.setR(1f);
      color.setG(0f);
      color.setB(0f);
      color.setA(1f);
      Vector3 scale = marker.getScale(); // size
      scale.setX(0.5);
      scale.setY(0.5);
      scale.setZ(0.5);

      markers.add(marker);
    }
    markerArray.setMarkers(markers);
    visualizationPub.publish(markerArray);
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
    vision_msgs.Detection3DArray detections = node.getTopicMessageFactory().newFromType(Detection3DArray._TYPE);
    detections.getHeader().setFrameId("base_link");
    detections.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));

    for (MemoryObject mo : mos) {
      // transform detection into the base_link frame
      mo.transformToBase();

      vision_msgs.Detection3D detection3D = node.getTopicMessageFactory().newFromType(Detection3D._TYPE);

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
      vision_msgs.ObjectHypothesisWithPose hypothesis = node.getTopicMessageFactory().newFromType(ObjectHypothesisWithPose._TYPE);
      detection3D.getResults().add(hypothesis);
      hypothesis.setId(mo.getTokenId());
      hypothesis.setScore(mo.getDetectionConfidence());
      PoseWithCovariance poseWithCovar = hypothesis.getPose();
      poseWithCovar.setPose(detection3D.getBbox().getCenter());

      // set point cloud of object
      double[][] cloud = mo.getPointCloud();
      int numPoints = cloud.length;
      List<PointField> fields = new ArrayList<>(numPoints);

      PointField pointFieldX = node.getTopicMessageFactory().newFromType(PointField._TYPE);
      pointFieldX.setName("x"); pointFieldX.setOffset(0); pointFieldX.setDatatype(PointField.FLOAT32); pointFieldX.setCount(1);
      PointField pointFieldY = node.getTopicMessageFactory().newFromType(PointField._TYPE);
      pointFieldX.setName("y"); pointFieldX.setOffset(4); pointFieldX.setDatatype(PointField.FLOAT32); pointFieldX.setCount(1);
      PointField pointFieldZ = node.getTopicMessageFactory().newFromType(PointField._TYPE);
      pointFieldX.setName("z"); pointFieldX.setOffset(8); pointFieldX.setDatatype(PointField.FLOAT32); pointFieldX.setCount(1);
      fields.add(pointFieldX);
      fields.add(pointFieldY);
      fields.add(pointFieldZ);
      ChannelBuffer data = ChannelBuffers.buffer(numPoints*Float.BYTES*3);
      for (int n = 0; n < numPoints; ++n) {
        for (int dim = 0; dim < 3; ++dim) {
          data.setFloat(dim + n*3, (float)cloud[n][dim]);
        }
      }
      PointCloud2 cloud2 = detection3D.getSourceCloud();
      cloud2.setData(data);
      cloud2.setFields(fields);
      cloud2.setIsDense(false);
      cloud2.setHeight(1);
      cloud2.setWidth(numPoints);
      cloud2.setIsBigendian(false);
      cloud2.setPointStep(Float.BYTES*3);
      cloud2.setRowStep(numPoints*Float.BYTES*3);

      // add detection to list
      detections.getDetections().add(detection3D);
    }

    return detections;
  }
}
