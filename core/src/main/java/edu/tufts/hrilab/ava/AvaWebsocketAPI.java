/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import edu.tufts.hrilab.ava.message.request.*;
import edu.tufts.hrilab.ava.message.response.*;

import java.lang.Thread;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import java.net.URI;


import edu.tufts.hrilab.util.Http;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AvaWebsocketAPI implements AvaAPIInterface {
  private Logger log = LoggerFactory.getLogger(AvaWebsocketAPI.class);
  public static AvaWebsocketClient client;
  public static boolean resolved = true;
  public static String message = null;

  private String mapName = "Building-10-1";
  private boolean session = false;

  public AvaWebsocketAPI(URI avaUri, String token, String mapNameString) {
    Map<String, String> httpHeaders = new HashMap<String, String>();
    httpHeaders.put("Authorization", "Basic " + token);
    httpHeaders.put("Sec-WebSocket-Protocol", "irobot-bcr-schedule");
    mapName = mapNameString;

    AvaWebsocketAPI.client = new AvaWebsocketClient(avaUri, httpHeaders);
    AvaWebsocketAPI.client.connect();
    while (!AvaWebsocketAPI.client.connected) {
      try {
        Thread.sleep(1);
      } catch (Exception ex) {
        log.error("Error sleeping 1ms", ex);
      }
    }
    messagePoll();
    start();
  }

  /**
   * Reserves or resumes an existing session. This gives the user
   * access to a robot for 1 hour by default.
   */
  private void start() {
    String uri = "/rms/goToMap";
    SessionArguments args = new SessionArguments(mapName);
    RMSRequest startMsg = new RMSRequest(uri, args);
    log.debug("Sending start session message");
    RMSResponse res = send(startMsg);
    session = true;
  }

  /**
   * Terminates a session. After 30 seconds the robot will return
   * to home base.
   */
  @Override
  public void end() {
    String uri = "/rms/endSession";
    RMSRequest endMsg = new RMSRequest(uri);
    log.debug("Sending end session message");
    RMSResponse res = send(endMsg);
    session = false;
  }

  @Override
  public RMSResponse setVelocity(float translation, float rotation, float sidestep) {
    String uri = "/robot/drive/velocity";
    VelocityRequest request = new VelocityRequest(translation, rotation, sidestep);
    RMSRequest velMsg = new RMSRequest(uri, request);
    log.debug("Sending velocity message");
    return send(velMsg);
  }

  @Override
  public RMSResponse setVelocity(float translation, float rotation, float sidestep, boolean nudge) {
    String uri = "/robot/drive/velocity";
    VelocityRequest request = new VelocityRequest(translation, rotation, sidestep, nudge);
    RMSRequest velMsg = new RMSRequest(uri, request);
    log.debug("Sending velocity message");
    return send(velMsg);
  }

  /**
   * Send the robot to a given position on the map.
   *
   * @param x     The x coordinates on the map (in pixels)
   * @param y     The y coordinates on the map (in pixels)
   */
  @Override
  public RMSResponse goToArbitrary(double x, double y) {
    String uri = "/robot/drive/destination";
    RMSDestinationPosition request = new RMSDestinationPosition(x, y);
    RMSRequest goToMsg = new RMSRequest(uri, request);
    log.debug("Sending arbitrary goto message");
    return send(goToMsg);
  }

  /**
   * Send the robot to a given pose on the map.
   *
   * @param x     The x coordinates on the map (in pixels)
   * @param y     The y coordinates on the map (in pixels)
   * @param theta The desired orientation of the robot on the map
   */
  @Override
  public RMSResponse goToArbitrary(double x, double y, double theta) {
    String uri = "/robot/drive/destination";
    RMSDestinationPose args = new RMSDestinationPose(x, y, theta);
    RMSRequest goToMsg = new RMSRequest(uri, args);
    log.debug("Sending arbitrary goto message");
    return send(goToMsg);
  }

  /**
   * Send the robot to a room waypoint on the map.
   *
   * @param room Name of room in map to navigate to
   */
  public RMSResponse goToLocation(String room) {
    String uri = "/robot/tel/goToRoom";
    LocationArguments args = new LocationArguments(mapName, room);
    RMSRequest goToMsg = new RMSRequest(uri, args);
    log.debug("Sending goto location message");
    return send(goToMsg);
  }

  /**
   * Send the robot to a waypoint within a room on the map.
   *
   * @param room       Name of room waypoint is in
   * @param waypointId Unique id of the waypoint
   */
  public RMSResponse goToLocation(String room, int waypointId) {
    String uri = "/robot/tel/goToRoom";
    LocationArguments args = new LocationArguments(mapName, room, waypointId);
    RMSRequest goToMsg = new RMSRequest(uri, args);
    log.debug("Sending goto location message");
    return send(goToMsg);
  }

  /**
   * Send the robot to the dock after waiting 30 seconds.
   *
   * @return object from RMS containing success or error
   */
  @Override
  public RMSResponse dock() {
    String uri = "/robot/dock/dock";
    DockRequest dockMsg = new DockRequest(uri);
    log.debug("Sending docking message");
    return sendDock(dockMsg);
  }

  /**
   * Stop any go to navigation action for named or arbitrary locations.
   *
   * @return object from RMS with error or status
   */
  @Override
  public RMSResponse stop() {
    String uri = "/robot/drive/stopRobot";
    RMSRequest stopMsg = new RMSRequest(uri);
    log.debug("Sending stop message");
    return send(stopMsg);
  }

  /**
   * Get the status of the robot while going to a tagged/named
   * location within RMS.
   *
   * @return status object from RMS
   */
  @Override
  public RMSGoToStatusResponse getGoToStatus() {
    String uri = "/robot/tel/goToStatus";
    RMSRequest statusMsg = new RMSRequest(uri);
    log.debug("Sending status message");
    return sendStatus(statusMsg);
  }

  /**
   * Get the current position of the robot on the map (in pixels).
   *
   * @return Object containing position and orientation data
   */
  @Override
  public RMSPositionResponse getPosition() {
    String uri = "/robot/drive/position";
    RMSRequest positionMsg = new RMSRequest(uri);
    log.debug("Sending get position message");
    return sendPosition(positionMsg);
  }

  /**
   * Gets a list of all rooms and waypoints available for
   * navigation.
   *
   * @return Object containing all rooms and waypoints in map
   *
   * TODO: Add room representation to RMSMapInfoResponse, currently just waypoints
   */
  public RMSMapResponse getMapLocations() {
    String uri = "/rms/layoutMapFeatures";
    MapArguments args = new MapArguments(mapName);
    RMSRequest mapMsg = new RMSRequest(uri, args);
    log.debug("Sending get map info message");
    return sendMap(mapMsg);
  }

  /**
   * Get the scale and resolution data about the current map.
   *
   * @return Object containing the map metadata
   */
  @Override
  public RMSMapMetadataResponse getMapMetadata() {
    String uri = "/rms/layoutMapMetadata";
    MapArguments args = new MapArguments(mapName);
    RMSRequest mapMsg = new RMSRequest(uri, args);
    log.debug("Sending get map metadata message");
    return sendMapMetadata(mapMsg);
  }
  @Override
  public RMSMapMetadataResponse getMapMetadata(String mapName) {
    String uri = "/rms/layoutMapMetadata";
    MapArguments args = new MapArguments(mapName);
    RMSRequest mapMsg = new RMSRequest(uri, args);
    log.debug("Sending get map metadata message");
    return sendMapMetadata(mapMsg);
  }
  @Override
  public List<RMSMapMetadataResponse> getMapMetadatas() {
    return List.of(getMapMetadata());
  }

  /**
   * Get the health of the robot.
   *
   * @return Object containing robot health metadata
   */
  @Override
  public RMSHealthResponse getHealth() {
    String uri = "/robot/health";
    RMSRequest mapMsg = new RMSRequest(uri);
    log.debug("Sending get robot health message");
    return sendHealth(mapMsg);
  }

  /**
   * Get the health of the robot.
   *
   * @return Object containing robot health metadata
   */
  @Override
  public RMSZLiftCameraTiltResponse getZLiftCameraTilt() {
    String uri = "/robot/drive/getZLiftCameraTilt";
    RMSRequest zcamMsg = new RMSRequest(uri);
    log.debug("Sending get robot Z lift and camera tilt message");
    return sendZLiftCameraTilt(zcamMsg);
  }

  /**
   * Get the estimated runtime of the robot.
   *
   * @return Object containing robot runtume metadata
   */
  @Override
  public RMSRuntimeResponse getRuntime() {
    String uri = "/robot/tel/runtime";
    RMSRequest runtimeMsg = new RMSRequest(uri);
    log.debug("Sending get runtime");
    return sendRuntime(runtimeMsg);
  }

  /**
   * Get a lidar scan from the robot.
   *
   * @return Object containing robot lidar
   */
  @Override
  public RMSLidarResponse getLidar() {
    String uri = "/robot/perception/rangescan";
    RMSRequest lidarMsg = new RMSRequest(uri);
    log.debug("Sending get lidar");
    return sendLidar(lidarMsg);
  }

  /**
   * Get list of dynamic obstacles from the robot
   *
   * @return Object containing dyanmic obstacles list
   */
  @Override
  public RMSDynamicObstaclesResponse getDynamicObstacles() {
    String uri = "/robot/perception/dynamicObstacles";
    RMSRequest obstaclesMsg = new RMSRequest(uri);
    log.debug("Sending get dynamic obstacles");
    return sendDyanmicObstacles(obstaclesMsg);
  }

  /**
   * Get status of robot docking from the robot.
   *
   * @return Object containing dock status
   */
  @Override
  public RMSDockStatusResponse getDockStatus() {
    String uri = "/robot/dock/status";
    RMSRequest dockStatusMsg = new RMSRequest(uri);
    log.debug("Sending get dock status");
    return sendDockStatus(dockStatusMsg);
  }

  /**
   * Set the Z lift position of the robot.
   *
   * @param position Z lift position
   * @return object from RMS with error or status
   */
  @Override
  public RMSResponse setZLift(double position) {
    String uri = "/robot/drive/payloadPose";
    ZLiftArguments args = new ZLiftArguments(position);
    RMSRequest zLiftMsg = new RMSRequest(uri, args);
    log.debug("Sending set Z lift message");
    return send(zLiftMsg);
  }

  /**
   * Set the Z lift position of the robot.
   *
   * @param position Z lift position
   * @return object from RMS with error or status
   */
  @Override
  public RMSResponse setCameraTilt(double position) {
    String uri = "/robot/drive/payloadPose";
    CameraTiltArguments args = new CameraTiltArguments(position);
    RMSRequest camTiltMsg = new RMSRequest(uri, args);
    log.debug("Sending set camera tilt message");
    return send(camTiltMsg);
  }

  /**
   * Rotates the robot on the Z axis.
   *
   * @param theta goal rotation
   * @return response object
   */
  @Override
  public RMSResponse setRotation(double theta) {
    String uri = "/robot/drive/destination";
    RMSPositionResponse position = getPosition();
    RMSDestinationPose args = new RMSDestinationPose(Double.valueOf(position.response.x), Double.valueOf(position.response.y), theta);
    RMSRequest rotateMsg = new RMSRequest(uri, args);
    log.debug("Sending rotate message");
    return send(rotateMsg);
  }

  /**
   * Look at a vector by rotation to face it.
   *
   * @param x        double of x position
   * @param y        double of y position
   * @param z        double of z position
   * @param duration double of seconds to hold look
   * @return response object
   */
  @Override
  public RMSResponse lookAt(double x, double y, double z, double duration) {
    String uri = "/robot/kinematics/look";
    LookAtArguments args = new LookAtArguments(x, y, z, duration);
    RMSRequest lookAtMsg = new RMSRequest(uri, args);
    log.debug("Sending look at message");
    return send(lookAtMsg);
  }

  /**
   * SEND METHODS
   */

  /**
   * Send a message to the RMS server for a generic response.
   *
   * @param req Request object for the RMS client
   * @return RMS client response object
   */
  private RMSResponse send(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSResponse res = RMSResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a dock response.
   *
   * @param req Request object for the RMS client
   * @return RMS client response object
   */
  private RMSResponse sendDock(DockRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSResponse res = RMSResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a map response.
   *
   * @param req Request object for the RMS client
   * @return Map response object
   */
  private RMSMapResponse sendMap(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSMapResponse res = RMSMapResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a map metadata response.
   *
   * @param req Request object for the RMS client
   * @return Map metadata response object
   */
  private RMSMapMetadataResponse sendMapMetadata(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSMapMetadataResponse res = RMSMapMetadataResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a position info response.
   *
   * @param req Request object for the RMS client
   * @return Position response object
   */
  private RMSPositionResponse sendPosition(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSPositionResponse res = RMSPositionResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a go to status response
   *
   * @param req Request object for the RMS client
   * @return Go to status response object
   */
  private RMSGoToStatusResponse sendStatus(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSGoToStatusResponse res = RMSGoToStatusResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a health status response
   *
   * @param req Request object for the RMS client
   * @return Health status response object
   */
  private RMSHealthResponse sendHealth(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSHealthResponse res = RMSHealthResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a Z lift and camera tilt repsonse.
   *
   * @param req Request object for the RMS client
   * @return Z lift status response object
   */
  private RMSZLiftCameraTiltResponse sendZLiftCameraTilt(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSZLiftCameraTiltResponse res = RMSZLiftCameraTiltResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a runtime estimate response.
   *
   * @param req Request object for the RMS client
   * @return Runtime estimate response object
   */
  private RMSRuntimeResponse sendRuntime(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSRuntimeResponse res = RMSRuntimeResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a runtime estimate response.
   *
   * @param req Request object for the RMS client
   * @return Runtime estimate response object
   */
  private RMSDynamicObstaclesResponse sendDyanmicObstacles(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSDynamicObstaclesResponse res = RMSDynamicObstaclesResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a lidar response.
   *
   * @param req Request object for the RMS client
   * @return Lidar status response object
   */
  private RMSLidarResponse sendLidar(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSLidarResponse res = RMSLidarResponse.fromString(resStr);
    return res;
  }

  /**
   * Send a message to the RMS server for a dock status response.
   *
   * @param req Request object for the RMS client
   * @return Dock status status response object
   */
  private RMSDockStatusResponse sendDockStatus(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSDockStatusResponse res = RMSDockStatusResponse.fromString(resStr);
    return res;
  }

  private String waitForResponse(String reqStr) {
    String response = "";
    AvaWebsocketAPI.resolved = false;
    AvaWebsocketAPI.client.waiting = true;
    AvaWebsocketAPI.client.send(reqStr);
    log.debug("Sent message " + reqStr);
    while (!AvaWebsocketAPI.resolved) {
      try {
        Thread.sleep(1);
      } catch (Exception ex) {
        log.error("Error sleeping 1ms", ex);
      }
    }
    response += AvaWebsocketAPI.message;
    AvaWebsocketAPI.message = null;
    return response;
  }

  /**
   * Poll for a response message from the websocket client
   * on a separate thread. Store the message in a static property.
   */
  private void messagePoll() {
    new Thread(() -> {
      log.debug("Started polling for new messages");
      while (true) {
        if (!AvaWebsocketAPI.client.waiting && AvaWebsocketAPI.client.message != null) {
          AvaWebsocketAPI.message = AvaWebsocketAPI.client.message;
          AvaWebsocketAPI.client.message = null;
          AvaWebsocketAPI.resolved = true;
          log.debug("Got new message " + message);
        }
        try {
          Thread.sleep(1);
        } catch (Exception ex) {
          log.error("Error sleeping 1ms", ex);
        }
      }
    }).start();
  }
  //  /robot/drive/stopRobot


  /**
   * Returns session status.
   *
   * @return whether robot has active session with client
   */
  @Override
  public boolean hasSession() {
    return session;
  }

  /**
   * Determine if robot is moving.
   *
   * @return whether robot is moving
   */
  @Override
  public boolean isMoving() {
    return false;
  }

  /**
   * Get the current battery level as a percentage.
   *
   * @return double 0-100 representing battery level
   */
  @Override
  public double getBatteryLevel() {
    RMSHealthResponse res = getHealth();
    return Double.valueOf(res.response.batteryCharge);
  }

  /**
   * Get status of robot driving from the robot.
   *
   * @return Object containing dock status
   */
  @Override
  public RMSDriveStatusResponse getDriveStatus() {
    String uri = "/robot/drive/updates";
    RMSRequest driveStatusMsg = new RMSRequest(uri);
    log.debug("Sending get dock status");
    return sendDriveStatus(driveStatusMsg);
  }

  /**
   * Send a message to the RMS server for a drive status response.
   *
   * @param req Request object for the RMS client
   * @return Drive status status response object
   */
  private RMSDriveStatusResponse sendDriveStatus(RMSRequest req) {
    String reqStr = req.toString();
    String resStr = waitForResponse(reqStr);
    RMSDriveStatusResponse res = RMSDriveStatusResponse.fromString(resStr);
    return res;
  }
}
