package edu.tufts.hrilab.ava;

import edu.tufts.hrilab.ava.message.request.*;
import edu.tufts.hrilab.ava.message.response.*;
import edu.tufts.hrilab.util.Http;

import java.awt.image.BufferedImage;
import java.net.URI;
import java.util.Map;

import com.google.gson.Gson;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AvaHTTPAPI implements AvaAPIInterface {
  private Logger log = LoggerFactory.getLogger(AvaHTTPAPI.class);
  private String mapName = "1";
  private String baseUrl = "http://172.18.0.1";
  private Map<String, String> headers = null;

  public AvaHTTPAPI(URI avaUri, String mapName) {
    baseUrl = avaUri.toString();
    this.mapName = mapName;
  }

  /**
   * Null method. Ends an RMS session. Not possible in HTTP client.
   */
  @Override
  public void end() {
    //do nothing
    log.warn("HTTP API does not have session to end");
  }

  /**
   * Get the metadata about the current map.
   *
   * @return object containing metadata about map dimensions and scale
   */
  @Override
  public RMSMapMetadataResponse getMapMetadata() {
    String uri = baseUrl + "/robot/mapping/maps";
    String res = null;
    log.debug("Sending getMapMetadata request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    HTTPMapsResponse mapsResponse = gson.fromJson(res, HTTPMapsResponse.class);
    RMSMapMetadataResponse metadataResponse = new RMSMapMetadataResponse();
    for (var entry : mapsResponse.maps.entrySet()) {
      if (entry.getKey().equals(mapName)) {
        metadataResponse.response = new MapMetadataResponse();
        HTTPMapResponse m = entry.getValue();
        metadataResponse.response.width = m.dimensions.cols;
        metadataResponse.response.height = m.dimensions.rows;
        double mapScaleX = m.dimensions.width / m.dimensions.cols;
        double mapScaleY = m.dimensions.height / m.dimensions.rows;
        metadataResponse.response.mapScaleX = mapScaleX;
        metadataResponse.response.mapScaleY = mapScaleY;
        break;
      }
    }

    return metadataResponse;
  }

  @Override
  public RMSResponse setVelocity(float translation, float rotation, float sidestep) {
    String uri = baseUrl + "/robot/drive/velocity";
    VelocityRequest request = new VelocityRequest(translation, rotation, sidestep);
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending setVelocity request");
    try {
      res = Http.sendPostRequest(uri, request, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    log.debug("RMSResponse: {}", res);

    return resObj;
  }

  /**
   * Send the robot to an arbitrary position on the map.
   *
   * @param x     position in x coordinates (meters)
   * @param y     position along y coordinates (meters)
   * @return status of the request
   */
  @Override
  public RMSResponse goToArbitrary(double x, double y) {
    String uri = baseUrl + "/robot/drive/destination";
    RMSDestinationPosition request = new RMSDestinationPosition(x, y);
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending goToArbitrary request");
    try {
      res = Http.sendPostRequest(uri, request, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    log.debug("RMSResponse: {}", res);

    return resObj;
  }

  /**
   * Send the robot to an arbitrary pose on the map.
   *
   * @param x     position in x coordinates (meters)
   * @param y     position along y coordinates (meters)
   * @param theta the z rotation of the robot (radians)
   * @return status of the request
   */
  @Override
  public RMSResponse goToArbitrary(double x, double y, double theta) {
    String uri = baseUrl + "/robot/drive/destination";
    RMSDestinationPose args = new RMSDestinationPose(x, y, theta);
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending goToArbitrary request");
    try {
      res = Http.sendPostRequest(uri, args, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    log.debug("RMSResponse: {}", res);

    return resObj;
  }

  /**
   * Gets the position of the robot at time of request.
   *
   * @return robot position information
   */
  @Override
  public RMSPositionResponse getPosition() {
    String uri = baseUrl + "/robot/drive/position";
    String res = null;
    log.debug("Sending getPosition request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSPositionResponse resObj = new RMSPositionResponse();
    resObj.response = gson.fromJson(res, PositionResponse.class);
    return resObj;
  }

  /**
   * Null method. Returns the status of the robot during a go to navigation
   * action to a named or tagged location. Not currently in use.
   *
   * @return status of go to location action
   */
  @Override
  public RMSGoToStatusResponse getGoToStatus() {
    String uri = baseUrl + "/robot/tel/goToStatus";
    //not in use
    return null;
  }

  /**
   * Get the health of the robot.
   *
   * @return Object containing robot health metadata
   */
  @Override
  public RMSHealthResponse getHealth() {
    String uri = baseUrl + "/robot/health";
    String res = null;
    log.debug("Sending health request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSHealthResponse resObj = new RMSHealthResponse();
    resObj.response = gson.fromJson(res, HealthResponse.class);
    return resObj;
  }

  /**
   * Get the Z lift position and camera tilt positon of the robot.
   *
   * @return Object containing z lift and camera tilt
   */
  @Override
  public RMSZLiftCameraTiltResponse getZLiftCameraTilt() {
    String uri = baseUrl + "/robot/drive/getZLiftCameraTilt";
    String res = null;
    log.debug("Sending getZLiftCameraTilt request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSZLiftCameraTiltResponse resObj = new RMSZLiftCameraTiltResponse();
    resObj.response = gson.fromJson(res, ZLiftCameraTiltResponse.class);
    return resObj;
  }

  /**
   * Get the estimate remaining runtime of the robot in minutes
   *
   * @return Object containing runtime estimate
   */
  @Override
  public RMSRuntimeResponse getRuntime() {
    String uri = baseUrl + "/robot/tel/runtime";
    String res = null;
    log.debug("Sending getRuntime request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSRuntimeResponse resObj = new RMSRuntimeResponse();
    resObj.response = gson.fromJson(res, RuntimeResponse.class);
    return resObj;
  }

  /**
   * Get the lidar scan from the robot
   *
   * @return Object containing lidar scan
   */
  @Override
  public RMSLidarResponse getLidar() {
    String uri = baseUrl + "/robot/perception/rangescan";
    String res = null;
    log.debug("Sending lidar request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSLidarResponse resObj = new RMSLidarResponse();
    resObj.response = gson.fromJson(res, LidarResponse.class);
    return resObj;
  }

  /**
   * Get list of dynamic obstacles
   *
   * @return Object containing list of dynamic obstacles
   */
  @Override
  public RMSDynamicObstaclesResponse getDynamicObstacles() {
    String uri = baseUrl + "/robot/perception/dynamicObstacles";
    String res = null;
    log.debug("Sending dynamic obstacles request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSDynamicObstaclesResponse resObj = new RMSDynamicObstaclesResponse();
    resObj.response = gson.fromJson(res, DynamicObstaclesResponse.class);
    return resObj;
  }

  /**
   * Get dock status
   *
   * @return Object containing dock status information
   */
  @Override
  public RMSDockStatusResponse getDockStatus() {
    String uri = baseUrl + "/robot/dock/status";
    String res = null;
    log.debug("Sending dock status request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
    Gson gson = new Gson();
    RMSDockStatusResponse resObj = new RMSDockStatusResponse();
    resObj.response = gson.fromJson(res, DockStatusResponse.class);
    return resObj;
  }

  /**
   * Stops the robot during any go to navigation action.
   *
   * @return response object
   */
  @Override
  public RMSResponse stop() {
    String uri = baseUrl + "/robot/drive/stopRobot";
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending stop request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    return resObj;
  }

  /**
   * Sends the robot to the dock after waiting 30s.
   *
   * @return response object
   */
  @Override
  public RMSResponse dock() {
    String uri = baseUrl + "/robot/dock/dock";
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending dock request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    return resObj;
  }

  /**
   * Set the robot Z lift position.
   *
   * @param position Z lift height
   * @return response object
   */
  @Override
  public RMSResponse setZLift(double position) {
    String uri = baseUrl + "/robot/drive/payloadPose";
    ZLiftArguments args = new ZLiftArguments(position);
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending setZLift request");
    try {
      res = Http.sendPostRequest(uri, args, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    return resObj;
  }

  /**
   * Set the robot camera tilt position.
   *
   * @param position camera tilt position
   * @return response object
   */
  @Override
  public RMSResponse setCameraTilt(double position) {
    String uri = baseUrl + "/robot/drive/payloadPose";
    CameraTiltArguments args = new CameraTiltArguments(position);
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending setCameraTilt request");
    try {
      res = Http.sendPostRequest(uri, args, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    return resObj;
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
    String uri = baseUrl + "/robot/kinematics/look";
    LookAtArguments args = new LookAtArguments(x, y, z, duration);
    String res = null;
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending lookAt request");
    try {
      res = Http.sendPostRequest(uri, args, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    return resObj;
  }

  /**
   * Returns whether the client has an active session, which in the
   * case of the HTTP client is always true.
   *
   * @return whether client has session
   */
  @Override
  public boolean hasSession() {
    return true;
  }

  /**
   * Returns whether the robot has any velocity values in the position
   * object.
   *
   * @return whether the robot is in motion
   */
  @Override
  public boolean isMoving() {
    String uri = baseUrl + "/robot/drive/position?includeVelocity=true";
    String res;
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      return false;
    }
    Gson gson = new Gson();
    VelocityResponse resObj = gson.fromJson(res, VelocityResponse.class);
    if (resObj.velocity.x > 0.0 || resObj.velocity.y > 0.0 || resObj.velocity.theta > 0.0) {
      return true;
    }
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
   * Rotates the robot on the Z axis.
   *
   * @param theta goal rotation
   * @return response object
   */
  @Override
  public RMSResponse setRotation(double theta) {
    String uri = baseUrl + "/robot/drive/localWaypoint";
    LocalWaypointArguments args = new LocalWaypointArguments(0f, 0f, (float) theta);
    RMSResponse resObj = new RMSResponse();
    log.debug("Sending setRotation request");
    try {
      Http.sendPostRequest(uri, args, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
      resObj.errorMessage = "Error making request to HTTP server";
    }
    return resObj;
  }

  /**
   * Get the map as image data (PNG). Only available on local
   * HTTP API.
   *
   * @return map data as image
   */
  public BufferedImage getMap() {
    String uri = baseUrl + "/robot/mapping/map/" + mapName;
    log.debug("Sending getMap request");
    return Http.sendGetImage(uri);
  }

  /**
   * Gets drive status
   *
   * @return Object containing various drive statuses
   */
  public RMSDriveStatusResponse getDriveStatus() {
    String uri = baseUrl + "/robot/drive/updates";
    String res = null;
    log.debug("Sending drive status request");
    try {
      res = Http.sendGetRequest(uri, null);
    } catch (Exception ex) {
      log.error("Error making request to " + uri, ex);
    }
    if (res == null) {
      return null;
    }
//        log.debug("[RMSDriveStatusResponse] response: {}", res);
    Gson gson = new Gson();
    RMSDriveStatusResponse resObj = new RMSDriveStatusResponse();
    resObj.response = gson.fromJson(res, DriveStatusResponse.class);
    return resObj;
  }
}