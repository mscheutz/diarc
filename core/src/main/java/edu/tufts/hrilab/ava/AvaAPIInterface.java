/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import edu.tufts.hrilab.ava.message.response.*;

import java.util.List;

/**
 * Represents the API of the Ava robot. Abstracts the
 * cloud-based RMS websocket API and the local HTTP APIs.
 *
 * @author Matt McWilliams
 */
public interface AvaAPIInterface {

  void end();

  RMSResponse setVelocity(float transVel, float rotVel, float stepVel);

  RMSResponse setVelocity(float transVel, float rotVel, float stepVel, boolean nudge);

  RMSResponse goToArbitrary(double x, double y);

  RMSResponse goToArbitrary(double x, double y, double theta);

  RMSResponse dock();

  RMSResponse stop();

  RMSPositionResponse getPosition();

  RMSDriveStatusResponse getDriveStatus();

  RMSGoToStatusResponse getGoToStatus();

  RMSHealthResponse getHealth();

  RMSMapMetadataResponse getMapMetadata();
  RMSMapMetadataResponse getMapMetadata(String mapName);
  List<RMSMapMetadataResponse> getMapMetadatas();

  RMSZLiftCameraTiltResponse getZLiftCameraTilt();

  RMSRuntimeResponse getRuntime();

  RMSLidarResponse getLidar();

  RMSDynamicObstaclesResponse getDynamicObstacles();

  RMSDockStatusResponse getDockStatus();

  RMSResponse lookAt(double x, double y, double z, double duration);

  //getDatabases
  //getDatabase
  double getBatteryLevel();

  RMSResponse setZLift(double position);

  RMSResponse setCameraTilt(double position);

  RMSResponse setRotation(double theta);

  boolean hasSession();

  boolean isMoving();

}