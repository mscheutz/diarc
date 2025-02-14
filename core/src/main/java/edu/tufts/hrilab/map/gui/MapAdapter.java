/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.ConfigurationHolder;
import edu.tufts.hrilab.gui.GuiAdapter;
import edu.tufts.hrilab.map.FloorMap;
import org.apache.commons.imaging.ImageFormats;
import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.ImageWriteException;
import org.apache.commons.imaging.Imaging;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.Map;

/**
 * Handles WebSocket messages for MapComponent-related functionalities within the web interface.
 * This class processes and responds to various navigation and location requests, converting
 * them into TRADEService calls and updates within the application.
 */
public class MapAdapter extends GuiAdapter {

  private FloorMap currFloorMap;
  private int currentFloor = -1;

  //==========================================================================
  // Constructors
  //==========================================================================
  public MapAdapter(Collection<String> groups) {
    super(groups);
  }

  //======================================================================
  // Implementing methods | TextWebSocketHandler
  //======================================================================

  /**
   * Handles incoming WebSocket messages by identifying the action requested
   * and delegating to the appropriate method.
   *
   * @param message The text message received from the client.
   */
  @Override
  protected void handleMessage(JsonObject message) {

    switch (message.get("action").getAsString()) {
      case "fetchMapData":
        fetchMapData();
        fetchKnownLocations();
        break;
      case "navigateToPoint":
        double x = message.get("x").getAsDouble();
        double y = message.get("y").getAsDouble();
        double quatX = message.get("quatX").getAsDouble();
        double quatY = message.get("quatY").getAsDouble();
        double quatZ = message.get("quatZ").getAsDouble();
        double quatW = message.get("quatW").getAsDouble();
        navigateToPoint(x, y, quatX, quatY, quatZ, quatW);
        break;
      case "goToLocation":
        String locationSymbol = message.get("locationSymbol").getAsString();
        goToLocation(Factory.createSymbol(locationSymbol));
        break;
      case "fetchRobotPose":
        fetchRobotPose();
        break;
      case "fetchKeyLocations":
        fetchKnownLocations();
        break;
      case "fetchPastLocations":
        log.error("fetchPastLocations not currently supported.");
        break;
      default:
        JsonObject response = new JsonObject();
        response.addProperty("error", "Unsupported action");
        response.addProperty("path", getPath());
        try {
          sendMessage(response);
        } catch (TRADEException e) {
          log.error("Failed to send message", e);
        }
    }
  }

  //======================================================================
  // Methods
  //======================================================================

  /**
   * Update local current FloorMap and current floor number.
   */
  private void fetchCurrentFloorMap() {
    try {
      currentFloor = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrentFloor").argTypes().inGroups(this.groups.toArray(new String[0])))
              .call(Integer.class);

      // only update the current floor map when not initialized or if not on the correct floor
      if (currFloorMap == null || currentFloor != currFloorMap.getFloorNumber()) {
        currFloorMap = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrentFloorMap").argTypes().inGroups(this.groups.toArray(new String[0])))
                .call(FloorMap.class);
      }
    } catch (TRADEException e) {
      log.error("Error getting map.", e);
      currentFloor = -1;
      currFloorMap = null;
    }
  }

  /**
   * Fetches and sends map data to the client, including current floor and associated metadata.
   */
  private void fetchMapData() {
    fetchCurrentFloorMap();

    JsonObject response = new JsonObject();
    if (currFloorMap != null && currFloorMap.getMapYamlFile() != null) {
      // TODO: this should use the pixelMap because the yaml/pgm files might not exist on this machine
      // Assumes corresponding .yaml and .pgm files share the same name and directory.
      String pgmFilePath = currFloorMap.getMapYamlFile().replace(".yaml", ".pgm");
      String pngFileName;
      try {
        pngFileName = convertPGMtoPNG(pgmFilePath);
        response.addProperty("currentFloor", currentFloor);
        String baseUrl = ConfigurationHolder.getBaseUrl();
        response.addProperty("mapImageUrl", baseUrl + "/images/" + pngFileName);
        response.addProperty("success", true);
        response.addProperty("message", "Map data fetched successfully.");
        response.addProperty("path", getPath());
      } catch (ImageReadException | ImageWriteException | IOException e) {
        log.error("Error converting map PGM to PNG");
        response.addProperty("success", false);
        response.addProperty("error", "Failed to retrieve map data.");
        response.addProperty("path", getPath());
      }
    } else {
      response.addProperty("success", false);
      response.addProperty("error", "Failed to retrieve map data.");
      response.addProperty("path", getPath());
    }

    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }

    // Fetch and send robot pose
    fetchRobotPose();
  }

  /**
   * Converts a PGM image file to a PNG image file and returns the new filename.
   *
   * @param pgmPathStr The path string of the PGM file to convert.
   * @return The filename of the converted PNG image.
   * @throws ImageReadException, ImageWriteException, IOException Throws if reading from or writing to the file system fails.
   */
  public String convertPGMtoPNG(String pgmPathStr) throws ImageReadException, ImageWriteException, IOException {
    Path pgmPath = Paths.get(pgmPathStr);
    String pngFilename = pgmPath.getFileName().toString().replace(".pgm", ".png");

    // Calculate the output directory based on the current working directory
    String outputDirectory = Paths.get("../core/src/main/resources/static/images/").toAbsolutePath().normalize() + "/";

    Path pngPath = Paths.get(outputDirectory, pngFilename);

    log.info("Reading PGM file from: {}", pgmPath);
    BufferedImage image = Imaging.getBufferedImage(pgmPath.toFile());

    log.info("Writing PNG file to: {}", pngPath);
    Imaging.writeImage(image, pngPath.toFile(), ImageFormats.PNG);

    log.info("Converted PGM to PNG: {} to {}", pgmPath, pngPath);

    return pngFilename;
  }

  /**
   * Initiates navigation to a specified point and orientation based on client requests.
   *
   * @param x     The x-coordinate to navigate to.
   * @param y     The y-coordinate to navigate to.
   * @param quatX The x-component of the quaternion representing orientation.
   * @param quatY The y-component of the quaternion representing orientation.
   * @param quatZ The z-component of the quaternion representing orientation.
   * @param quatW The w-component (scalar) of the quaternion representing orientation.
   */
  private void navigateToPoint(double x, double y, double quatX, double quatY, double quatZ, double quatW) {
    log.info("Initiating navigation to coordinates: x={}, y={}, and quaternion: quatX={}, quatY={}, quatZ={}, quatW={}", x, y, quatX, quatY, quatZ, quatW);
    JsonObject response = new JsonObject();

    try {
      // Fetch the trade service information for navigation
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("goToLocation").inGroups(this.groups.toArray(new String[0]))
              .argTypes(Double.class, Double.class, Double.class, Double.class, Double.class, Double.class, Boolean.class));

      Point2d pixelPosition = new Point2d(x, y);
      FloorMap currFloorMap = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrentFloorMap").inGroups(this.groups.toArray(new String[0]))).call(FloorMap.class);
      Point3d meterPosition = currFloorMap.getPixelMap().toMeter(pixelPosition);
      x = meterPosition.getX();
      y = meterPosition.getY();

      Justification result = tsi.call(Justification.class, x, y, quatX, quatY, quatZ, quatW, true);

      // Check the result of the navigation attempt
      if (result != null && result.getValue()) {
        response.addProperty("success", true);
        response.addProperty("message", "Navigation to point initiated successfully.");
        log.info("Navigation to point successfully initiated.");
        fetchRobotPose();  // Call to fetch robot pose after successful navigation
      } else {
        response.addProperty("success", false);
        response.addProperty("message", "Failed to initiate navigation.");
        log.warn("Failed to initiate navigation to point.");
      }
    } catch (TRADEException e) {
      log.error("TRADE service call failed: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("message", "TRADE service call failed: " + e.getMessage());
    }

    // Send the response back to the client
    response.addProperty("path", getPath());
    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }
  }

  /**
   * Navigates to a known location identified by a Symbol reference.
   */
  private void goToLocation(Symbol location) {
    log.info("Attempting to navigate to location: {}", location);
    JsonObject response = new JsonObject();

    try {
      // Call MoveBase TRADEService goToLocation
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("goToLocation")
              .inGroups(this.groups.toArray(new String[0]))
              .argTypes(Symbol.class, Boolean.class));

      Justification result = tsi.call(Justification.class, location, true);

      // Check the result of the navigation attempt
      if (result != null && result.getValue()) {
        response.addProperty("success", true);
        response.addProperty("message", "Navigation to location initiated successfully.");
        log.info("Navigation to location {} initiated successfully.", location);
        fetchRobotPose();  // Call to fetch robot pose after successful navigation
      } else {
        response.addProperty("success", false);
        response.addProperty("message", "Failed to initiate navigation to location.");
        log.warn("Failed to initiate navigation to location {}.", location);
      }
    } catch (TRADEException e) {
      log.error("Service call failed: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("message", "Service call failed: " + e.getMessage());
    }

    // Send the response to the client
    response.addProperty("path", getPath());
    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }
  }

  /**
   * Fetches and sends the current pose of the robot to the client.
   */
  private void fetchRobotPose() {
    JsonObject response = new JsonObject();

    // get current robot pose
    Pair<Point3d, Quat4d> currPose = null;
    try {
      currPose = TRADE.getAvailableService(new TRADEServiceConstraints()
                      .name("getPose").argTypes().inGroups(this.groups.toArray(new String[0])))
              .call(Pair.class);

      if (currPose == null) {
        response.addProperty("success", false);
        response.addProperty("error", "Failed to fetch robot pose.");
      } else {
        Point3d position = currPose.getLeft();
        Quat4d orientation = currPose.getRight();

        JsonObject positionJson = new JsonObject();
        positionJson.addProperty("x", position.getX());
        positionJson.addProperty("y", position.getY());
        positionJson.addProperty("z", position.getZ());
        JsonObject orientationJson = new JsonObject();
        positionJson.addProperty("x", orientation.getX());
        positionJson.addProperty("y", orientation.getY());
        positionJson.addProperty("z", orientation.getZ());
        positionJson.addProperty("w", orientation.getW());

        response.add("position", positionJson);
        response.add("orientation", orientationJson);

        response.addProperty("success", true);
        response.addProperty("message", "Robot pose fetched successfully.");
      }
    } catch (TRADEException e) {
      log.error("Error fetching robot pose.", e);
    }

    // get current pixel location on map
    if (currFloorMap == null || currFloorMap.getPixelMap() == null || currPose == null) {
      response.add("robotPixelPosition", JsonNull.INSTANCE);
    } else {
      // PixelMap
      Point2d robotPixelPosition = currFloorMap.getPixelMap().toPixel(currPose.getLeft());

      // Add the robot's pixel position to the response
      JsonObject pixlePos = new JsonObject();
      pixlePos.addProperty("x", robotPixelPosition.getX());
      pixlePos.addProperty("y", robotPixelPosition.getY());
      response.add("robotPixelPosition", pixlePos);
    }

    // send response
    response.addProperty("path", getPath());
    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }
  }

  private void fetchKnownLocations() {
    JsonObject response = new JsonObject();
    JsonObject knownPosesJson = new JsonObject();

    try {
      // Fetch the description from the TRADE service
      Map<Symbol, PoseReference> knownPoses = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapReferences").argTypes().inGroups(this.groups.toArray(new String[0])))
              .call(Map.class);

      // Loop through each line to extract information
      for (Symbol refId : knownPoses.keySet()) {
        PoseReference ref = knownPoses.get(refId);

        JsonObject positionJson = new JsonObject();
        positionJson.addProperty("x", ref.getPose().getLeft().x);
        positionJson.addProperty("y", ref.getPose().getLeft().y);
        positionJson.addProperty("description", ref.properties.toString());

        knownPosesJson.add(refId.toString(), positionJson);
      }
      response.add("keyLocations", knownPosesJson);

      response.addProperty("success", true);
      response.addProperty("message", "Key locations fetched successfully.");

    } catch (Exception e) {
      log.error("Error fetching key locations: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("error", "Error during ref data retrieval: " + e.getMessage());
    }

    // Send the final JsonObject back to the client
    response.addProperty("path", getPath());
    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  public String getPathRoot() {
    return "map";
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected boolean providesTradeServices() {
    return false;
  }
}
