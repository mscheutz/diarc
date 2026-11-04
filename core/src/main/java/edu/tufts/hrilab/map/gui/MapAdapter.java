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
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.ConfigurationHolder;
import edu.tufts.hrilab.gui.GuiAdapter;
import edu.tufts.hrilab.map.FloorMap;
import edu.tufts.hrilab.map.MapObject;
import edu.tufts.hrilab.map.util.Pose;
import org.apache.commons.imaging.ImageFormats;
import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.ImageWriteException;
import org.apache.commons.imaging.Imaging;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Handles WebSocket messages for MapComponent-related functionalities within the web interface.
 * This class processes and responds to various navigation and location requests, converting
 * them into TRADEService calls and updates within the application.
 */
public class MapAdapter extends GuiAdapter {

  //==========================================================================
  // Constructors
  //==========================================================================
  public MapAdapter(Collection<String> groups) {
    super(groups);
  }

  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Static logger instance.
   */
  private static final Logger log = LoggerFactory.getLogger(MapAdapter.class);

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
        fetchRefs();
        fetchMapData();
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
        fetchKeyLocations();
        break;
      case "fetchPastLocations":
        fetchPastLocations();
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
   * Fetches and sends map data to the client, including current floor and associated metadata.
   */
  private void fetchMapData() {
    log.info("Fetching map data for the current floor.");
    JsonObject response = new JsonObject();

    try {
      int currentFloor = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrFloor").inGroups(this.groups.toArray(new String[0]))).call(Integer.class);
      @SuppressWarnings("unchecked")
      Map<Integer, FloorMap> floorMaps = (Map<Integer, FloorMap>)
              TRADE.getAvailableService(
                      new TRADEServiceConstraints()
                              .name("getFloorMaps")
                              .inGroups(this.groups.toArray(new String[0]))
              ).call(Map.class);
      FloorMap currentMap = floorMaps.get(currentFloor);

      // Assumes corresponding .yaml and .pgm files share the same name and directory.
      String pgmFilePath = currentMap.getMapYamlFile().replace(".yaml", ".pgm");

      log.info("Converting PGM to PNG for file: {}", pgmFilePath);

      String pngFileName = convertPGMtoPNG(pgmFilePath);
      response.addProperty("currentFloor", currentFloor);
      String baseUrl = ConfigurationHolder.getBaseUrl();
      response.addProperty("mapImageUrl", baseUrl + "/images/" + pngFileName);
      response.addProperty("success", true);
      response.addProperty("message", "Map data fetched successfully.");
      // Send the initial map data response
      response.addProperty("path", getPath());
      try {
        sendMessage(response);
      } catch (TRADEException e) {
        log.error("Failed to send message", e);
      }

    } catch (Exception e) {
      log.error("Failed to retrieve or convert map data: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("error", "Failed to retrieve map data: " + e.getMessage());
      response.addProperty("path", getPath());
      try {
        sendMessage(response);
      } catch (TRADEException ex) {
        log.error("Failed to send message", ex);
      }
    }

    // Fetch and send robot pose
    fetchRobotPose();

    // Fetch and send key/past locations
    fetchKeyLocations();
    fetchPastLocations();

  }

  /**
   * Converts a PGM image file to a PNG image file and returns the new filename.
   *
   * @param pgmPathStr The path string of the PGM file to convert.
   * @return The filename of the converted PNG image.
   * @throws ImageReadException
   * @throws ImageWriteException
   * @throws IOException
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
      FloorMap currFloorMap = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrFloorMap").inGroups(this.groups.toArray(new String[0]))).call(FloorMap.class);
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

        // Create new location and add to past locations
        TRADE.getAvailableService(new TRADEServiceConstraints()
                        .name("recordLocation").inGroups(this.groups.toArray(new String[0]))
                        .argTypes(Point3d.class))
                .call(void.class, meterPosition);

        fetchPastLocations();
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
    log.info("Fetching robot pose.");
    JsonObject response = new JsonObject();

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("updateRobotPose").inGroups(this.groups.toArray(new String[0]))).call(void.class);
      Pose currPose = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrRobotPose").inGroups(this.groups.toArray(new String[0]))).call(Pose.class);
      Point3d position = currPose.getPosition();
      Quat4d orientation = currPose.getOrientation();

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

      try {
        // FloorMap (requires a map)
        FloorMap currFloorMap = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrFloorMap").inGroups(this.groups.toArray(new String[0]))).call(FloorMap.class);
        // PixelMap
        Point2d robotPixelPosition = currFloorMap.getPixelMap().toPixel(position);

        // Add the robot's pixel position to the response
        JsonObject pixlePos = new JsonObject();
        pixlePos.addProperty("x", robotPixelPosition.getX());
        pixlePos.addProperty("y", robotPixelPosition.getY());
        response.add("robotPixelPosition", pixlePos);
      } catch (Exception e) {
        // Handle the case where the FloorMap or PixelMap is not available
        response.add("robotPixelPosition", JsonNull.INSTANCE);
        log.error("Failed to retrieve the FloorMap or PixelMap: {}", e.getMessage(), e);
      }

      response.addProperty("success", true);
      response.addProperty("message", "Robot pose fetched successfully.");
    } catch (Exception e) {
      log.error("Error fetching robot pose: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("error", "Failed to fetch robot pose: " + e.getMessage());
    }

    response.addProperty("path", getPath());
    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }
  }

  /**
   * Fetches and sends information about key locations and their properties to the client.
   */
  private void fetchKeyLocations() {
    log.info("Fetching key locations from FloorMap.");

    JsonObject response = new JsonObject();
    JsonObject finalLocationsWithPositions = new JsonObject();

    try {
      FloorMap currFloorMap = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrFloorMap").inGroups(this.groups.toArray(new String[0]))).call(FloorMap.class);
      Set<MapObject> objects = currFloorMap.getAllObjects();
      if (objects != null) {
        log.info("Fetched {} objects from the current floor map.", objects.size());

        for (MapObject object : objects) {
          Point2d pixelPosition = object.getCenterPixel();
          Symbol symbol = object.getReferenceId();

          // Get the full description and modify it
          String description = object.toString();
          Point3d position = object.getPosition();
          // Regex to find and replace the coordinate part of the description
          description = description.replaceAll("\\(\\d+\\.\\d+, \\d+\\.\\d+\\)$", "(" + position.getX() + ", " + position.getY() + ")");

          JsonObject positionJson = new JsonObject();
          positionJson.addProperty("x", pixelPosition.getX());
          positionJson.addProperty("y", pixelPosition.getY());
          positionJson.addProperty("description", description);

          finalLocationsWithPositions.add(symbol.toString(), positionJson);
        }

        response.add("keyLocations", finalLocationsWithPositions);
        response.addProperty("success", true);
        response.addProperty("message", "Key locations fetched successfully.");
      } else {
        log.error("No objects found in the current floor map.");
        response.addProperty("success", false);
        response.addProperty("error", "No objects available in the current floor map.");
      }
    } catch (Exception e) {
      log.error("Error fetching key locations: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("error", "Error during map data retrieval: " + e.getMessage());
    }

    // Send the final JsonObject back to the client
    response.addProperty("path", getPath());
    try {
      sendMessage(response);
    } catch (TRADEException ex) {
      log.error("Failed to send message", ex);
    }
  }

  private void fetchRefs() {
    log.info("Fetching key locations from refs.");

    JsonObject response = new JsonObject();
    JsonObject finalLocationsWithPositions = new JsonObject();

    try {
      // Fetch the description from the TRADE service
      String description = TRADE.getAvailableService(new TRADEServiceConstraints().name("getRefs").inGroups(this.groups.toArray(new String[0]))).call(String.class);

      // Split the description into lines
      String[] lines = description.split("\n");

      // Loop through each line to extract information
      for (String line : lines) {
        // Splitting by '=' to separate the key part from the value part
        String[] parts = line.split("=", 2);  // Limit split to 2 parts to ensure only the first '=' is used as the delimiter
        if (parts.length == 2) {
          String refId = parts[0].trim();   // "movebaselocation_4:movebaselocation"
          String properties = parts[1].trim(); // "[tableELocation(VAR0:movebaselocation)] (not null)"

          Symbol symbol = Factory.createSymbol(refId);

          JsonObject positionJson = new JsonObject();
          positionJson.add("x", JsonNull.INSTANCE);
          positionJson.add("y", JsonNull.INSTANCE);

          positionJson.addProperty("description", properties);

          finalLocationsWithPositions.add(symbol.toString(), positionJson);
        }
      }

      response.add("keyLocations", finalLocationsWithPositions);
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
   * Retrieves and sends past robot locations to the client.
   */
  private void fetchPastLocations() {
    log.info("Fetching past locations.");

    JsonObject response = new JsonObject();

    Map<Symbol, Pair<Point3d, Quat4d>> storedPoses = new HashMap<>(); // Initialize storedPoses

    try {
      @SuppressWarnings("unchecked")
      Map<Symbol, Pair<Point3d, Quat4d>> tempStoredPoses = (Map<Symbol, Pair<Point3d, Quat4d>>)
              TRADE.getAvailableService(
                      new TRADEServiceConstraints()
                              .name("getStoredPoses")
                              .inGroups(this.groups.toArray(new String[0]))
              ).call(Map.class);
      storedPoses = tempStoredPoses;
    } catch (TRADEException e) {
      log.error("Failed to retrieve stored poses: {}", e.getMessage(), e);
    } catch (Exception e) {
      log.error("An unexpected error occurred: {}", e.getMessage(), e);
    }

    // Early exit if no locations are stored
    // initial location is excluded from "pastLocations"
    if (storedPoses.isEmpty()) {
      response.add("pastLocations", new JsonObject()); // Empty object for no locations
      response.addProperty("path", getPath());
      try {
        sendMessage(response);
      } catch (TRADEException ex) {
        log.error("Failed to send message", ex);
      }
      return;
    }

    JsonObject finalLocationsWithPositions = new JsonObject();
    try {
      for (Map.Entry<Symbol, Pair<Point3d, Quat4d>> entry : storedPoses.entrySet()) {
        Symbol symbol = entry.getKey();
        Pair<Point3d, Quat4d> pose = entry.getValue();
        Point3d position = pose.getLeft();

        FloorMap currFloorMap = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrFloorMap").inGroups(this.groups.toArray(new String[0]))).call(FloorMap.class);
        Point2d robotPixelPosition = currFloorMap.getPixelMap().toPixel(position);

        JsonObject positionJson = new JsonObject();
        positionJson.addProperty("x", robotPixelPosition.getX());
        positionJson.addProperty("y", robotPixelPosition.getY());

        finalLocationsWithPositions.add(symbol.toString(), positionJson);
      }

      response.add("pastLocations", finalLocationsWithPositions);
      response.addProperty("success", true);
      response.addProperty("message", "Past locations fetched successfully.");
    } catch (Exception e) {
      log.error("Error fetching past locations: {}", e.getMessage(), e);
      response.addProperty("success", false);
      response.addProperty("error", "Error during map data retrieval: " + e.getMessage());
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
