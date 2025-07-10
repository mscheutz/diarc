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
import edu.tufts.hrilab.ava.message.response.RMSMapMetadataResponse;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.ConfigurationHolder;
import edu.tufts.hrilab.gui.GuiAdapter;
import org.apache.commons.imaging.ImageFormats;
import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.ImageWriteException;
import org.apache.commons.imaging.Imaging;
import org.apache.commons.lang3.tuple.Pair;

import javax.imageio.ImageIO;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.List;

/**
 * Handles WebSocket messages for MapComponent-related functionalities within the web interface.
 * This class processes and responds to various navigation and location requests, converting
 * them into TRADEService calls and updates within the application.
 */
public class AvaMapAdapter extends GuiAdapter {

  private List<BufferedImage> mapImages = new ArrayList<>();
  private List<RMSMapMetadataResponse> mapMetadatas = new ArrayList<>();
  private BufferedImage finalMap = null;
  int finalMapWidth;
  int finalMapHeight;
  int finalMapOriginX;
  int finalMapOriginY;
  double mapScaleX;
  double mapScaleY;
  private boolean mock = false;
  private boolean runRobotPosThread = true;

  //==========================================================================
  // Constructors
  //==========================================================================
  public AvaMapAdapter(Collection<String> groups) {
    super(groups);
    startRobotPosThread();
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
      case "startup":
      case "fetchMapData":
        fetchMapData();
        fetchKnownLocations();
        break;
      case "navigateToPoint":
        //double x = message.get("x").getAsDouble();
        //double y = message.get("y").getAsDouble();
        //double quatX = message.get("quatX").getAsDouble();
        //double quatY = message.get("quatY").getAsDouble();
        //double quatZ = message.get("quatZ").getAsDouble();
        //double quatW = message.get("quatW").getAsDouble();
        //navigateToPoint(x, y, quatX, quatY, quatZ, quatW);
        log.error("[handleMessage] navigateToPoint not implemented");
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

  //TODO: need to update map tsx to handle live updates gracefully
  private void startRobotPosThread() {
    new Thread(() -> {
      while (runRobotPosThread) {
        try {
          Thread.sleep(5000);
          fetchRobotPose();
        } catch (InterruptedException e) {
          log.error("[listenForWWandASR] exception sleeping in clearDisplay thread");
        }
      }
    }).start();
  }

  /**
   * Update local current FloorMap and current floor number.
   */
  private void fetchMapImage() {
    mapImages = new ArrayList<>();
    try {
      List<byte[]> rawData = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapImages").argTypes().inGroups(this.groups.toArray(new String[0]))).call(List.class);
      for (byte[] data : rawData) {
        ByteArrayInputStream bais = new ByteArrayInputStream(data);
        mapImages.add(ImageIO.read(bais));
      }
    } catch (TRADEException | IOException e) {
      log.error("Error getting map.", e);
    }
  }

  private void getMapMetadata() {
    try {
      mapMetadatas = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapMetadatas").argTypes().inGroups(this.groups.toArray(new String[0]))).call(List.class);
    } catch (TRADEException e) {
      log.error("Error getting map.", e);
      mapMetadatas = new ArrayList<>();
    }
  }

  private BufferedImage getFinalMap() {
    if (mapImages.isEmpty()) {
      return null;
    }
    //TODO: Revisit rounding throughout this function
    mapScaleX = mapMetadatas.get(0).response.mapScaleX;
    mapScaleY = mapMetadatas.get(0).response.mapScaleY;
    if (mapImages.size() == 1) {
      finalMapOriginX = (int)(mapMetadatas.get(0).response.originX/mapScaleX);
      finalMapOriginY = (int)(mapMetadatas.get(0).response.originY/mapScaleY);
      finalMapHeight = mapMetadatas.get(0).response.height;
      finalMapWidth = mapMetadatas.get(0).response.width;
      finalMap = mapImages.get(0);
      return mapImages.get(0);
    }
    double minX = 999999;
    double maxX = -999999;
    double minY = 999999;
    double maxY = -999999;
    List<Double> minXs = new ArrayList<>();
    List<Double> maxYs = new ArrayList<>();
    for (RMSMapMetadataResponse metadata : mapMetadatas) {
      double originX = metadata.response.originX;
      double originY = metadata.response.originY;
      double scaleX = metadata.response.mapScaleX;
      double scaleY = metadata.response.mapScaleY;
      int width = metadata.response.width;
      int height = metadata.response.height;
      double mapMinX = (originX/scaleX)-width*0.5;
      double mapMaxX = (originX/scaleX)+width*0.5;
      double mapMinY = (originY/scaleY)-height*0.5;
      double mapMaxY = (originY/scaleY)+height*0.5;
      minX = Math.min(minX, mapMinX);
      maxX = Math.max(maxX, mapMaxX);
      minY = Math.min(minY, mapMinY);
      maxY = Math.max(maxY, mapMaxY);
      minXs.add(mapMinX);
      maxYs.add(mapMaxY);
    }
    finalMapWidth = (int)(maxX - minX);
    finalMapHeight = (int)(maxY - minY);
    finalMapOriginX = (int)(minX + maxX)/2;
    finalMapOriginY = (int)(minY + maxY)/2;
    BufferedImage merged = new BufferedImage(finalMapWidth, finalMapHeight, BufferedImage.TYPE_BYTE_BINARY);
    Graphics mergedGraphics = merged.getGraphics();
    for (int i = 0; i < mapImages.size(); i++) {
      mergedGraphics.drawImage(mapImages.get(i),finalMapWidth/2 + (minXs.get(i).intValue()-finalMapOriginX), finalMapHeight/2 - (maxYs.get(i).intValue()-finalMapOriginY), null);
    }
    finalMap = merged;
    return merged;
  }

  /**
   * Fetches and sends map data to the client, including current floor and associated metadata.
   */
  private void fetchMapData() {
    getMapMetadata();
    fetchMapImage();
    getFinalMap();

    JsonObject response = new JsonObject();
    if (!mapImages.isEmpty() && !mapMetadatas.isEmpty()) {
      try {
        String pngFileName = saveMapAsPNG(finalMap);
        response.addProperty("currentFloor", 0);
        String baseUrl = ConfigurationHolder.getBaseUrl();
        response.addProperty("mapImageUrl", baseUrl + "/images/" + pngFileName);
        response.addProperty("success", true);
        response.addProperty("message", "Map data fetched successfully.");
        response.addProperty("path", getPath());
      } catch (ImageReadException | ImageWriteException | IOException e) {
        log.error("Error converting map PGM to PNG", e);
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
   * @return The filename of the converted PNG image.
   * @throws ImageReadException
   * @throws ImageWriteException
   * @throws IOException
   */
  public String saveMapAsPNG(BufferedImage map) throws ImageReadException, ImageWriteException, IOException {

    // Calculate the output directory based on the current working directory
    String outputDirectory = Paths.get("../core/src/main/resources/static/images/ava").toAbsolutePath().normalize() + "/";
    Path pngPath = Paths.get(outputDirectory, "map.png");

    Imaging.writeImage(map, pngPath.toFile(), ImageFormats.PNG);

    log.info("saved PNG: {}", pngPath);

    return "ava/map.png";
  }

  private Point2d mapToPixel(Point3d mapPosition) {
    //TODO: does mapScale ever meaningfully change or vary across maps?
    return new Point2d((mapPosition.getX()/mapScaleX-finalMapOriginX) + finalMapWidth*0.5, (-mapPosition.getY()/mapScaleY+finalMapOriginY) + finalMapHeight*0.5);
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
        fetchRobotPose();
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
    if (mapImages.isEmpty() || mapMetadatas.isEmpty() || currPose == null) {
      response.add("robotPixelPosition", JsonNull.INSTANCE);
    } else {
      // PixelMap
      Point2d robotPixelPosition = mapToPixel(currPose.getLeft());

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
      Map<Symbol, Point3d> knownPoses = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLocationsOnMap").argTypes().inGroups(this.groups.toArray(new String[0])))
              .call(Map.class);

      Map<Symbol, String> poseNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLocationNames").argTypes(List.class).inGroups(this.groups.toArray(new String[0])))
              .call(Map.class, knownPoses.keySet().stream().toList());
      // Loop through each line to extract information
      for (Symbol refId : knownPoses.keySet()) {
        Point3d refPose = knownPoses.get(refId);

        JsonObject positionJson = new JsonObject();
        Point2d pixelPosition = mapToPixel(refPose);
        positionJson.addProperty("x", pixelPosition.x);
        positionJson.addProperty("y", pixelPosition.y);
        positionJson.addProperty("description", poseNames.get(refId));
        knownPosesJson.add(refId.getName(), positionJson);
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

  public void shutdown() {
    runRobotPosThread = false;
  }
}
