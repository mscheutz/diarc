/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.webgui;

import ai.thinkingrobots.trade.*;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.gui.GuiAdapter;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.vision.util.CompressionUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.*;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.*;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import java.util.zip.DataFormatException;

public class VisionAdapter extends GuiAdapter {

  //==========================================================================
  // Constructors
  //==========================================================================
  public VisionAdapter(Collection<String> groups) {
    super(groups);
  }

  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Static logger instance.
   */
  private static final Logger logger = LoggerFactory.getLogger(VisionAdapter.class);

  /**
   * Update period (time between frames) for the video stream.
   * <p>
   * TODO: This is actually capped by getJNIFrame.
   */
  private static final long UPDATE_PERIOD = 1000 / 30; // Increasing the frequency to 30 FPS

  /**
   * Control flag for video streaming.
   */
  private volatile boolean isVideoStreaming = false;

  /**
   * Control flag for depth streaming.
   */
  private volatile boolean isDepthStreaming = false;

  /**
   * Scheduler for handling periodic tasks.
   */
  private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

  /**
   * Map to store typeId and its running status.
   */
  Map<Long, Boolean> typeIdToIsRunning = new HashMap<>();

  /**
   * Dimension of the captured image.
   */
  private Dimension imageSize = null;

  /**
   * Raster for video frames.
   */
  private WritableRaster videoRaster = null;

  /**
   * BufferedImage for video frames.
   */
  private BufferedImage videoBufferedImage = null;

  /**
   * Raster for depth frames.
   */
  private WritableRaster depthRaster = null;

  /**
   * BufferedImage for depth frames.
   */
  private BufferedImage depthBufferedImage = null;

  //======================================================================
  // Methods
  //======================================================================

  /**
   * Helper method to get image size from the vision component and initialize related objects.
   */
  private void initializeImageSize() {
    try {
      TRADEServiceInfo imgSizeTsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageSize").argTypes());
      imageSize = imgSizeTsi.call(Dimension.class);

      // Initialize video raster and buffered image
      videoRaster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE, imageSize.width, imageSize.height, imageSize.width * 3, 3, new int[]{0, 1, 2}, null);
      videoBufferedImage = new BufferedImage(imageSize.width, imageSize.height, BufferedImage.TYPE_3BYTE_BGR);
      videoBufferedImage.setData(videoRaster);

      // Initialize depth raster and buffered image
      depthRaster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE, imageSize.width, imageSize.height, imageSize.width * 4, 4, new int[]{0, 1, 2, 3}, null);
      depthBufferedImage = new BufferedImage(imageSize.width, imageSize.height, BufferedImage.TYPE_4BYTE_ABGR);
      depthBufferedImage.setData(depthRaster);
    } catch (TRADEException e) {
      logger.warn("Failed to call getImageSize.", e);
    }
  }

  /**
   * Stops video streaming.
   */
  private void stopStreaming() {
    isVideoStreaming = false;
  }

  /**
   * Starts video streaming.
   */
  private void startStreaming() {
    stopDepthStreaming(); // Ensure depth streaming is stopped
    isVideoStreaming = true;
    isDepthStreaming = false;

    if (imageSize == null) {
      initializeImageSize();
      if (imageSize == null) {
        logger.error("Failed to retrieve image size.");
        return;
      }
    }

    scheduler.scheduleAtFixedRate(() -> {
      if (!isVideoStreaming) {
        return;
      }
      try {
        //get image data
        byte[] frameComp;
        try {
          frameComp = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFrame").argTypes()).call(byte[].class);
        } catch (TRADEException e) {
          logger.warn("Failed to call getFrame.", e);
          return;
        }
        byte[] frame;
        try {
          frame = CompressionUtil.decompress(frameComp);
        } catch (IOException | DataFormatException e) {
          logger.error("Error decompressing image.", e);
          return;
        }

        videoRaster.setDataElements(0, 0, imageSize.width, imageSize.height, frame);
        videoBufferedImage.setData(videoRaster);

        String b64 = encodeImageToBase64(videoBufferedImage, "jpg");
        JsonObject message = new JsonObject();
        message.addProperty("imageData", b64);

        // Directly fetch
        // see   public static List<MemoryObject> getTokens(final double conf) in ShortTermMemoryInterface
        // and   public List<MemoryObject> getTokens(final double conf) in VisionComponent.
        @SuppressWarnings("unchecked")
        List<MemoryObject> newMemoryObjects = (List<MemoryObject>) TRADE.getAvailableService(
                new TRADEServiceConstraints()
                        .name("getTokens")
                        .argTypes()
        ).call(List.class);

        JsonArray detectionsArray = new JsonArray();

        // Iterate over all memory objects and convert them to JSON
        for (MemoryObject detection : newMemoryObjects) {
          JsonObject detectionData = detection.toJson();
          detectionsArray.add(detectionData);
        }

        // Only add the array of detections to the message if there are any detections
        if (detectionsArray.size() != 0) {
          message.add("detectionData", detectionsArray);
        }

        // Send the message
//                logger.debug("Fetch Time: {} | Detection Data: {}", System.currentTimeMillis(), detectionsArray);
        message.addProperty("path", getPath());
        try {
          sendMessage(message);
        } catch (TRADEException e) {
          log.error("Failed to send message", e);
        }
      } catch (IOException e) {
        logger.error("Failed to send video frame", e);
      } catch (Exception e) {
        logger.error("Error fetching or processing detection data", e);
      }
    }, 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS);
  }

  /**
   * Encodes a video frame as a Base64 string.
   *
   * @param image  the image to encode
   * @param format the format of the image (e.g., "jpg", "png")
   * @return Base64-encoded string
   * @throws IOException if an error occurs while encoding
   */
  private String encodeImageToBase64(BufferedImage image, String format) throws IOException {
    ByteArrayOutputStream baos = new ByteArrayOutputStream();
    ImageIO.write(image, format, baos);
    byte[] imageBytes = baos.toByteArray();
    return Base64.getEncoder().encodeToString(imageBytes);
  }

  /**
   * Save the current frame to file. Invokes takeSnapshot @TRADESerice method
   */
  private void takeSnapshot() {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("takeSnapshot").argTypes(String.class)).call(void.class, (Object) null);
    } catch (TRADEException e) {
      logger.warn("Failed to call takeSnapshot.", e);
    }
  }

  /**
   * Stops depth streaming.
   */
  private void stopDepthStreaming() {
    isDepthStreaming = false;
  }

  /**
   * Starts depth streaming.
   */
  private void startDepthStreaming() {
    stopStreaming(); // Ensure video streaming is stopped
    isVideoStreaming = false;
    isDepthStreaming = true;

    if (imageSize == null) {
      initializeImageSize();
      if (imageSize == null) {
        logger.error("Failed to retrieve image size.");
        return;
      }
    }

    scheduler.scheduleAtFixedRate(() -> {
      if (!isDepthStreaming) {
        return;
      }
      try {
        //get depth image data
        byte[] frameComp;
        try {
          frameComp = TRADE.getAvailableService(new TRADEServiceConstraints().name("getDepthFrame").argTypes()).call(byte[].class);
        } catch (TRADEException e) {
          logger.warn("Failed to call getFrame.", e);
          return;
        }
        byte[] frame;
        try {
          frame = CompressionUtil.decompress(frameComp);
        } catch (IOException | DataFormatException e) {
          logger.error("Error decompressing image.", e);
          return;
        }

        depthRaster.setDataElements(0, 0, imageSize.width, imageSize.height, frame);
        depthBufferedImage.setData(depthRaster);

        // Convert depth data to a color map
        BufferedImage colorMapImage = createColorMapImage(depthBufferedImage);

        // encode the depth frame
        String depthB64 = encodeImageToBase64(colorMapImage, "png");
        JsonObject message = new JsonObject();
        message.addProperty("depthData", depthB64);
        message.addProperty("path", getPath());
        try {
          sendMessage(message);
        } catch (TRADEException e) {
          log.error("Failed to send message", e);
        }
      } catch (IOException e) {
        logger.error("Failed to send depth data frame", e);
      } catch (Exception e) {
        logger.error("Error during depth data streaming", e);
      }
    }, 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS);
  }

  /**
   * Creates a color map image from depth data.
   *
   * @param depthImage the depth image
   * @return the color map image
   */
  private BufferedImage createColorMapImage(BufferedImage depthImage) {
    int width = depthImage.getWidth();
    int height = depthImage.getHeight();
    BufferedImage colorMapImage = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int rgba = depthImage.getRGB(x, y);
        int depth = (rgba >> 24) & 0xFF; // Assuming depth data is in the alpha channel
        int color = getColorForDepth(depth);
        colorMapImage.setRGB(x, y, color);
      }
    }

    return colorMapImage;
  }

  /**
   * Gets the color for a given depth value using a Jet colormap.
   *
   * @param depth the depth value
   * @return the color corresponding to the depth value
   */
  private int getColorForDepth(int depth) {
    // Normalize the depth value to a range [0, 1]
    double normalizedDepth = depth / 255.0;

    // Jet colormap mapping
    int r = (int) (255 * Math.max(0, Math.min(1, 1.5 - Math.abs(4 * (normalizedDepth - 0.75)))));
    int g = (int) (255 * Math.max(0, Math.min(1, 1.5 - Math.abs(4 * (normalizedDepth - 0.5)))));
    int b = (int) (255 * Math.max(0, Math.min(1, 1.5 - Math.abs(4 * (normalizedDepth - 0.25)))));

    return (r << 16) | (g << 8) | b;
  }

  /**
   * Extracts an index from the JSON payload.
   *
   * @param payload the JSON payload
   * @return the extracted index
   */
  private int extractIndex(JsonObject payload) {
    return Integer.parseInt(payload.get("id").getAsString()
            .substring("search".length()));
  }

  /**
   * Removes a search from the managed list.
   *
   * @param index the index of the search to remove
   */
  private void removeSearch(int index) {
    try {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("stopAndRemoveType").argTypes(Long.class)).call(void.class, (long) index);
      } catch (TRADEException e) {
        logger.warn("Failed to call stopAndRemoveType.", e);
      }
    } catch (Exception e) {
      logger.error("Error removing SearchManager with ID: {}", index, e);
    }
  }

  /**
   * Starts the specified search.
   *
   * @param request the JSON request containing the search ID
   */
  public void startSearch(JsonObject request) {
    int searchId = extractIndex(request);

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("startType").argTypes(Long.class)).call(void.class, (long) searchId);

      logger.info("Search with ID {} started successfully.", searchId);

      // Update the map to indicate that the type is now running
      typeIdToIsRunning.put((long) searchId, true);

      // THESE ARE HANDLED IN FRONTEND handleStartSearch
      //  (dis/en)able start/stop/suspend buttons
      //disallow switching detector/tracker while running

    } catch (TRADEException e) {
      logger.warn("Failed to call startType for SearchManager ID " + searchId, e);

      // Send error message
      try {
        JsonObject errorMsg = new JsonObject();
        errorMsg.addProperty("error", "Failed to call startType for SearchManager ID : " + searchId);
        errorMsg.addProperty("path", getPath());
        sendMessage(errorMsg);
      } catch (TRADEException ex) {
        log.error("Failed to send message", ex);
      }

    }
  }

  /**
   * Stops the specified search.
   *
   * @param request the JSON request containing the search ID
   */
  private void stopSearch(JsonObject request) {
    int searchId = extractIndex(request);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("stopType").argTypes(Long.class)).call(void.class, (long) searchId);

      logger.info("Search with ID {} stopped successfully.", searchId);

      // Update the map to indicate that the type is now stopped
      typeIdToIsRunning.put((long) searchId, false);

      // THESE ARE HANDLED IN FRONTEND handleStopSearch
      //  (dis/en)able start/stop/suspend buttons
      //allow switching detector/tracker while stopped

    } catch (TRADEException e) {
      logger.warn("Failed to call stopType for SearchManager ID " + searchId, e);

      // Send error message back to the client
      try {
        JsonObject errorMsg = new JsonObject();
        errorMsg.addProperty("error", "Failed to call stopType for SearchManager ID : " + searchId);
        errorMsg.addProperty("path", getPath());
        sendMessage(errorMsg);
      } catch (TRADEException ex) {
        log.error("Failed to send message", ex);
      }
    }

  }

  /**
   * Sends available constraint options to the client.
   */
  private void sendConstraintOptions() {
    List<Term> constraintOptions = new ArrayList<>();

    try {
      @SuppressWarnings("unchecked")
      List<Term> result = (List<Term>) TRADE.getAvailableService(new TRADEServiceConstraints().name("getPropertiesHandled").argTypes().inGroups("physobj")).call(List.class);
      if (result != null) {
        constraintOptions = result;
      }
    } catch (TRADEException e) {
      logger.warn("Failed to call getConstraintOptions from VisionComponent.", e);
    }

    List<Term> sortedConstraintOptions = new ArrayList<>(constraintOptions);
    sortedConstraintOptions.sort(Comparator.comparing(Symbol::getName));
    JsonArray optionsArray = new JsonArray();
    for (Term term : sortedConstraintOptions) {
      JsonObject optionObj = new JsonObject();
      optionObj.addProperty("value", term.getName());  // Use the string representation as a unique identifier
      optionObj.addProperty("label", term.getName());
      optionObj.addProperty("category", "Detector");
      optionsArray.add(optionObj);
    }

    JsonObject message = new JsonObject();
    message.addProperty("action", "configOptions");
    message.add("constraintOptions", optionsArray);
    message.addProperty("path", getPath());
    try {
      sendMessage(message);
    } catch (TRADEException e) {
      log.error("Failed to send message", e);
    }
  }

  /**
   * Applies the specified constraints to the specified search and informs the user of success or failure.
   *
   * @param request the JSON request containing the index and constraints
   */
  private void applyConstraints(JsonObject request) {
    JsonArray constraintsArray = request.getAsJsonArray("constraints");

    //select Detector
    try {
      if (constraintsArray.size() == 1) {
        // If there is only one constraintValue
        String constraintValue = constraintsArray.get(0).getAsString();
        log.debug("constraintValue: " + constraintValue);

        try {
          Long typeId = TRADE.getAvailableService(new TRADEServiceConstraints()
                          .name("getTypeId")
                          .argTypes(Symbol.class)
                          .inGroups("physobj"))
                  .call(Long.class, Factory.createPredicate(constraintValue));
          typeIdToIsRunning.put(typeId, true);  // Mapping typeId to true
        } catch (TRADEException e) {
          logger.warn("Failed to call getTypeId with single predicate input.", e);
        }
      } else {
        // If there are multiple constraintValues
        List<Predicate> description = new ArrayList<>();

        for (int i = 0; i < constraintsArray.size(); i++) {
          String constraintValue = constraintsArray.get(i).getAsString();
          log.debug("constraintValue: " + constraintValue);

          // Use constraintValue in createPredicate
          description.add(Factory.createPredicate(constraintValue));
        }

        try {
          Long typeId = TRADE.getAvailableService(new TRADEServiceConstraints()
                          .name("getTypeId")
                          .argTypes(List.class))
                  .call(Long.class, description);
          typeIdToIsRunning.put(typeId, true);  // Mapping typeId to true
        } catch (TRADEException e) {
          logger.warn("Failed to call getTypeId with input description.", e);
        }
      }
    } catch (Exception e) {
      logger.error("Failed to apply constraints: " + e.getMessage());
      try {
        JsonObject errorMsg = new JsonObject();
        errorMsg.addProperty("error", "Failed to apply constraints");
        errorMsg.addProperty("details", e.getMessage());
        errorMsg.addProperty("path", getPath());
        sendMessage(errorMsg);
      } catch (TRADEException ex) {
        log.error("Failed to send message", ex);
      }
    }
  }

  /**
   * Carries out a request from the client.
   *
   * @param request the request from the client
   * @throws JsonParseException if the action is not found in the request
   */
  private void handleAction(JsonObject request) throws JsonParseException {
    if (!request.has("action")) {
      throw new JsonParseException("Action not found in request");
    }
    String action = request.get("action").getAsString();

    logger.info("Handling action: {}", action);
    switch (action) {
      case "init" -> {
        logger.debug("Initializing...");
        sendConstraintOptions();
        logger.debug("sendConstraintOptions complete.");
        sendUpdate(); // Send initial search data if necessary
        logger.debug("Initialization complete.");
      }

      case "startStream" -> {
        logger.debug("Starting stream...");
        startStreaming();
        logger.debug("Stream started.");
      }

      case "stopStream" -> {
        logger.debug("Stopping stream...");
        stopStreaming();
        logger.debug("Stream stopped.");
      }

      case "takeSnapshot" -> {
        logger.debug("Taking snapshot...");
        takeSnapshot();
        logger.debug("takeSnapshot complete.");
      }

      case "startDepthStream" -> {
        logger.debug("Starting depth stream...");
        startDepthStreaming();
        logger.debug("Depth stream started.");
      }

      case "stopDepthStream" -> {
        logger.debug("Stopping depth stream...");
        stopDepthStreaming();
        logger.debug("Depth stream stopped.");
      }

      case "remove" -> {
        int index = extractIndex(request);
        logger.debug("Removing search at index: {}", index);
        removeSearch(index);
        logger.debug("Search removed.");
      }

      case "startSearch" -> {
        logger.debug("Starting search...");
        startSearch(request);
        logger.debug("Search started.");
      }

      case "stopSearch" -> {
        logger.debug("Stopping search...");
        stopSearch(request);
        logger.debug("Search stopped.");
      }

      case "applyConstraints" -> {
        logger.debug("Applying constraints...");
        applyConstraints(request);
        logger.debug("Constraints applied.");
      }

      default -> {
        logger.warn("Unknown action: {}", action);
        try {
          JsonObject errorMsg = new JsonObject();
          errorMsg.addProperty("error", "unknown command");
          errorMsg.addProperty("path", getPath());
          sendMessage(errorMsg);
        } catch (TRADEException ex) {
          log.error("Failed to send message", ex);
        }
      }
    }
  }

  /**
   * Update the client on current list of searches.
   */
  private void sendUpdate() {
    JsonArray array = new JsonArray();

    List<Long> availableTypeIds = new ArrayList<>();
    try {
      @SuppressWarnings("unchecked")
      List<Long> result = (List<Long>) TRADE.getAvailableService(
              new TRADEServiceConstraints()
                      .name("getAvailableTypeIds")
                      .argTypes()
      ).call(List.class);
      if (result != null) {
        availableTypeIds = result;
      }
    } catch (TRADEException e) {
      logger.warn("Failed to call getAvailableTypeIds.", e);
    }

    for (Long typeId : availableTypeIds) {

      JsonArray constraintsArray = new JsonArray();

      List<Term> descriptors = new ArrayList<>();
      try {
        @SuppressWarnings("unchecked")
        List<Term> result = (List<Term>) TRADE.getAvailableService(
                new TRADEServiceConstraints()
                        .name("getDescriptors")
                        .argTypes(Long.class)
        ).call(List.class, typeId);
        if (result != null) {
          descriptors = result;
        }
      } catch (TRADEException e) {
        logger.warn("Failed to call getDescriptors.", e);
      }

      for (Term descriptor : descriptors) {
        JsonObject constraintObj = new JsonObject();
        constraintObj.addProperty("value", descriptor.toString());
        constraintObj.addProperty("label", descriptor.getName());
        constraintObj.addProperty("category", "Detector");
        constraintsArray.add(constraintObj);
      }

      JsonObject metadata = new JsonObject();
      metadata.addProperty("running", typeIdToIsRunning.getOrDefault(typeId, false));
      metadata.add("constraints", constraintsArray);

      JsonObject search = new JsonObject();
      search.addProperty("name", typeId);
      search.addProperty("id", "search" + typeId);
      search.add("metadata", metadata);

      array.add(search);
    }

    JsonObject message = new JsonObject();
    message.addProperty("action", "updateSearches");
    message.add("searches", array);
    message.addProperty("path", getPath());
    try {
      sendMessage(message);
    } catch (TRADEException e) {
      log.error("Failed to send message", e);
    }
  }

  //======================================================================
  // Implementing methods | TextWebSocketHandler
  //======================================================================

  /**
   * Handles incoming WebSocket messages by identifying the action requested and delegating to the appropriate method.
   *
   * @param message The text message received from the client.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    try {
      logger.debug("Received WebSocket message: {}", message);
      handleAction(message);
      logger.debug("handleAction complete.");
      sendUpdate();
      logger.debug("sendUpdate complete.");
    } catch (JsonParseException e) {
      logger.error("Json parsing error.", e);
      // Send error message back to client
      try {
        JsonObject errorMsg = new JsonObject();
        errorMsg.addProperty("error", "Invalid JSON format");
        errorMsg.addProperty("path", getPath());
        sendMessage(errorMsg);
      } catch (TRADEException ex) {
        log.error("Failed to send message", ex);
      }
    } catch (Exception e) {
      logger.error("Failed to handle message: {}", e.getMessage());
    }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  public String getPathRoot() {
    return "vision";
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
