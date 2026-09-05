/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.ava.message.response.RMSMapMetadataResponse;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.GuiAdapter;
import edu.tufts.hrilab.util.Convert;
import org.apache.commons.lang3.tuple.Pair;

import javax.imageio.ImageIO;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

// the ava base serves the map as one png per tile; stitched here and served by MapImageController
public class AvaMapAdapter extends GuiAdapter {

  //well clear of the tab's 20 s renewal, which a browser throttles hard once it is in the background
  private static final long SUBSCRIBE_TTL_MS = 180000;

  private final ScheduledExecutorService poseExecutor = Executors.newScheduledThreadPool(1);
  private final ExecutorService commandExecutor = Executors.newSingleThreadExecutor();
  private final Object mapLock = new Object();
  // one drive at a time: a second one would queue up behind the first and set off again the moment
  // the user stopped the one they could see
  private final AtomicBoolean driving = new AtomicBoolean(false);
  private ScheduledFuture<?> poseTask;
  // a browser that goes away without unsubscribing (closed laptop, dropped network) would leave us
  // polling the robot for good, so the tab renews while it is watching and the push expires without it
  private volatile long subscribedAt;

  private List<BufferedImage> mapImages = new ArrayList<>();
  private List<RMSMapMetadataResponse> mapMetadatas = new ArrayList<>();
  private BufferedImage finalMap;
  private int finalMapWidth;
  private int finalMapHeight;
  private int finalMapOriginX;
  private int finalMapOriginY;
  private double mapScaleX;
  private double mapScaleY;
  private long version;

  public AvaMapAdapter(Collection<String> groups) {
    super(groups);
  }

  @Override
  protected void handleMessage(JsonObject message) {
    String method = message.get("method").getAsString();
    switch (method) {
      case "startup" -> onStartup();
      case "refresh" -> onRefresh();
      case "subscribe" -> subscribe();
      case "unsubscribe" -> unsubscribe();
      case "goToLocation" -> goToLocation(Factory.createSymbol(message.get("id").getAsString()));
      case "stop" -> stop();
      default -> sendError("unsupported method " + method);
    }
  }

  private void onStartup() {
    if (finalMap == null) {
      fetchMap();
    }
    sendMap();
    sendLocations();
    sendPose();
  }

  private void onRefresh() {
    fetchMap();
    sendMap();
    sendLocations();
    sendPose();
  }

  private void fetchMap() {
    List<RMSMapMetadataResponse> metadatas = fetchMapMetadata();
    List<BufferedImage> images = fetchMapImages();
    // a failed pull leaves whatever we already had on screen rather than blanking the tab
    if (metadatas.isEmpty() || images.isEmpty()) {
      log.warn("[fetchMap] the robot returned no map, keeping the last one");
      return;
    }
    synchronized (mapLock) {
      mapMetadatas = metadatas;
      mapImages = images;
      stitchMap();
      version = System.currentTimeMillis();
    }
  }

  private List<RMSMapMetadataResponse> fetchMapMetadata() {
    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getMapMetadatas")
              .argTypes()
              .inGroups(this.groups.toArray(new String[0]))).call(List.class);
    } catch (TRADEException e) {
      log.error("[fetchMapMetadata] could not get map metadata", e);
      return new ArrayList<>();
    }
  }

  private List<BufferedImage> fetchMapImages() {
    List<BufferedImage> images = new ArrayList<>();
    List<byte[]> imageBytes;
    try {
      imageBytes = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getMapImages")
              .argTypes()
              .inGroups(this.groups.toArray(new String[0]))).call(List.class);
    } catch (TRADEException e) {
      log.error("[fetchMapImages] could not get map images", e);
      return images;
    }
    // a tile that will not decode still gets an entry so the indices line up with the metadata,
    // and is skipped when the tiles are drawn
    for (byte[] bytes : imageBytes) {
      try {
        images.add(ImageIO.read(new ByteArrayInputStream(bytes)));
      } catch (IOException e) {
        log.error("[fetchMapImages] could not decode a map tile", e);
        images.add(null);
      }
    }
    return images;
  }

  private void stitchMap() {
    double scaleX = mapMetadatas.get(0).response.mapScaleX;
    double scaleY = mapMetadatas.get(0).response.mapScaleY;
    if (scaleX <= 0 || scaleY <= 0) {
      log.error("[stitchMap] map metadata has no usable scale, x {} y {}", scaleX, scaleY);
      return;
    }
    mapScaleX = scaleX;
    mapScaleY = scaleY;
    if (mapMetadatas.size() == 1) {
      if (mapImages.get(0) == null) {
        log.error("[stitchMap] the only map tile did not decode");
        return;
      }
      finalMapWidth = mapMetadatas.get(0).response.width;
      finalMapHeight = mapMetadatas.get(0).response.height;
      finalMapOriginX = (int) (mapMetadatas.get(0).response.originX / mapScaleX);
      finalMapOriginY = (int) (mapMetadatas.get(0).response.originY / mapScaleY);
      finalMap = mapImages.get(0);
      return;
    }

    double minX = Double.MAX_VALUE;
    double maxX = -Double.MAX_VALUE;
    double minY = Double.MAX_VALUE;
    double maxY = -Double.MAX_VALUE;
    List<Double> tileMinXs = new ArrayList<>();
    List<Double> tileMaxYs = new ArrayList<>();
    for (RMSMapMetadataResponse metadata : mapMetadatas) {
      double originX = metadata.response.originX;
      double originY = metadata.response.originY;
      double tileScaleX = metadata.response.mapScaleX;
      double tileScaleY = metadata.response.mapScaleY;
      int width = metadata.response.width;
      int height = metadata.response.height;
      if (tileScaleX <= 0 || tileScaleY <= 0) {
        log.error("[stitchMap] a map tile has no usable scale, x {} y {}", tileScaleX, tileScaleY);
        return;
      }
      double tileMinX = (originX / tileScaleX) - width * 0.5;
      double tileMaxX = (originX / tileScaleX) + width * 0.5;
      double tileMinY = (originY / tileScaleY) - height * 0.5;
      double tileMaxY = (originY / tileScaleY) + height * 0.5;
      minX = Math.min(minX, tileMinX);
      maxX = Math.max(maxX, tileMaxX);
      minY = Math.min(minY, tileMinY);
      maxY = Math.max(maxY, tileMaxY);
      tileMinXs.add(tileMinX);
      tileMaxYs.add(tileMaxY);
    }
    int mergedWidth = (int) (maxX - minX);
    int mergedHeight = (int) (maxY - minY);
    if (mergedWidth <= 0 || mergedHeight <= 0) {
      log.error("[stitchMap] stitched map has no area, {}x{}", mergedWidth, mergedHeight);
      return;
    }
    finalMapWidth = mergedWidth;
    finalMapHeight = mergedHeight;
    finalMapOriginX = (int) ((minX + maxX) / 2);
    finalMapOriginY = (int) ((minY + maxY) / 2);

    BufferedImage merged = new BufferedImage(finalMapWidth, finalMapHeight, BufferedImage.TYPE_BYTE_BINARY);
    Graphics mergedGraphics = merged.getGraphics();
    for (int i = 0; i < mapMetadatas.size(); i++) {
      if (i >= mapImages.size() || mapImages.get(i) == null) {
        continue;
      }
      int tileX = finalMapWidth / 2 + (tileMinXs.get(i).intValue() - finalMapOriginX);
      int tileY = finalMapHeight / 2 - (tileMaxYs.get(i).intValue() - finalMapOriginY);
      mergedGraphics.drawImage(mapImages.get(i), tileX, tileY, null);
    }
    mergedGraphics.dispose();
    finalMap = merged;
  }

  private void sendMap() {
    JsonObject map = new JsonObject();
    synchronized (mapLock) {
      if (finalMap == null) {
        sendError("no map available");
        return;
      }
      map.addProperty("imagePath", "api/map/image");
      map.addProperty("version", version);
      map.addProperty("width", finalMapWidth);
      map.addProperty("height", finalMapHeight);
      map.addProperty("metersPerPixelX", mapScaleX);
      map.addProperty("metersPerPixelY", mapScaleY);
    }

    JsonObject response = new JsonObject();
    response.add("map", map);
    send(response);
  }

  private void sendLocations() {
    JsonArray locations = new JsonArray();
    try {
      Map<Symbol, PoseReference> refs = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getMapReferences")
              .argTypes()
              .inGroups(this.groups.toArray(new String[0]))).call(Map.class);
      TRADEServiceInfo nameService = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getMapReferenceName")
              .argTypes(Symbol.class)
              .inGroups(this.groups.toArray(new String[0])));
      for (Map.Entry<Symbol, PoseReference> entry : refs.entrySet()) {
        PoseReference ref = entry.getValue();
        if (!ref.hasPose()) {
          continue;
        }
        JsonObject location = new JsonObject();
        location.addProperty("id", entry.getKey().toString());
        location.addProperty("name", nameService.call(String.class, entry.getKey()));
        addPosition(location, ref.getPosition());
        locations.add(location);
      }
    } catch (TRADEException e) {
      log.error("[sendLocations] could not get map references", e);
      sendError("could not read the robot's locations");
      return;
    }

    JsonObject response = new JsonObject();
    response.add("locations", locations);
    send(response);
  }

  private void sendPose() {
    Pair<Point3d, Quat4d> currPose;
    try {
      currPose = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getPose")
              .argTypes()
              .inGroups(this.groups.toArray(new String[0]))).call(Pair.class);
    } catch (TRADEException e) {
      log.error("[sendPose] could not get robot pose", e);
      return;
    }
    if (currPose == null) {
      return;
    }

    double yaw = Convert.convertToEuler(currPose.getRight())[Convert.YAW];
    JsonObject pose = new JsonObject();
    addPosition(pose, currPose.getLeft());
    pose.addProperty("yaw", yaw);

    JsonObject response = new JsonObject();
    response.add("pose", pose);
    send(response);
  }

  private void addPosition(JsonObject json, Point3d position) {
    json.addProperty("x", position.getX());
    json.addProperty("y", position.getY());
    synchronized (mapLock) {
      if (finalMap == null) {
        return;
      }
      double px = position.getX() / mapScaleX - finalMapOriginX + finalMapWidth * 0.5;
      double py = -position.getY() / mapScaleY + finalMapOriginY + finalMapHeight * 0.5;
      json.addProperty("px", px);
      json.addProperty("py", py);
    }
  }

  private synchronized void subscribe() {
    subscribedAt = System.currentTimeMillis();
    if (poseTask != null) {
      return;
    }
    //fixed delay, not fixed rate: getPose is a round trip to the base and a slow one would otherwise
    //come back to a queue of overdue runs
    poseTask = poseExecutor.scheduleWithFixedDelay(() -> {
      try {
        //inside the try: an uncaught throw here cancels the scheduled task permanently
        if (!expired()) {
          sendPose();
        }
      } catch (Exception e) {
        log.error("[subscribe] exception while sending the robot pose", e);
      }
    }, 0, 1, TimeUnit.SECONDS);
  }

  private synchronized boolean expired() {
    if (System.currentTimeMillis() - subscribedAt <= SUBSCRIBE_TTL_MS) {
      return false;
    }
    unsubscribe();
    return true;
  }

  private synchronized void unsubscribe() {
    if (poseTask != null) {
      poseTask.cancel(false);
      poseTask = null;
    }
  }

  private void goToLocation(Symbol location) {
    JsonObject response = new JsonObject();
    response.addProperty("command", "goToLocation");
    if (!driving.compareAndSet(false, true)) {
      response.addProperty("accepted", false);
      response.addProperty("message", "already driving somewhere, stop it first");
      send(response);
      return;
    }
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("goToLocation")
              .argTypes(Symbol.class, Boolean.class)
              .inGroups(this.groups.toArray(new String[0])));
      // the ava side drives to the goal inside the call whichever way the wait flag is set, so the
      // websocket thread hands it off and acks the request rather than the arrival
      commandExecutor.submit(() -> {
        try {
          Justification result = tsi.call(Justification.class, location, false);
          boolean reached = result != null && result.getValue();
          log.info("[goToLocation] {} finished, reached {}", location, reached);
        } catch (TRADEException e) {
          log.error("[goToLocation] navigation to {} failed", location, e);
        } finally {
          driving.set(false);
        }
      });
      response.addProperty("accepted", true);
      response.addProperty("message", "navigating to " + location);
    } catch (TRADEException e) {
      log.error("[goToLocation] no navigation service for {}", location, e);
      driving.set(false);
      response.addProperty("accepted", false);
      response.addProperty("message", "no navigation service available");
    }
    send(response);
  }

  private void stop() {
    JsonObject response = new JsonObject();
    response.addProperty("command", "stop");
    try {
      Justification result = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("stop")
              .argTypes()
              .inGroups(this.groups.toArray(new String[0]))).call(Justification.class);
      boolean accepted = result != null && result.getValue();
      log.info("[stop] stop requested, accepted {}", accepted);
      response.addProperty("accepted", accepted);
      response.addProperty("message", "stop requested");
    } catch (TRADEException e) {
      log.error("[stop] could not stop the robot", e);
      response.addProperty("accepted", false);
      response.addProperty("message", "stop call failed");
    }
    send(response);
  }

  private void sendError(String error) {
    JsonObject response = new JsonObject();
    response.addProperty("error", error);
    send(response);
  }

  private void send(JsonObject response) {
    try {
      sendMessage(response);
    } catch (TRADEException e) {
      log.error("[send] failed to send message", e);
    }
  }

  @TRADEService
  public BufferedImage getMapImage() {
    synchronized (mapLock) {
      return finalMap;
    }
  }

  @Override
  public String getPathRoot() {
    return "map";
  }

  @Override
  protected boolean providesTradeServices() {
    return true;
  }
}
