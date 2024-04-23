/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import edu.tufts.hrilab.fol.Symbol;
import com.google.common.graph.EndpointPair;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import com.google.common.reflect.TypeToken;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.tufts.hrilab.map.util.Pose;
import edu.tufts.hrilab.map.util.Utils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class FloorMap {
  private static Logger log = LoggerFactory.getLogger(FloorMap.class);

  private int floorNumber;
  private PixelMap pixelMap;

  private MutableValueGraph<MapObject, MapObject> roomGraph = ValueGraphBuilder.undirected().build();

  private Map<String, List<MapObject>> knownPaths = new HashMap<>();

  private Set<MapObject> roomObjects = new HashSet<>();
  private Set<MapObject> portalObjects = new HashSet<>();
  private Set<MapObject> closedDoors = new HashSet<>();

  // map an elevator button to the elevators it can call
  private Map<MapObject, List<MapObject>> buttonsToElevatorDoors = new HashMap<>();
  private Map<MapObject, MapObject> elevatorRoomsToPanels = new HashMap<>();

  /**
   * YAML filename used to load map data.
   */
  private String yamlFile;

  // map from refId -> mapObejctId
  private Map<Symbol, Integer> refIdMap = new HashMap<>();

  public FloorMap(Path jsonPath) {
    parseJson(jsonPath);
  }

  /**
   * Get YAML filename used to load map data. Exposed here so that it can be sent to ROS
   * to load a new map when switching floors.
   * @return
   */
  public String getMapYamlFile() {
    return yamlFile;
  }

  /**
   * Assign the MapObject a unique referenceId that is unique across all maps.
   *
   * @param mo
   * @param refId
   */
  public void setReferenceId(MapObject mo, Symbol refId) {
    mo.setReferenceId(refId);
    refIdMap.put(refId, mo.getID());
  }

  private void parseJson(Path jsonPath) {
    JsonParser parser = new JsonParser();
    JsonObject root = null;
    try {
      root = parser.parse(Files.newBufferedReader(jsonPath)).getAsJsonObject();
    } catch (IOException e) {
      log.error("Error parsing map file: " + jsonPath, e);
    }

    String dir = jsonPath.getParent().toString() + "/";
    yamlFile = dir + root.get("map_yaml").getAsString();
    floorNumber = root.get("floor").getAsInt();
    // instantiate pixel map
    pixelMap = new PixelMap(floorNumber);
    pixelMap.parseMapYAML(yamlFile);

    // parse semantic labels
    Map<String, Integer> labels = new Gson().fromJson(root.get("semantic_labels"),
            new TypeToken<Map<String, Integer>>() {}.getType());
    labels.forEach((k, v) -> pixelMap.addSemanticLabel(k, v));

    // parse portals (after semantic labels are added)
    Map<String, List<Integer>> portals = new Gson().fromJson(root.get("portals"),
            new TypeToken<Map<String, List<Integer>>>() {}.getType());
    portals.forEach((k, v) -> addPortal(k, v));
  }

  /**
   * Get floor number of this map.
   *
   * @return
   */
  public int getFloorNumber() {
    return floorNumber;
  }

  /**
   * Get underlying pixel map.
   * @return
   */
  public PixelMap getPixelMap() {
    return pixelMap;
  }

  /**
   * gets a set of all known MapObjects
   *
   * @return set with every MapObject
   */
  public Set<MapObject> getAllObjects() {
    Set<MapObject> objects = new HashSet<>();
    objects.addAll(roomObjects);
    objects.addAll(portalObjects);
    objects.addAll(buttonsToElevatorDoors.keySet()); // add elevator buttons
    objects.addAll(elevatorRoomsToPanels.values()); // add elevator panels
    return objects;
  }

  /**
   * gets a set of all known room MapObjects
   *
   * @return set with all room MapObject
   */
  public Set<MapObject> getAllRooms() {
    Set<MapObject> objects = new HashSet<>();
    objects.addAll(roomObjects);
    return objects;
  }

  public List<MapObject> getKnownPath(String pathName) {
    return knownPaths.get(pathName);
  }

  public void addKnownPath(String pathName, List<MapObject> path) {
    knownPaths.put(pathName, path);
  }

  /**
   * converts a symbol refID to a mapobject
   *
   * @param refID symbol to get MapObject from
   * @return MapObject represented by refID
   */
  public MapObject getMapObjectFromReferenceId(Symbol refID) {
    Integer mapId = refIdMap.get(refID);
    if (mapId == null) {
      return null;
    }
    return pixelMap.getMapObjectFromId(mapId);
  }

/************************* Map-Querying Functions *************************/

  /**
   * gets the position and orientation facing towards/away portal p from room r padding meters away
   *
   * @param portal       MapObject portal to face towards/away
   * @param room         MapObject room to be in
   * @param facingPortal if true, face portal, else face away from portal
   * @param padding      how far away from portal to be in meters
   * @return pose representing looking at portal from room
   */
  public Matrix4d getPortalFromRoomPose(MapObject portal, MapObject room, boolean facingPortal, double padding) {

    Point2d portalPixel = pixelMap.toPixel(portal.getPosition());
    padding /= pixelMap.getResolution();

    // create position options on either side of portal
    double ang = pixelMap.getPortalNormalAngle(portal);
    Point2d opt1 = new Point2d(portalPixel.x + (padding * Math.cos(ang + Math.PI)), portalPixel.y + (padding * Math.sin(ang)));
    Point2d opt2 = new Point2d(portalPixel.x + (-1 * padding * Math.cos(ang + Math.PI)), portalPixel.y + (-1 * padding * Math.sin(ang)));

    // get MapObjects of two options, while avoiding creating new MapObjects for out of
    // bounds pixels (which can incur a huge connected components cost for large maps)
    MapObject opt1Room = null;
    if (pixelMap.isExistingMapObjectPixel((int) opt1.x, (int) opt1.y)) {
      opt1Room = pixelMap.getMapObjectFromPixel((int) opt1.x, (int) opt1.y);
    }
    MapObject opt2Room = null;
    if (pixelMap.isExistingMapObjectPixel((int) opt2.x, (int) opt2.y)) {
      opt2Room = pixelMap.getMapObjectFromPixel((int) opt2.x, (int) opt2.y);
    }

    // pick the position option inside the specified room
    Point3d pos;
    if (opt1Room != null && opt1Room.equals(room)) {
      pos = pixelMap.toMeter(opt1);
    } else if (opt2Room != null && opt2Room.equals(room)) {
      pos = pixelMap.toMeter(opt2);
      ang = ang + Math.PI;
    } else {
      log.error("Target position not in target room.");
      return null;
    }

    // if pose should face away from portal, do a 180
    if (!facingPortal) {
      ang = ang + Math.PI;
    }

    // bring between [0,2pi]
    ang = ang % (2 * Math.PI);

    return Utils.convertToMatrix4d(new Pose(pos, Utils.eulerToQuat4d(ang, 0, 0)));
  }

  /**
   * gets rooms accessible from a given room
   *
   * @param room MapObject of room to get neighbors of
   * @return List of room MapObjects adjacent to the given room
   */
  public List<MapObject> getConnectingRooms(MapObject room) {
    try {
      Iterator<MapObject> neighbors = roomGraph.adjacentNodes(room).iterator();
      List<MapObject> out = new ArrayList<>();
      while (neighbors.hasNext()) {
        out.add(neighbors.next());
      }
      return out;
    } catch (IllegalArgumentException e) {
      log.error("Error trying getConnectingRooms:  " + floorNumber + " room: " + room);
      return new ArrayList<>();
    }
  }

  /**
   * gets the elevator doors controlled by an elevator button
   *
   * @param button MapObject of the button to get doors of
   * @return List of door MapObjects associated with the elevator button
   */
  public List<MapObject> getElevatorDoors(MapObject button) {
    List<MapObject> doors = new ArrayList<>();
    if (!buttonsToElevatorDoors.containsKey(button)) {
      log.error(button + " is not a known elevator button!");
      return doors;
    }

    return buttonsToElevatorDoors.get(button);
  }

  /**
   * gets the MapObject of the nearest elevator button to the location
   *
   * @param p Point in meter-space to query
   * @return MapObject of nearest elevator button
   */
  public MapObject getNearestElevatorButton(Point3d p) {
    double minD = Double.POSITIVE_INFINITY;
    MapObject minB = null;
    for (MapObject button : buttonsToElevatorDoors.keySet()) {
      Point3d currCenter = button.getPosition();
      double d = Utils.dist(currCenter, p);
      if (d < minD) {
        minD = d;
        minB = button;
      }
    }
    return minB;
  }

  /**
   * Marks a portal between two rooms as either closed or open.
   *
   * @param portal referenceID of portal
   * @param closed boolean specifying if door is open or closed; true is closed, false is open
   */
  public void markDoor(Symbol portal, boolean closed) {
    MapObject door = getMapObjectFromReferenceId(portal);
    if (door == null) {
      log.error("No door found: " + portal);
      return;
    }
    if (closed) {
      closedDoors.add(door);
    } else {
      closedDoors.remove(door);
    }
  }

  public boolean isDoorClosed(MapObject roomA, MapObject roomB) {
    MapObject portal = getPortalBetween(roomA, roomB);
    return closedDoors.contains(portal);
  }

  public Set<MapObject> getAdjacentNodes(MapObject node) {
    return roomGraph.adjacentNodes(node);
  }

  /**
   * getNearestPortal(): gets the MapObject of the portal nearest to the point
   *
   * @param p Point in meter-space to query
   * @return Object color of nearest portal (can be door or elevator)
   */
  public MapObject getNearestPortal(Point3d p) {
    if (!isPortal(p)) {
      MapObject currRoom = getMapObjectAt(p);
      List<MapObject> neighbors = getConnectingRooms(currRoom);
      MapObject nearestDoor;
      double minDist = Double.POSITIVE_INFINITY;
      int minIdx = -1;
      for (int i = 0; i < neighbors.size(); i++) {
        double d = Utils.dist(p, getPortalBetween(currRoom, neighbors.get(i)).getPosition());
        if (d < minDist || minIdx < 0) {
          minDist = d;
          minIdx = i;
        }
      }
      nearestDoor = getPortalBetween(currRoom, neighbors.get(minIdx));
      return nearestDoor;
    } else {
      return getMapObjectAt(p);
    }
  }

  /**
   * queries if a point is a portal
   *
   * @param p Point in meter-space being query
   * @return true if there is a portal at p, false otherwise
   */
  public boolean isPortal(Point3d p) {
    return isPortal(getMapObjectAt(p));
  }

  /**
   * queries if a MapObject is a portal
   *
   * @param mo MapObject to check
   * @return true if color is of a portal type, false otherwise
   */
  public boolean isPortal(MapObject mo) {
    String type = mo.getProperty();
    return pixelMap.isPortal(type);
  }

  /**
   * gets the color of the portal between two rooms
   *
   * @param roomA MapObject of the source room
   * @param roomB MapObject of the target room
   * @return Color value of portal, returns -1 if no such portal exists
   */
  public MapObject getPortalBetween(MapObject roomA, MapObject roomB) {
    Optional<MapObject> edgeVal = roomGraph.edgeValue(roomA, roomB);
    if (edgeVal.isPresent()) {
      return edgeVal.get();
    } else {
      log.warn("No portal between " + roomA + " and " + roomB + "!");
      return null;
    }
  }

  /**
   * gets the colors of the rooms the portal connects
   *
   * @param portal MapObject of the portal to query
   * @return Pair of room MapObjects
   */
  public EndpointPair<MapObject> getRoomsFromPortal(MapObject portal) {
    Iterator<EndpointPair<MapObject>> portals = roomGraph.edges().iterator();
    while (portals.hasNext()) {
      EndpointPair p = portals.next();
      if (getPortalBetween((MapObject) p.nodeU(), (MapObject) p.nodeV()) == portal) {
        return p;
      }
    }
    log.warn("returning null from portalRooms");
    return null;
  }

  /**
   * Gets the map object at a location in meters. Assumes query point is on current floor.
   *
   * @param p - Point in meters to get object of
   * @return MapObject at p
   */
  public MapObject getMapObjectAt(Point3d p) {
    Point2d pixel = pixelMap.toPixel(p);
    return pixelMap.getMapObjectFromPixel((int) pixel.x, (int) pixel.y);
  }

  /**
   * gets the elevator button associated with the elevator-door
   *
   * TODO: can this be generalized?
   *
   * @param door the MapObject of the elevator door
   * @return the MapObject of the elevator connected to provided door
   */
  public MapObject getButtonFromDoor(MapObject door) {
    if (!door.isA("elevator-door")) {
      log.error("Can't get elevator button from non-elevator-door " + door);
      return null;
    }
    for (MapObject button : buttonsToElevatorDoors.keySet()) {
      if (buttonsToElevatorDoors.get(button).contains(door)) {
        return button;
      }
    }
    log.error("Could not find button for elevator door.");
    return null;
  }

  /**
   * gets the elevator panel in an elevator
   *
   * TODO: can this be generalized?
   *
   * @param elevator the MapObject for the elevator to look in
   * @return a MapObject of the elevator panel
   */
  public MapObject getPanelInElevator(MapObject elevator) {
    return elevatorRoomsToPanels.get(elevator);
  }

  /**
   * parses the map .portal file to load relationships between rooms and portals
   */
  private void addPortal(String name, List<Integer> portalVals) {
    MapObject room1;
    MapObject room2;
    MapObject portal;
    int len = portalVals.size();
    portal = pixelMap.getMapObjectFromPixel(portalVals.get(len - 2), portalVals.get(len - 1));
    if (portal.isA("elevator-button")) {
      List<MapObject> doorList = new ArrayList<>();
      for (int i = 0; i < len - 2; i += 2) {
        MapObject curr = pixelMap.getMapObjectFromPixel(portalVals.get(i), portalVals.get(i + 1));
        doorList.add(curr);
      }
      buttonsToElevatorDoors.put(portal, doorList);
    } else if (portal.isA("elevator-panel")) {
      MapObject elevator = pixelMap.getMapObjectFromPixel(portalVals.get(0), portalVals.get(1));
      elevatorRoomsToPanels.put(elevator, portal);
//        roomObjects.add(portal); // TODO: do we want to add elevators as rooms?
    } else {
      if (len != 6) {
        log.error("Portal does not have 6 pixel values, ignoring portal: " + name);
        return;
      }
      room1 = pixelMap.getMapObjectFromPixel(portalVals.get(0), portalVals.get(1));
      room2 = pixelMap.getMapObjectFromPixel(portalVals.get(2), portalVals.get(3));
      portal = pixelMap.getMapObjectFromPixel(portalVals.get(4), portalVals.get(5));

      try {
        roomGraph.putEdgeValue(room1, room2, portal);
        roomObjects.add(room1);
        roomObjects.add(room2);
        portalObjects.add(portal);
      } catch (IllegalArgumentException e) {
        log.error("Error trying to add portal. Ignoring. Floor:  " + floorNumber + " portal: " + name);
      }
    }
  }

}
