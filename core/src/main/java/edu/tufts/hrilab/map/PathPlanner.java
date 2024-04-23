/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import edu.tufts.hrilab.map.PathAction;
import edu.tufts.hrilab.map.util.Utils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

public class PathPlanner {
  private static Logger log = LoggerFactory.getLogger(PathPlanner.class);

  /**
   * gets all paths from source to destination node in a graph
   *
   * @param s source node
   * @param d destination node
   * @return all path options
   */
  public static List<List<MapObject>> getAllPaths(FloorMap floorMap, MapObject s, MapObject d) {
    Set<MapObject> rooms = floorMap.getAllRooms();
    boolean[] visited = new boolean[rooms.size()];
    Map<MapObject, Integer> moToIdx = new HashMap<>();
    List<MapObject> pathList = new ArrayList<>();
    AtomicInteger vid = new AtomicInteger(0);
    rooms.forEach(room -> moToIdx.put(room, vid.getAndIncrement()));

    // add source to path[]
    pathList.add(s);

    // Call recursive utility
    List<List<MapObject>> paths = new ArrayList<>();
    getAllPathsUtil(floorMap, s, d, visited, pathList, moToIdx, paths);
    return paths;
  }

  /**
   * recursive utility function for getAllPaths()
   *
   * @param u             current node
   * @param d             target node
   * @param isVisited     list of visited nodes
   * @param localPathList current path
   * @param moToIdx       mapping from a MapObject to an index in isVisited
   * @param paths         output paths that get populated by this method
   * @return
   */
  private static void getAllPathsUtil(FloorMap floorMap, MapObject u, MapObject d, boolean[] isVisited, List<MapObject> localPathList, Map<MapObject, Integer> moToIdx, List<List<MapObject>> paths) {
    if (u.equals(d)) {
      List<MapObject> copy = new ArrayList<>();
      for (MapObject i : localPathList) {
        copy.add(i);
      }
      paths.add(copy);
      // if match found then no need to traverse more till depth
      return;
    }

    // Mark the current node
    isVisited[moToIdx.get(u)] = true;
    Set<MapObject> adj_u = floorMap.getAdjacentNodes(u);
    for (MapObject mo : adj_u) {
      if (!mo.isA("room")) {
        continue;
      }
      if (!isVisited[moToIdx.get(mo)] && !floorMap.isDoorClosed(u, mo)) {
        localPathList.add(mo);
        getAllPathsUtil(floorMap, mo, d, isVisited, localPathList, moToIdx, paths);
        localPathList.remove(mo);
      }
    }

    // Mark the current node
    isVisited[moToIdx.get(u)] = false;
  }

  /**
   * computes the distance of a path given a list of rooms that make up the path
   *
   * @param start   starting location
   * @param path    list of rooms that constitute the path
   * @return distance path travels
   */
  public static double getPathDistanceFromRooms(MapObject start, List<MapObject> path) {
    MapObject prev = start;
    double dist = 0;
    for (MapObject curr : path) {
      dist += Utils.dist(curr.getPosition(), prev.getPosition());
      prev = curr;
    }
    return dist;
  }

  /**
   * get shortest path (shortest qualified as lowest number of rooms in path) from room A to room B
   *
   * @param src  Color of room you are in
   * @param dest color of destination room
   * @return List of rooms to achieve shortest path
   */
  public static List<MapObject> getShortestDistance(FloorMap floorMap, MapObject src, MapObject dest) {
    String pathName = src.getID() + "-" + dest.getID();
    if (floorMap.getKnownPath(pathName) == null) {
      // no known path between locations
      List<List<MapObject>> paths = getAllPaths(floorMap, src, dest);
      if (paths.size() == 0) {
        return new ArrayList<>();
      }
      int minLeng = Integer.MAX_VALUE;
      for (int i = 0; i < paths.size(); i++) {
        int curr = paths.get(i).size();
        if (curr < minLeng) {
          minLeng = curr;
        }
      }
      double minDist = Double.MAX_VALUE;
      int minIdx = 0;
      for (int i = 0; i < paths.size(); i++) {
        // if (paths.get(i).size() <= minLeng + 1) {
        double curr = getPathDistanceFromRooms(src, paths.get(i));
        if (curr < minDist) {
          minDist = curr;
          minIdx = i;
        }
        // }
      }

      List<MapObject> out = new ArrayList<>();
      for (MapObject i : paths.get(minIdx)) {
        out.add(i);
      }

      // cache path
      floorMap.addKnownPath(pathName, out);
      return out;
    } else {
      return floorMap.getKnownPath(pathName);
    }
  }

  /**
   * gets path from start position to end point
   *
   * @param start    start location MapObject
   * @param end     end location MapObject
   * @return List of poses to go to to acheive path
   */
  public static List<MapObject> getRoomPath(FloorMap floorMap, MapObject start, MapObject end) {
    if (start.equals(end)) {
      List<MapObject> path = new ArrayList<>();
      path.add(end);
      return path;
    }

    if (end.isA("room")) {
      List<MapObject> path = getShortestDistance(floorMap, start, end);
      int pathLen = path.size();
      path.set(pathLen - 1, end);
      return path;
    } else {
      log.error("Target location must be in a room");
    }
    return new ArrayList<>();
  }

  /**
   * turns a list of rooms to go to into a list of poses to get there with
   *
   * @param roomPath list of MapObjects representing rooms
   * @param padding  how far away from doors to stop
   * @return list of poses representing same path as roomPath
   */
  public static List<PathAction> convertRoomPathToPathActions(FloorMap floorMap, List<MapObject> roomPath, boolean canOpenDoors, double padding) {
    List<PathAction> actions = new ArrayList<>();
    for (int i = 0; i < roomPath.size(); i++) {
      MapObject currMO = roomPath.get(i);
      if (i < roomPath.size() - 1) {
        MapObject nextMO = roomPath.get(i+1);
        //for intermediary rooms, pathfind to door to next room
        MapObject portal = floorMap.getPortalBetween(currMO, nextMO);

        // face towards portal in current room
        Matrix4d enterPortalPose = floorMap.getPortalFromRoomPose(portal, currMO, true, padding);
        actions.add(new PathAction(PathAction.Type.ACTION_GOTO, currMO.getReferenceId(), enterPortalPose));

        // add door checking/opening
        if (portal.isA("door")) {
          if (canOpenDoors) {
            actions.add(new PathAction(PathAction.Type.ACTION_OPEN_DOOR, portal.getReferenceId()));
          } else {
            actions.add(new PathAction(PathAction.Type.ACTION_CHECK_DOOR, portal.getReferenceId()));
          }
        }

        // face away from portal in next room
        Matrix4d exitPortalPose = floorMap.getPortalFromRoomPose(portal, nextMO, false, padding);
        actions.add(new PathAction(PathAction.Type.ACTION_GOTO, currMO.getReferenceId(), exitPortalPose));
      } else {
        //for final room, pathfind to the middle of the room
        actions.add(new PathAction(PathAction.Type.ACTION_GOTO, currMO.getReferenceId(), currMO.getPose()));
      }
    }

    return actions;
  }

}
