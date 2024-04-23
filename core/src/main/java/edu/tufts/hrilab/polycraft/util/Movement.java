/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.util;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class Movement {
  static private Logger log = LoggerFactory.getLogger(Movement.class);

  /**
   * Get the directions (N,S,E,W) that are empty next to the target location (x,y) with the specified radius.
   * Radius = 1 means the one block in that direction is empty, radius = 2 means two blocks in that direction are empty.
   *
   * @param x
   * @param y
   * @param radius
   * @return directions that are empty (N,S,E,W)
   */
  static public List<Direction> getEmptyDirections(Symbol x, Symbol y, Symbol radius) {
    // HACK: distance is currently either "one" or "two"
    int rad = 1;//Integer.parseInt(distance.toString());
    if (radius.getName().equals("one")) {
      rad = 1;
    } else if (radius.getName().equals("two")) {
      rad = 2;
    } else {
      log.error("Invalid distance value: " + radius);
    }
    int goalX = Integer.parseInt(x.getName());
    int goalY = Integer.parseInt(y.getName());
    List<Direction> valid = new ArrayList<>(Arrays.asList(Direction.values())); //Start with all valid directions

    for (int i = 1; i <= rad; ++i) {
      try {
        if (valid.contains(Direction.NORTH) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX, goalY - i)) {
          valid.remove(Direction.NORTH);
        }
        if (valid.contains(Direction.SOUTH) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX, goalY + i)) {
          valid.remove(Direction.SOUTH);
        }
        if (valid.contains(Direction.WEST) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX - i, goalY)) {
          valid.remove(Direction.WEST);
        }
        if (valid.contains(Direction.EAST) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX + i, goalY)) {
          valid.remove(Direction.EAST);
        }
      } catch (TRADEException e) {
        log.error("Error calling isPassable.", e);
      }

      if (valid.isEmpty()) {
        break;
      }
    }
    return valid;
  }

  static public List<Direction> getEmptyCorners(Symbol x, Symbol y, Symbol radius) {
    // HACK: distance is currently either "one" or "two"
    //this is for checking the corners, i.e. NW, SE, SW, NE
    int rad = 1;//Integer.parseInt(distance.toString());
    if (radius.getName().equals("one")) {
      rad = 1;
    } else if (radius.getName().equals("two")) {
      rad = 2;
    } else {
      log.error("Invalid distance value: " + radius);
    }
    int goalX = Integer.parseInt(x.getName());
    int goalY = Integer.parseInt(y.getName());
    List<Direction> valid = new ArrayList<>(Arrays.asList(Direction.values())); //Start with all valid directions

    for (int i = 1; i <= rad; ++i) {
      try {
        if (valid.contains(Direction.NORTH) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX - i, goalY - i)) {
          //northwest
          valid.remove(Direction.NORTH);
        }
        if (valid.contains(Direction.SOUTH) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX + i, goalY + i)) {
          //southeast
          valid.remove(Direction.SOUTH);
        }
        if (valid.contains(Direction.WEST) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX - i, goalY + i)) {
          //southwest
          valid.remove(Direction.WEST);
        }
        if (valid.contains(Direction.EAST) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isEmpty")).call(boolean.class, goalX + i, goalY - i)) {
          //northeast
          valid.remove(Direction.EAST);
        }
      } catch (TRADEException e) {
        log.error("Error calling isPassable.", e);
      }

      if (valid.isEmpty()) {
        break;
      }
    }
    return valid;
  }

  /**
   * Checks which directions around a goal location are valid directions for an actor to stand.
   *
   * @param goalX
   * @param goalY
   * @param radius
   * @return
   */
  static public List<Direction> getPassableDirections(int goalX, int goalY, int radius) {

    List<Direction> valid = new ArrayList<>(Arrays.asList(Direction.values())); //Start with all valid directions
    for (int i = 1; i <= radius; ++i) {
      try {
        if (valid.contains(Direction.NORTH) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isPassable")).call(boolean.class, goalX, goalY - i)) {
          valid.remove(Direction.NORTH);
        }
        if (valid.contains(Direction.SOUTH) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isPassable")).call(boolean.class, goalX, goalY + i)) {
          valid.remove(Direction.SOUTH);
        }
        if (valid.contains(Direction.WEST) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isPassable")).call(boolean.class, goalX - i, goalY)) {
          valid.remove(Direction.WEST);
        }
        if (valid.contains(Direction.EAST) && !TRADE.getAvailableService(new TRADEServiceConstraints().name("isPassable")).call(boolean.class, goalX + i, goalY)) {
          valid.remove(Direction.EAST);
        }
      } catch (TRADEException e) {
        log.error("Error calling isPassable.", e);
      }

      if (valid.isEmpty()) {
        break;
      }
    }
    return valid;
  }


  /**
   * Check that input (x,y) has at least one empty adjacent square, and if not, find nearest
   * location that has an empty adjacent square.
   *
   * @param x
   * @param y
   * @return at(X, locX, locY)
   */
  static public Predicate getClosestEmptyAdjacent(Symbol x, Symbol y) {
    int goalX = Integer.parseInt(x.toString());
    int goalY = Integer.parseInt(y.toString());
    List<Pair<Integer, Integer>> spaces = new ArrayList<>();
    spaces.add(Pair.of(goalX, goalY));
    while (!spaces.isEmpty()) {
      Pair<Integer, Integer> space = spaces.remove(0);
      List<Direction> valid = Movement.getEmptyDirections(Factory.createSymbol(space.getLeft().toString()), Factory.createSymbol(space.getRight().toString()), Factory.createSymbol("one"));
      if (!valid.isEmpty()) {
        return Factory.createPredicate("at", "X", space.getLeft().toString(), space.getRight().toString());
      } else {
        valid = Movement.getPassableDirections(space.getLeft(), space.getRight(), 1);
        for (Direction d : valid) {
          int newX = space.getLeft();
          int newY = space.getRight();
          switch (d) {
            case NORTH:
              newY--;
              break;
            case SOUTH:
              newY++;
              break;
            case WEST:
              newX--;
              break;
            case EAST:
              newX++;
              break;
          }
          //add newX and newY to list
          spaces.add(Pair.of(newX, newY));
        }
      }
    }
    log.error("No empty space available near target.");
    return null;
  }

  /**
   * Check that input (x,y) is empty on all sides, and if not, find nearest
   * location that is empty on all sides.
   *
   * @param x
   * @param y
   * @return at(X, locX, locY)
   */
  static public Predicate getClosestAllEmptyAdjacent(Symbol x, Symbol y) {
    int goalX = Integer.parseInt(x.toString());
    int goalY = Integer.parseInt(y.toString());
    List<Pair<Integer, Integer>> spaces = new ArrayList<>();
    spaces.add(Pair.of(goalX, goalY));
    while (!spaces.isEmpty()) {
      Pair<Integer, Integer> space = spaces.remove(0);
      List<Direction> valid = Movement.getEmptyDirections(Factory.createSymbol(space.getLeft().toString()), Factory.createSymbol(space.getRight().toString()), Factory.createSymbol("one"));
      List<Direction> validcorners = Movement.getEmptyCorners(Factory.createSymbol(space.getLeft().toString()), Factory.createSymbol(space.getRight().toString()), Factory.createSymbol("one"));
      if (valid.size() == 4 && validcorners.size() == 4) {
        return new Predicate("at", "X", space.getLeft().toString(), space.getRight().toString());
      } else {
        for (Direction d : valid) {
          int newX = space.getLeft();
          int newY = space.getRight();
          switch (d) {
            case NORTH:
              newY--;
              break;
            case SOUTH:
              newY++;
              break;
            case WEST:
              newX--;
              break;
            case EAST:
              newX++;
              break;
          }
          //add newX and newY to list
          spaces.add(Pair.of(newX, newY));
        }
      }
    }
    log.error("No space available without unbreakable neighbors.");
    return null;
  }

  /**
   * Get the object at the specified location. Returns null if nothing is at that location.
   *
   * @param locX
   * @param locY
   * @return
   */
  static public Symbol getObjectAt(Symbol locX, Symbol locY) {
    Variable objVar = Factory.createVariable("OBJ");
    Predicate query = Factory.createPredicate("at", objVar, locX, locY);
    try {
      List<Map<Variable, Symbol>> result = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
      if (result.isEmpty()) {
        return null;
      } else if (result.size() > 1) {
        log.warn("More than one object found at: " + query);
      }
      return result.get(0).get(objVar);
    } catch (TRADEException e) {
      log.error("Error querying belief: " + query, e);
      return null;
    }
  }

  static public int getDistance(int x1, int y1, int x2, int y2) {
    return (Math.abs(x1 - x2) + Math.abs(y1 - y2));
  }

  /**
   * Get path to target object, so that actor is standing at the specified distance from the target object. If more
   * than one target object exists, this will choose the object with the least path weight.
   *
   * @param targetObject
   * @param distance
   * @return
   */
  static private List<Pair<Integer, Integer>> getPathToObjectHelper(Symbol targetObject, Symbol distance) {
    Variable XVar = new Variable("X");
    Variable YVar = new Variable("Y");
    Predicate query = Factory.createPredicate("at", targetObject, XVar, YVar);
    try {
      log.debug("getPathToObjectHelper: " + query);
      List<Map<Variable, Symbol>> result = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
      log.debug("results: " + result);
      if (result.size() == 0) {
        Predicate fluentCorrection = Factory.createPredicate("fluent_equals", Factory.createPredicate("world", targetObject), Factory.createSymbol("0"));
        log.warn("[getPathToObjectHelper] None found: " + targetObject + ". Asserting " + fluentCorrection + " to belief.");
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, fluentCorrection, MemoryLevel.EPISODIC);
        return null;
      } else if (targetObject.getName().equals("self")) {
        int objectX = Integer.parseInt(result.get(0).get(XVar).toString());
        int objectY = Integer.parseInt(result.get(0).get(YVar).toString());
        log.debug("Location of " + targetObject + ":" + objectX + ", " + objectY);
        List<Pair<Integer, Integer>> object = new ArrayList<>();
        object.add(Pair.of(objectX, objectY));
        return object;
      } else {
        //maybe we can have something like with less than X objects, find closest, over X objects, spiral
        List<Map<Variable, Symbol>> agentLoc = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, Factory.createPredicate("at", Factory.createSymbol("self"), XVar, YVar));
        Symbol agentX = agentLoc.get(0).get(XVar);
        Symbol agentY = agentLoc.get(0).get(YVar);

        Symbol xTemp;
        Symbol yTemp;
        List<Pair<Integer, Integer>> path = null;
        Integer tmpweight = null;
        Integer leastweight = null;

        //get neighbors to access
        for (Map<Variable, Symbol> r : result) {
          List<Pair<Integer, Integer>> tmppath = new ArrayList<>();
          xTemp = r.get(XVar);
          yTemp = r.get(YVar);
          tmpweight = getPathToLocationHelper(agentX, agentY, xTemp, yTemp, distance, tmppath);

          if (tmppath != null && tmpweight != null) {
            tmppath.add(Pair.of(Integer.parseInt(xTemp.toString()), Integer.parseInt(yTemp.toString())));

            if (path == null || (tmpweight <= leastweight)) {
              path = new ArrayList<>();
              path.addAll(tmppath);
              leastweight = tmpweight;
            }
          }
        }
        log.debug("Path to " + targetObject + " is " + path);
        return path;
      }
    } catch (TRADEException e) {
      log.error("Error while calculating path to object: " + targetObject, e);
      return null;
    }
  }

  /**
   * Get actor's new location if the actor is currently at (currX,currY), facing currDir, and moving one
   * step in direction specified by movement key (w/a/d/x)).
   *
   * @param currX       current X location
   * @param currY       current Y location
   * @param currDir     current cardinal direction
   * @param movementKey polycraft movement direction (w/a/d/x)
   * @return
   */
  static public Predicate getNewLocation(Symbol currX, Symbol currY, Symbol currDir, Symbol movementKey) {
    int goalX = Integer.parseInt(currX.toString());
    int goalY = Integer.parseInt(currY.toString());
    Direction currDirection = Direction.valueOf(currDir.toString().toUpperCase());
    switch (movementKey.toString()) {
      case "w": // foward
      case "W":
        switch (currDirection) {
          case NORTH:
            --goalY;
            break;
          case SOUTH:
            ++goalY;
            break;
          case EAST:
            ++goalX;
            break;
          case WEST:
            --goalX;
            break;
        }
        break;
      case "x": // backward
      case "X":
        switch (currDirection) {
          case NORTH:
            ++goalY;
            break;
          case SOUTH:
            --goalY;
            break;
          case EAST:
            --goalX;
            break;
          case WEST:
            ++goalX;
            break;
        }
        break;
      case "d": // strafe right
      case "D":
        switch (currDirection) {
          case NORTH:
            ++goalX;
            break;
          case SOUTH:
            --goalX;
            break;
          case EAST:
            ++goalY;
            break;
          case WEST:
            --goalY;
            break;
        }
        break;
      case "a": // strafe left
      case "A":
        switch (currDirection) {
          case NORTH:
            --goalX;
            break;
          case SOUTH:
            ++goalX;
            break;
          case EAST:
            --goalY;
            break;
          case WEST:
            ++goalY;
            break;
        }
        break;
      default:
        log.error("Not a valid movement direction: " + movementKey);
        return null;
    }

    Predicate newLocationPred = Factory.createPredicate("at", "X", Integer.toString(goalX), Integer.toString(goalY));
    return newLocationPred;
  }

  /**
   * For an actor currently at (currX,currY), this method plans a path so that the actor is a certain number
   * of steps (distance) from and facing (targetX,targetY).
   *
   * @param currX    input starting location X
   * @param currY    input starting location Y
   * @param targetX  input target location X
   * @param targetY  input target location Y
   * @param distance input distance from target location
   * @param path     output path
   * @return weight of path
   */
  static public Integer getPathToLocationHelper(Symbol currX, Symbol currY, Symbol targetX, Symbol targetY, Symbol distance, List<Pair<Integer, Integer>> path) {

    int playerX = Integer.parseInt(currX.toString());
    int playerY = Integer.parseInt(currY.toString());
    int goalX = Integer.parseInt(targetX.toString());
    int goalY = Integer.parseInt(targetY.toString());
    // HACK: distance is currently either "one" or "two"
    int dist = 1;//Integer.parseInt(distance.toString());
    if (distance.getName().equals("one")) {
      dist = 1;
    } else if (distance.getName().equals("two")) {
      dist = 2;
    } else {
      log.error("Invalid distance value: " + distance);
    }

    List<Direction> valid = Movement.getPassableDirections(goalX, goalY, dist);
    List<Pair<Integer, Integer>> minWeightPath = null;
    Integer minWeight = null;
    Integer tmpWeight = null;
    log.debug("Adjacent to: " + goalX + "," + goalY + " by " + dist);
    for (Direction d : valid) {
      List<Pair<Integer, Integer>> tmpPath = new ArrayList<>();

      int tempX = goalX;
      int tempY = goalY;

      switch (d) {
        case NORTH:
          tempY = goalY - dist;
          break;
        case SOUTH:
          tempY = goalY + dist;
          break;
        case WEST:
          tempX = goalX - dist;
          break;
        case EAST:
          tempX = goalX + dist;
          break;
      }

      try {
        tmpWeight = TRADE.getAvailableService(new TRADEServiceConstraints().name("findPath")).call(Integer.class, playerX, playerY, tempX, tempY, tmpPath);
      } catch (TRADEException e) {
        log.error("Couldn't call findPath:", e);
      }
      if (tmpPath != null && tmpWeight != null) {
        if (minWeightPath == null || (tmpWeight <= minWeight)) {
          minWeightPath = tmpPath;
          minWeight = tmpWeight;
        }
      }
    }

    // set min weight path to return path and return min weight
    if (minWeightPath != null) {
      path.clear();
      path.addAll(minWeightPath);
      return minWeight;
    } else {
      return null;
    }
  }

  /**
   * Get which direction to face in order to face the target location from the current location. This assumes the two
   * locations are one euclidean unit away from each other (i.e., not diagonal).
   *
   * @param currX   current X location
   * @param currY   current Y location
   * @param targetX target X location
   * @param targetY target Y location
   * @return cardinal direction
   */
  static public Symbol getDirectionToFace(Symbol currX, Symbol currY, Symbol targetX, Symbol targetY) {
    int agentX = Integer.parseInt(currX.toString());
    int agentY = Integer.parseInt(currY.toString());
    int x = Integer.parseInt(targetX.toString());
    int y = Integer.parseInt(targetY.toString());
    Symbol direction = null;
    if (agentX > x) {
      direction = Factory.createSymbol(Direction.WEST.toString().toLowerCase());
    } else if (agentX < x) {
      direction = Factory.createSymbol(Direction.EAST.toString().toLowerCase());
    } else if (agentY > y) {
      direction = Factory.createSymbol(Direction.NORTH.toString().toLowerCase());
    } else if (agentY < y) {
      direction = Factory.createSymbol(Direction.SOUTH.toString().toLowerCase());
    } else {
      log.warn("[getDirectionToFace] target location same as current location. Returning null.");
    }

    return direction;
  }

  /**
   * Get degrees to turn to get from currDir to targetDir.
   *
   * @param currDir   current cardinal direction
   * @param targetDir target cardinal direction
   * @return degrees to turn (-180 to 180). (-) left/counter-clockwise, (+) right/clockwise
   */
  static public Symbol getDegreesToTurn(Symbol currDir, Symbol targetDir) {
    Direction currDirEnum = Direction.valueOf(currDir.toString().toUpperCase());
    Direction targetDirEnum = Direction.valueOf(targetDir.toString().toUpperCase());

    int currDegrees = currDirEnum.getDegrees();
    int targetDegrees = targetDirEnum.getDegrees();
    int degreesToTurn = targetDegrees - currDegrees;
    if (degreesToTurn > 180) {
      degreesToTurn = degreesToTurn - 360;
    } else if (degreesToTurn < -180) {
      degreesToTurn = degreesToTurn + 360;
    }

    return Factory.createSymbol(Integer.toString(degreesToTurn));
  }

  /**
   * Get the locations as "at(OBJ,x,y)".
   *
   * @param currX
   * @param currY
   * @param targetX
   * @param targetY
   * @param distance
   * @return
   */
  static public List<Predicate> getPathToLocation(Symbol currX, Symbol currY, Symbol targetX, Symbol targetY, Symbol distance) {
    List<Pair<Integer, Integer>> path = new ArrayList<>();
    getPathToLocationHelper(currX, currY, targetX, targetY, distance, path);
    if (path == null || path.isEmpty()) {
      log.error("No path found to (" + targetX + ", " + targetY + ").");
      return null;
    }
    List<Predicate> pathPreds = new ArrayList<>();
    for (Pair<Integer, Integer> step : path) {
      pathPreds.add(Factory.createPredicate("at", "OBJ", Integer.toString(step.getLeft()), Integer.toString(step.getRight())));
    }
    return pathPreds;
  }

  /**
   * Get the locations as "at(OBJ,x,y)".
   *
   * @param targetObject
   * @param distance
   * @return
   */
  static public List<Predicate> getPathToObject(Symbol targetObject, Symbol distance) {
    List<Pair<Integer, Integer>> path = getPathToObjectHelper(targetObject, distance);
    if (path == null || path.isEmpty()) {
      log.error("No path found.");
      return null;
    }
    List<Predicate> pathPreds = new ArrayList<>();
    for (Pair<Integer, Integer> step : path) {
      pathPreds.add(Factory.createPredicate("at", "OBJ", Integer.toString(step.getLeft()), Integer.toString(step.getRight())));
    }
    return pathPreds;
  }

  /**
   * Get next cardinal direction if actor is facing currDirection and turning specified degrees left/right.
   *
   * @param currDirection current cardinal direction
   * @param degrees       degrees to turn (-) left/counter-clockwise, (+) right/clockwise
   * @return
   */
  static public Symbol getNextDirection(Symbol currDirection, Symbol degrees) {
    Direction direction = Direction.valueOf(currDirection.toString().toUpperCase());
    int degreesToTurn = Integer.parseInt(degrees.toString());
    int newDirectionInDegrees = (direction.getDegrees() + degreesToTurn + 360) % 360;
    Direction newDirection = Direction.getDirection(newDirectionInDegrees);
    if (newDirection == null) {
      log.error("Could not calculate new direction from currDirection: " + currDirection + " degrees: " + degrees);
      return null;
    }

    return Factory.createSymbol(newDirection.toString().toLowerCase());
  }

  /**
   * Get location directly in front of actor that's currently at (currX,currY) and facing currDir.
   *
   * @param currX   current X location
   * @param currY   current Y location
   * @param currDir current cardinal direction
   * @return at(X, locX, locY)
   */
  static public Predicate getInFront(Symbol currX, Symbol currY, Symbol currDir) {
    int targetX = Integer.parseInt(currX.toString());
    int targetY = Integer.parseInt(currY.toString());
    Direction dir = Direction.valueOf(currDir.toString().toUpperCase());
    switch (dir) {
      case NORTH:
        --targetY;
        break;
      case EAST:
        ++targetX;
        break;
      case SOUTH:
        ++targetY;
        break;
      case WEST:
        --targetX;
        break;
    }

    // create at(object,x,y) predicate. this location isn't for a particular object so using variable X for object
    Predicate locationPred = Factory.createPredicate("at", "X", Integer.toString(targetX), Integer.toString(targetY));
    return locationPred;
  }


  /**
   * Get location of the closest object from (currX,currY).
   *
   * @param currX        current X location of actor
   * @param currY        current Y location of actor
   * @param targetObject target object
   * @return at(X, locX, locY)
   */
  static public Predicate getClosestObject(Symbol currX, Symbol currY, Symbol targetObject) {
    Variable xVar = new Variable("X");
    Variable yVar = new Variable("Y");
    Predicate query = Factory.createPredicate("at", targetObject, xVar, yVar);
    Symbol targetX = null;
    Symbol targetY = null;
    try {
      List<Map<Variable, Symbol>> queryResults = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
      if (queryResults.size() == 0) {
        Predicate fluentCorrection = Factory.createPredicate("fluent_equals", Factory.createPredicate("world", targetObject), Factory.createSymbol("0"));
        log.warn("[getClosestObject] None found: " + targetObject + ". Asserting " + fluentCorrection + " to belief.");
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, fluentCorrection, MemoryLevel.EPISODIC);
        return null;
      } else if (queryResults.size() == 1) {
        targetX = queryResults.get(0).get(xVar);
        targetY = queryResults.get(0).get(yVar);
      } else {
        int minDist = Integer.MAX_VALUE;
        int tmpDist;

        // find closest object
        int currLocX = Integer.parseInt(currX.toString());
        int currLocY = Integer.parseInt(currY.toString());
        for (Map<Variable, Symbol> result : queryResults) {
          Symbol tmpX = result.get(xVar);
          Symbol tmpY = result.get(yVar);
          int objectX = Integer.parseInt(tmpX.toString());
          int objectY = Integer.parseInt(tmpY.toString());
          tmpDist = getDistance(currLocX, currLocY, objectX, objectY);
          if (tmpDist < minDist) {
            minDist = tmpDist;
            targetX = tmpX;
            targetY = tmpY;
          }
        }
      }
    } catch (TRADEException e) {
      log.error("Error query belief to get closest object of type: " + targetObject);
      return null;
    }

    // create at(object,x,y) predicate
    return Factory.createPredicate("at", targetObject, targetX, targetY);
  }
}
