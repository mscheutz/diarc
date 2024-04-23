/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pathfinding;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import edu.tufts.hrilab.diarc.DiarcComponent;

import java.util.*;

public class PathPlannerComponent extends DiarcComponent {

  protected Map<Integer, Map<Integer, Float>> grid = new HashMap<>();
  protected boolean registerForBeliefNotifications = true;
  protected boolean shouldLogBeliefMsg = true;

  public PathPlannerComponent() {
    super();
    log.info("Path Planner running");
  }

  @Override
  protected void init() {
    this.shouldRunExecutionLoop = true;
    this.executionLoopCycleTime = 500;
  }

  @Override
  protected void executionLoop() {
    if (registerForBeliefNotifications) {
      try {
        TRADEServiceInfo registerService= TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForNotification").argTypes(Term.class,TRADEServiceInfo.class));
        if (!shouldLogBeliefMsg) {
          log.warn("registerForNotification service now available.");
        }
        Term atPredicate = Factory.createPredicate("at(X,Y,Z)");
        Term typePredicate = Factory.createPredicate("subtype(X,Y)");
        try {
          TRADEServiceInfo learnService = getMyService("updateMap", Term.class, List.class);
          registerService.call(void.class, atPredicate, learnService);
          registerService.call(void.class, typePredicate, learnService);
          registerForBeliefNotifications = false; //don't want to keep on registering
          this.shouldRunExecutionLoop = false; // no need to keep running executionLoop
        } catch (TRADEException e) {
          log.warn("Could not register for belief notifications. Will try again.", e);
          registerForBeliefNotifications = true;
        }
      } catch (TRADEException e){
        if (shouldLogBeliefMsg) {
          shouldLogBeliefMsg = false;
          log.warn("registerForNotification service is not yet available. Will only print once.",e);
        }
      }
    }
  }

  @TRADEService
  public void resetMap() {
    grid = new HashMap<>();
  }

  @TRADEService
  public void updateMap(Term atTerm, List<Map<Variable, Symbol>> bindings) {
    if (log.isDebugEnabled()) {
      log.debug("map update: " + atTerm.copyWithNewBindings(bindings));
    }

    // if atTerm in negated (e.g., not(at(self,1,2))), treat it as if nothing is at that location
    boolean retraction = false;
    if (atTerm.getName().equals("not")) {
      retraction = true;
      atTerm = atTerm.toUnnegatedForm();
    }

    if (atTerm.getName().equals("subtype")) {
      // if subtype(object,type), update the weight of all object instances in map
      Symbol objVar = atTerm.get(0);
      List<Map<Variable, Symbol>> atBindings = new ArrayList<>();
      for (Map<Variable, Symbol> binding : bindings) {
        Symbol obj = binding.get(objVar);
        Term queryTerm = Factory.createPredicate("at(" + obj + ",X,Y)");
        try {
          List<Map<Variable, Symbol>> queryResults = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class)).call(List.class, queryTerm);
          for (Map<Variable, Symbol> result : queryResults) {
            result.put(new Variable("O"), obj);
          }
          atBindings.addAll(queryResults);
        } catch (TRADEException e) {
          log.error("Couldn't call queryBelief in updateMap for subtype notification.", e);
        }
      }
      updateMapHelper(Factory.createPredicate("at(O,X,Y)"), atBindings, false);
    } else {
      updateMapHelper(atTerm, bindings, retraction);
    }
  }

  /**
   * Local wrapper around querySupport call to Belief to handle try-catch
   * to make local usage in conditionals less ugly.
   * @param query
   * @return
   */
  private boolean querySupport(Predicate query) {
    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(Boolean.class, query);
    } catch (TRADEException e) {
      log.error("Error querying belief for: " + query, e);
    }

    return false;
  }

  void updateMapHelper(Term atTerm, List<Map<Variable, Symbol>> bindings, Boolean retraction) {
    Symbol objVar = atTerm.get(0);
    Symbol xVar = atTerm.get(1);
    Symbol yVar = atTerm.get(2);
    boolean finalRetraction = retraction;
    bindings.forEach(binding -> {
      String obj = binding.get(objVar).toString();
      Float weight = 0.0f;
      int x = Integer.parseInt(binding.get(xVar).toString());
      int y = Integer.parseInt(binding.get(yVar).toString());
      // TODO: change these to check for "passable" type and add to type hierarchy in belief
      if (finalRetraction || obj.equals("self") || obj.equals("air") || obj.equals("open_door")) {
        weight = 0.0f;
      } else if (querySupport(Factory.createPredicate("typeobject", obj, "air"))) {
        // type of "air"
        weight = 0.1f;
      } else if (querySupport(Factory.createPredicate("type", obj, "breakable"))) {
        // breakable
        weight = 0.5f;
      } else {
        // assume non-passable
        weight = 1.0f;
      }

      Map<Integer, Float> column = grid.get(x);
      if (column == null) {
        column = new HashMap<>();
      }
      column.put(y, weight);
      grid.put(x, column);
    });
  }

  @TRADEService
  public boolean isPassable(int x, int y) {
    if (grid.get(x) == null || grid.get(x).get(y) == null) {
      return false;
    } else if (grid.get(x).get(y) == 1.0f) {
      return false;
    } else {
      return true;
    }
  }

  @TRADEService
  public boolean isEmpty(int x, int y) {
    if (grid.get(x) == null || grid.get(x).get(y) == null) {
      return false;
    } else if (grid.get(x).get(y) <= 0.1f) {
      return true;
    } else {
      return false;
    }
  }

  @TRADEService
  @Action
  public boolean isBreakable(Symbol locx, Symbol locy) {
    int x = Integer.parseInt(locx.toString());
    int y = Integer.parseInt(locy.toString());

    if (grid.get(x) == null || grid.get(x).get(y) == null) {
      return false;
    } else if (grid.get(x).get(y) == 0.5f) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Wrapper around findPath that makes usage from an action script easier. The returned path is a list of at(ACTOR,x,y)
   * predicates defining each step of the path.
   *
   * @param startX
   * @param startY
   * @param goalX
   * @param goalY
   * @param path
   * @return
   */
  @TRADEService
  @Action
  public Justification findPath(Symbol startX, Symbol startY, Symbol goalX, Symbol goalY, List<Predicate> path) {
    List<Pair<Integer, Integer>> tmpPath = new ArrayList<>();

    findPath(Integer.parseInt(startX.toString()),
            Integer.parseInt(startY.toString()),
            Integer.parseInt(goalX.toString()),
            Integer.parseInt(goalY.toString()),
            tmpPath);

    if (tmpPath == null) {
      log.debug("No breakable path found.");
      return new ConditionJustification(false, Factory.createPredicate("found(path)"));
    }

    // create step predicates of form at(ACTOR,x,y).
    tmpPath.forEach(step -> path.add(Factory.createPredicate("at", "ACTOR", step.getLeft().toString(), step.getRight().toString())));
    return new ConditionJustification(true);
  }


  /**
   * Find path from [startX,startY] to [goalX,goalY] using A*. This finds the path with min weight, which can
   * be a path containing objects in the way.
   *
   * @param startX
   * @param startY
   * @param goalX
   * @param goalY
   * @param path
   * @return
   */
  @TRADEService
  public Integer findPath(int startX, int startY, int goalX, int goalY, List<Pair<Integer, Integer>> path) {
    // bounds checking start and goal locations
    if (grid.get(startX) == null || grid.get(startX).get(startY) == null) {
      log.warn("Starting location is not on map: " + startX + " " + startY);
      return null;
    }
    if (grid.get(goalX) == null || grid.get(goalX).get(goalY) == null) {
      log.warn("Goal location is not on map: " + goalX + " " + goalY);
      return null;
    }

    // already at goal location
    if (startX == goalX && startY == goalY) {
      return 0;
    }

    List<Point> open = new ArrayList<>();
    List<Point> closed = new ArrayList<>();
    Point start = new Point(startX, startY, null);
    Point end = new Point(goalX, goalY, null);
    open.add(start);
    Point currentPoint;

    while (open.size() > 0) {
      currentPoint = getLowest(open);
      if (currentPoint.equals(end)) {
        log.trace("Path found.");
        path.addAll(generatePath(currentPoint));
        return currentPoint.f;
      }
      closed.add(currentPoint);
      open.remove(currentPoint);

      log.trace("Exploring around: " + currentPoint);

      List<Pair<Point, Float>> neighbors = getNeighbors(currentPoint, grid);
      for (Pair<Point, Float> pair : neighbors) {
        Point p = pair.getKey();
        if (closed.contains(p)) {
          continue;
        }
        //adding weight from breakable blocks
        float w = 0;
        //if (breakable) {
        w = 10 * pair.getValue();
        //}
        p.g = currentPoint.g + 1 + (int) w;
        p.h = calcDist(p, end);
        p.f = p.g + p.h;
        if (!open.contains(p)) {
          open.add(p);
          continue;
        }
        for (Point o : open) {
          if (p.equals(o)) {
            //p is new, o is old. minimize f
            if (p.f < o.f) {
              open.remove(o);
              open.add(p);
            }
            break;
          }
        }
      }
    }
    log.warn("No path found. Returning null.");
    return null;
  }

  /**
   * Wrapper around findDirectPath that makes usage from an action script easier. The returned path is a list of at(agent,x,y)
   * predicates defining each step of the path.
   *
   * @param startX
   * @param startY
   * @param goalX
   * @param goalY
   * @param path
   * @return
   */
  @TRADEService
  @Action
  public Justification findDirectPath(Symbol startX, Symbol startY, Symbol goalX, Symbol goalY, List<Symbol> path) {
    List<Pair<Integer, Integer>> tmpPath = findDirectPath(
            Integer.parseInt(startX.toString()),
            Integer.parseInt(startY.toString()),
            Integer.parseInt(goalX.toString()),
            Integer.parseInt(goalY.toString()));

    if (tmpPath == null) {
      log.debug("No direct path found.");
      return new ConditionJustification(false, Factory.createPredicate("found(path)"));
    }

    // create step predicates of form at(object,x,y). Using variable X because the object/agent is not known in this context
    tmpPath.forEach(step -> path.add(Factory.createPredicate("at", "X", step.getLeft().toString(), step.getRight().toString())));
    return new ConditionJustification(true);
  }

  /**
   * Euclidean path from [startX,startY] to [goalX,goalY].
   * TODO: replace this with a findMostClearPath implementation that tries to avoid as many obstacles as possible
   *
   * @param startX
   * @param startY
   * @param goalX
   * @param goalY
   * @return
   */
  @TRADEService
  public List<Pair<Integer, Integer>> findDirectPath(int startX, int startY, int goalX, int goalY) {

    List<Pair<Integer, Integer>> path = new ArrayList<>();
    if (startX > goalX) {
      for (int x = startX; x > goalX; x--) {
        path.add(new MutablePair<>(x, startY));
      }
    } else {
      for (int x = startX; x < goalX; x++) {
        path.add(new MutablePair<>(x, startY));
      }
    }

    if (startY > goalY) {
      for (int y = startY; y >= goalY; y--) {
        path.add(new MutablePair<>(goalX, y));
      }
    } else {
      for (int y = startY; y <= goalY; y++) {
        path.add(new MutablePair<>(goalX, y));
      }
    }

    return path;
  }

  private Point getLowest(List<Point> open) {
    Point lowest = open.get(0);
    for (Point p : open) {
      if (lowest.f > p.f) {
        lowest = p;
      }
    }
    return lowest;
  }

  private List<Pair<Integer, Integer>> generatePath(Point endPoint) {
    List<Pair<Integer, Integer>> path = new ArrayList<>();

    while (endPoint != null) {
      path.add(new MutablePair<>(endPoint.x, endPoint.y));
      endPoint = endPoint.parent;
    }

    path.remove(path.size() - 1); // remove start location (agent is already there)
    Collections.reverse(path); //this is super inefficient, just make it a different datatype to begin with
    return path;
  }


  private int calcDist(Point a, Point b) {
    return (Math.abs(a.x - b.x) + Math.abs(a.y - b.y));
  }

  private List<Pair<Point, Float>> getNeighbors(Point p, Map<Integer, Map<Integer, Float>> grid) {
    List<Pair<Point, Float>> n = new ArrayList<>();

    if (grid.get(p.x + 1) != null && grid.get(p.x + 1).get(p.y) != null && grid.get(p.x + 1).get(p.y) != 1) {
      n.add(Pair.of(new Point(p.x + 1, p.y, p), grid.get(p.x + 1).get(p.y)));
    }
    if (grid.get(p.x - 1) != null && grid.get(p.x - 1).get(p.y) != null && grid.get(p.x - 1).get(p.y) != 1) {
      n.add(Pair.of(new Point(p.x - 1, p.y, p), grid.get(p.x - 1).get(p.y)));
    }
    if (grid.get(p.x) != null && grid.get(p.x).get(p.y + 1) != null && grid.get(p.x).get(p.y + 1) != 1) {
      n.add(Pair.of(new Point(p.x, p.y + 1, p), grid.get(p.x).get(p.y + 1)));
    }
    if (grid.get(p.x) != null && grid.get(p.x).get(p.y - 1) != null && grid.get(p.x).get(p.y - 1) != 1) {
      n.add(Pair.of(new Point(p.x, p.y - 1, p), grid.get(p.x).get(p.y - 1)));
    }

    return n;
  }


}
