import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;

// Approach ?to_actor, trying to get ?distance away, and face ?to_actor
// ?actor is assumed to currently be facing ?from_obj, at a distance of ?from_dist
() = approach_actor(Symbol ?to_actor:agent, Symbol ?to_dist:distance) {
  Symbol !x;
  Symbol !y;
  Symbol ?from_obj:physical;
  Symbol ?from_dist:distance;

  conditions : {
    pre infer : not(equals(?to_actor, self)); // don't want to allow approaching self
    pre infer : facing_obj(?actor, ?from_obj, ?from_dist);
  }
  effects : {
    success infer : facing_obj(?actor, ?to_actor, ?to_dist);
    success infer : not(facing_obj(?actor, ?from_obj, ?from_dist));
    success obs : not(discrepancy(!x,!y));
  }

  op:log("info", ">> approach_actor ?to_actor");

  act:approach_actor_helper(?to_actor, ?to_dist);
}

() = approach_actor_helper(Symbol ?to_actor:agent, Symbol ?to_dist:distance) {
  Symbol !beliefX;
  Symbol !beliefY;
  java.util.List !path;
  java.lang.Integer !size;
  Predicate !targetLoc;
  Symbol !targetX;
  Symbol !targetY;
  Symbol !step;
  Symbol !stepX;
  Symbol !stepY;
  Symbol !obj;
  java.lang.Integer !count = 0;
  Predicate !lastKnownLoc;
  Symbol !lastKnownX;
  Symbol !lastKnownY;

  conditions : {
    pre infer : at(?to_actor, !lastKnownX, !lastKnownY);
  }

  op:log("debug", "[approach_actor_helper] ?to_actor.");

  // retract (potentially) old ?to_actor location from belief
  !lastKnownLoc = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?to_actor, !lastKnownX, !lastKnownY)");
  act:retractBelief(!lastKnownLoc);

  // observe new ?to_actor location (which also assert it to belief)
  if (~obs:at(?to_actor, !targetX, !targetY)) {
    // if can't observe actor, must be in a different room --> use last known location
    op:log("debug", "[approach_actor_helper] can't observe ?to_actor so teleporting to last known location !lastKnownLoc.");
    !targetX = op:newObject("edu.tufts.hrilab.fol.Symbol", !lastKnownX);
    !targetY = op:newObject("edu.tufts.hrilab.fol.Symbol", !lastKnownY);

    act:assertBelief(!lastKnownLoc);
  }

  op:log("debug", "[approach_actor_helper] at(?to_actor, !targetX, !targetY).");
  act:teleport_to(!targetX,!targetY, ?to_dist);

  (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?to_actor, ?to_dist);
  while (op:isNull(!path) && op:lt(!count, 5)) {
    (!count) = op:++(!count);
    act:nop();
    act:nop();

    (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?to_actor, ?to_dist);
  }

  if (op:isNull(!path)) {
    exit(FAIL_PRECONDITIONS, found(pathTo, ?to_actor));
  }

  // remove the last element of the path (don't want to move into same location as target object)
  (!size) = op:invokeMethod(!path, "size");
  !size = op:--(!size);
  (!targetLoc)  = op:invokeMethod(!path, "remove", !size);

  foreach (!step : !path) {
    // if the targetLocation changes, then re-plan
    if(~obs:at(?to_actor, !targetX, !targetY)) {
      op:log("debug", "[approach_actor_helper] ?to_actor moved...");
      act:approach_actor_helper(?to_actor, ?to_dist);
      op:log("debug", "[approach_actor_helper] returning from at(?to_actor, !targetX, !targetY)");
      return;
    }

    // !step preds are in form at(obj,x,y)
    (!stepX) = op:invokeMethod(!step, "get", 1);
    (!stepY) = op:invokeMethod(!step, "get", 2);
    act:faceLocation(!stepX, !stepY);

    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !stepX, !stepY);
    if (~op:isNull(!obj) && act:isBreakable(!stepX,!stepY)) {
      act:breakBlock();
    }

    act:moveForward();
  }

  act:faceLocation(!targetX,!targetY);
  op:log("debug", "[approach_actor_helper] Done following path.");
}

// Approach ?to_obj, trying to get ?to_dist away, and face ?to_obj
// ?actor is assumed to currently be facing ?from_obj, at a distance of ?from_dist
() = approach_object(Symbol ?to_obj:physobj, Symbol ?to_dist:distance) {
  Symbol !currX;
  Symbol !currY;
  Symbol !objX;
  Symbol !objY;
  java.util.List !path;
  java.lang.Integer !size;
  Predicate !objLoc;
  Symbol !step;
  Symbol !stepX;
  Symbol !stepY;
  Symbol !obj;
  Symbol !x;
  Symbol !y;
  int !count = 0;
  Symbol ?from_obj:physical;
  Symbol ?from_dist:distance;

  conditions : {
    pre infer : fluent_geq(world(?to_obj), 1);
    // don't really care what these values are in the real world (i.e., don't obs) -- only here for planner
    pre infer : facing_obj(?actor, ?from_obj, ?from_dist);
    pre infer : at(?actor, !currX, !currY);
  }
  effects : {
    success infer : not(facing_obj(?actor, ?from_obj, ?from_dist));
    success infer : facing_obj(?actor, ?to_obj, ?to_dist); // not obs so that post-cond recovery mechanism isn't used
    success obs : not(discrepancy(!x,!y));
  }

  // get closest ?to_obj
  (!objLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getClosestObject", !currX, !currY, ?to_obj);
  if (op:isNull(!objLoc)) {
    exit(FAIL_PRECONDITIONS, found(pathTo, ?to_obj));
  }
  (!objX) = op:invokeMethod(!objLoc, "get", 1);
  (!objY) = op:invokeMethod(!objLoc, "get", 2);

  op:log("info", ">> approach_object ?to_obj by ?to_dist at (!objX,!objY)");
  op:log("debug", "currently facing ?from_obj by ?from_dist");
  if (~op:equals(?to_obj,?from_obj) || ~op:equals(?to_dist,?from_dist)) {
    act:teleport_to(!objX,!objY,?to_dist);

    // Gets the location of ?to_obj (in case of more than 1, gets closest)
    (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?to_obj, ?to_dist);
    while (op:isNull(!path) && op:lt(!count, 5)) {
      (!count) = op:++(!count);
      act:nop();
      act:nop();

      (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?to_obj, ?to_dist);
    }

    if (op:isNull(!path)) {
      exit(FAIL_PRECONDITIONS, found(pathTo, ?to_obj));
    }

    // get ?to_obj location as first element in path
    (!size) = op:invokeMethod(!path, "size");
    !size = op:--(!size);
    (!objLoc) = op:invokeMethod(!path, "remove", !size);
    (!objX) = op:invokeMethod(!objLoc, "get", 1);
    (!objY) = op:invokeMethod(!objLoc, "get", 2);

    // follow path
    op:log("info", "    Found path. Following path to object at (!objX,!objY)...");
    foreach (!step : !path) {
      // !step preds are in form at(obj,x,y)
      (!stepX) = op:invokeMethod(!step, "get", 1);
      (!stepY) = op:invokeMethod(!step, "get", 2);
      act:faceLocation(!stepX, !stepY);

      // TODO: this is problematic bc we're still breaking objects which might break the plan later on
      // first check if there's an object in the way, and if so, if it's breakable
      (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !stepX, !stepY);
      if (~op:isNull(!obj) && act:isBreakable(!stepX,!stepY)) {
        op:log("info", "    Block in path. Breaking object: !obj");
        // perform primitive break bc we don't know which break_and_pickup version to call (object dependent)
        // moveForward below will handle updating inventory when picking up objects or location discrepancies
        act:breakBlock();
      }

      act:moveForward();
    }

    act:faceLocation(!objX,!objY);
  }

  // if ?to_dist is two, and there's an object one space away, break the object directly in front of ?actor
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  java.lang.String !distStr;
  (!distStr) = op:invokeMethod(?to_dist, "getName");
  if (op:equals(!distStr, "two")) {
    obs:facing(?actor, !currDir); // get current ?actor direction
    (!currX) = op:setNull();
    (!currY) = op:setNull();
    obs:at(?actor, !currX, !currY); // get current ?actor location
    (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
    (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
    (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);
    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement","getObjectAt",!inFrontX,!inFrontY);
    if (~op:isNull(!obj)) {
      op:log("info", "  Block in between ?actor and ?to_obj. Breaking object: !obj");
      // perform primitive break bc we don't know which break_and_pickup version to call (object dependent)
      // moveForward below will handle updating inventory when picking up objects or location discrepancies
      act:breakBlock();
      act:moveForward();
      act:moveBack();
    }
  }
  op:log("debug", "Done following path.");
}

() = turn(Symbol ?degrees) {
  Symbol !currDir;
  Symbol !newDir;
  Symbol !turnDir;
  Symbol !actionCost;

  conditions : {
    pre infer : facing(?actor, !currDir);
    pre infer : fluent_equals(cost_1(smooth_turn),!actionCost); // get expected action cost that will be checked in post-condition
  }
  effects : {
    success infer : not(facing(?actor, !currDir));
    success obs : facing(?actor, !newDir);
    success obs : fluent_equals(cost_1(smooth_turn),!actionCost);
//    success infer : fluent_increase(totalcost(), cost_1(smooth_turn));
  }

  op:log("debug", "[turn] degrees: ?degrees currDir: !currDir");
  act:smoothTurn(?degrees);
  (!newDir) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getNextDirection", !currDir, ?degrees);
  op:log("debug", "[turn] newDir: !newDir");
}

() = moveForward() {
  Symbol !x;
  Symbol !y;
  effects : {
     success obs : not(discrepancy(!x,!y));
  }
  act:move("W");
}

() = moveBack() {
  Symbol !x;
  Symbol !y;
  effects : {
     success obs : not(discrepancy(!x,!y));
  }
  act:move("X");
}

() = moveLeft() {
  Symbol !x;
  Symbol !y;
  effects : {
     success obs : not(discrepancy(!x,!y));
  }
  act:move("A");
}

() = moveRight() {
  Symbol !x;
  Symbol !y;
  effects : {
     success obs : not(discrepancy(!x,!y));
  }

  act:move("D");
}

// special moveforward that does not look for discrepancies bc it's used to pickup objects, although it's
// used non-deterministically
() = moveToEnsurePickup() {
  act:move("W"); // move forward without discrepancy check
  act:move("X"); // move back without discrepancy check
}

//assumes target is not the current location of actor
() = teleport_to(Symbol ?targetX, Symbol ?targetY, Symbol ?distance) {
  Symbol !obj;
  Symbol !startX;
  Symbol !startY;
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Predicate !newTarget;
  Symbol !newTargetX;
  Symbol !newTargetY;
  Symbol !newDir;
  java.util.List !valid;
  java.util.Set !targetSet;
  java.util.Set !inroom;
  java.util.List !path;
  Predicate !entrywayPred;
  Symbol !entryX;
  Symbol !entryY;
  java.lang.String !objName;
  int !size;
  int !entryIndex=-1;
  Symbol !singledist;
  Variable !actorVar;
  edu.tufts.hrilab.action.justification.Justification !failJustification;
  Predicate !condition;

  conditions : {
    pre infer: at(?actor,!startX,!startY);
    pre infer: facing(?actor,!currDir);
  }
  effects : {
    success obs : at(?actor,!newTargetX,!newTargetY);
    success obs : facing(?actor, !newDir);
  }

  (!valid) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement","getEmptyDirections",?targetX,?targetY,?distance);
  if (op:invokeMethod(!valid,"isEmpty")) {
    (!newTarget) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement","getClosestEmptyAdjacent",?targetX,?targetY);
    if (op:isNull(!newTarget)) {
      (!condition) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","found(pathTo,?targetX,?targetY)");
      (!failJustification) = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification",false,!condition);
      (!condition) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","teleport_to(?actor,?targetX,?targetY,one)");
      op:invokeMethod(!failJustification,"setStep",!condition);
      exit(FAIL_RETURNVALUE, !failJustification);
    }
    (?targetX) = op:invokeMethod(!newTarget,"get",1);
    (?targetY) = op:invokeMethod(!newTarget,"get",2);
    (!singledist) = op:newObject("edu.tufts.hrilab.fol.Symbol","one");
  }

  (!actorVar) = op:newObject("edu.tufts.hrilab.fol.Variable","ACTOR");
  (!targetSet) = op:newObject("java.util.HashSet");
  if (op:isNull(!newTarget)) {
    (!newTarget) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","at", !actorVar,?targetX,?targetY);
  }
  op:add(!targetSet,!newTarget);
  op:log("debug","checking !targetSet in room");
  (!inroom) = act:filterPointsNotInCurrentRoom(!targetSet);
  while (op:invokeMethod(!inroom,"isEmpty")) {
    // reset currX and currY so they can be bound to curr state in observation
    (!currX) = op:setNull();
    (!currY) = op:setNull();
    obs: at(?actor,!currX,!currY);

    // find path
    op:log("debug","!currX !currY to ?targetX ?targetY");
    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", ?targetX, ?targetY);
    if (op:isNull(!obj)) {
      (!path) = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
      act:findPath(!currX,!currY,?targetX,?targetY,!path);
    } else {
      (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement","getPathToLocation",!currX,!currY,?targetX,?targetY,?distance);
    }

    op:log("debug","path is !path");
    if (op:isNull(!path)) {
      exit(FAIL_PRECONDITIONS, found(pathTo,?targetX,?targetY));
    }

    // find first step along path that is an entryway
    (!size) = op:invokeMethod(!path,"size");
    (!entryIndex) = op:newObject("java.lang.Integer", -1);
    for (!index = 0; !index lt !size; !index ++) {
      // !step preds are in form at(ACTOR,x,y)
      (!entrywayPred) = op:invokeMethod(!path,"get",!index);
      (!entryX) = op:invokeMethod(!entrywayPred, "get", 1);
      (!entryY) = op:invokeMethod(!entrywayPred, "get", 2);
      (!entrywayPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","entryway",!entryX,!entryY);
      if (act:querySupport(!entrywayPred)) {
        // found entryway
        op:log("debug", "found entryway in path at index: !index");
        (!entryIndex) = op:newObject("java.lang.Integer", !index);
        (!index) = op:newObject("java.lang.Integer", !size); // exit out of for loop
      }
    }

    op:log("debug","entryIndex: !entryIndex");
    if (op:equals(!entryIndex,-1)) {
      //presumeably the target is next to a non-entry space in this room
      //or we're on an entry space and shouldn't teleport, just face first step of path and enter door
      (!entrywayPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "entryway(!currX,!currY)");
      if (act:querySupport(!entrywayPred)) {
        op:log("debug","standing in entryway at !entrywayPred");
        (!newTarget) = op:invokeMethod(!path,"get",0);
        (!entryX) = op:invokeMethod(!newTarget,"get",1);
        (!entryY) = op:invokeMethod(!newTarget,"get",2);
        op:log("debug","standing in entryway at !entrywayPred");
        act:faceLocation(!entryX,!entryY);
        act:enter_door(door);
      } else {
        (!size) = op:invokeMethod(!path,"size");
        !size = op:--(!size);
        (!newTarget) = op:invokeMethod(!path,"get",!size);
        (?targetX) = op:invokeMethod(!newTarget,"get",1);
        (?targetY) = op:invokeMethod(!newTarget,"get",2);
        op:add(!inroom,!newTarget);
      }
    } else {
      // teleport to entryway and move through it
      (!entrywayPred) = op:invokeMethod(!path,"get",!entryIndex);

      (!entryX) = op:invokeMethod(!entrywayPred,"get",1);
      (!entryY) = op:invokeMethod(!entrywayPred,"get",2);
      op:log("info", "    teleporting to entryway at location (!entryX, !entryY) path index !entryIndex");
      act:teleport_helper(!entryX,!entryY,one);

      (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !entryX, !entryY);
      if (~op:isNull(!obj)) {
        (!objName) = op:invokeMethod(!obj,"getName");
        if (op:equals(!objName,"door")) {
          act:open_door(door);
        } elseif (~op:equals(!objName,"open_door")) {
          op:log("debug", "Obj in path is !objName");
          act:breakBlock();
          act:moveForward();
        } else {
          act:moveForward();
        }
      } else {
        act:moveForward();
      }

      !entryIndex = op:++(!entryIndex);
      (!size) = op:invokeMethod(!path,"size");
      op:log("debug", "    entryIndex: !entryIndex path size: !size");
      if (op:lt(!entryIndex, !size)) {
        (!entrywayPred) = op:invokeMethod(!path,"get",!entryIndex);
        (!entryX) = op:invokeMethod(!entrywayPred,"get",1);
        (!entryY) = op:invokeMethod(!entrywayPred,"get",2);
        op:log("debug", "next step in path is !entrywayPred");

        (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !entryX, !entryY);
        if (~op:isNull(!obj)) {
          op:log("debug", "Obj in path is !obj");
          (!objName) = op:invokeMethod(!obj,"getName");
          if (op:equals(!objName,"door")) {
            act:open_door(door);
          } elseif (~op:equals(!objName,"open_door")) {
            act:breakBlock();
          }
        }
        act:enter_door(door);

        (!inroom) = act:filterPointsNotInCurrentRoom(!targetSet);
      } else {
        op:log("info", "  Reached end of path after approaching entryway. Should now be at target.");
        // adding target to !inroom to terminate while loop
        op:add(!inroom,!newTarget);
      }
    }
  }

  // finally, target location is in the current room, teleport to it
  if (~op:isNull(!singledist)) {
    op:log("info","  teleporting to ?targetX, ?targetY by !singledist");
    act:teleport_helper(?targetX,?targetY,!singledist);
  } else {
    op:log("info","  teleporting to ?targetX, ?targetY by ?distance");
    act:teleport_helper(?targetX,?targetY,?distance);
  }
}

() = teleport_helper(Symbol ?targetX, Symbol ?targetY, Symbol ?distance) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !newX;
  Symbol !newY;
  Symbol !newDir;
  Predicate !prevLoc;
  Predicate !prevDir;
  Symbol !x;
  Symbol !y;

  conditions : {
    pre infer : at(?actor, !currX, !currY);
    pre infer : facing(?actor, !currDir);
  }
  effects : {
     success obs : at(?actor, !newX, !newY);
     success obs : facing(?actor, !newDir);
     success obs : not(discrepancy(!x,!y));
  }
  try {
    act:teleport(?targetX,?targetY,?distance);
  } catch (FAIL_RETURNVALUE, !error) {
    op:log("debug","teleport failed with error !error");
  }

  // retract previous location and direction -- updates happen in post-condition observations
  (!prevLoc) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,!currX,!currY)");
  (!prevDir) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "facing(?actor,!currDir)");
  act:retractBelief(!prevLoc);
  act:retractBelief(!prevDir);
}

// this teleport does not do discrepancy checks, and also assumes the target
// location is in the same room as the agent
() = teleport_to_pickup(Symbol ?targetX, Symbol ?targetY, Symbol ?distance) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !newX;
  Symbol !newY;
  Symbol !newDir;
  Predicate !prevLoc;
  Predicate !prevDir;

  conditions : {
    pre infer : at(?actor, !currX, !currY);
    pre infer : facing(?actor, !currDir);
  }
  effects : {
     success obs : at(?actor, !newX, !newY);
     success obs : facing(?actor, !newDir);
  }

  act:teleport(?targetX,?targetY,?distance);

  // retract previous location and direction -- updates happen in post-condition observations
  (!prevLoc) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,!currX,!currY)");
  (!prevDir) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "facing(?actor,!currDir)");
  act:retractBelief(!prevLoc);
  act:retractBelief(!prevDir);
}

() = move(Symbol ?polycraftMoveKey) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !newLoc;
  Symbol !newX;
  Symbol !newY;
  Symbol !object;
  Predicate !query;
  Symbol !actionCost;

  conditions : {
    pre infer : at(?actor, !currX, !currY);
    pre infer : facing(?actor, !currDir);
    pre infer : fluent_equals(cost_1(smooth_move),!actionCost); // get expected action cost that will be checked in post-condition
  }
  effects : {
     success infer : not(at(?actor, !currX, !currY));
     success obs : at(?actor, !newX, !newY);
     success obs : facing(?actor, !currDir);
     success obs : fluent_equals(cost_1(smooth_move),!actionCost);
//    success infer : fluent_increase(totalcost(), cost_1(smooth_move));
  }

  (!newLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getNewLocation", !currX, !currY, !currDir, ?polycraftMoveKey);
  (!newX) = op:invokeMethod(!newLoc, "get", 1);
  (!newY) = op:invokeMethod(!newLoc, "get", 2);

  act:smoothMove(?polycraftMoveKey);
}

// move from [startX, startY] to [targetX, targetY]. NOTE: this assumes [startX, startY] != [targetX, targetY]
() = moveTo(Symbol ?targetX, Symbol ?targetY) {
  java.util.List !path;
  Symbol !stepX;
  Symbol !stepY;
  Symbol !obj;
  Symbol !currX;
  Symbol !currY;
  Symbol !startX;
  Symbol !startY;

  conditions : {
    pre infer : at(?actor, !startX, !startY);
  }
  effects : {
    success obs : at(?actor, ?targetX, ?targetY);
    success infer : not(at(?actor, !startX, !startY));
  }

  act:teleport_to(?targetX,?targetY,one);
  obs:at(?actor,!currX,!currY);

  //op:log("debug", "Finding clear path ...");
  (!path) = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
  act:findPath(!currX, !currY, ?targetX, ?targetY, !path);

  // follow path
  op:log("debug", "Found clear path. Following path ...");
  foreach (!step : !path) {
    // !step preds are in form at(obj,x,y)
    (!stepX) = op:invokeMethod(!step, "get", 1);
    (!stepY) = op:invokeMethod(!step, "get", 2);
    act:faceLocation(!stepX, !stepY);

    // first check if there's an object in the way, and if so, if it's breakable
    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !stepX, !stepY);
    if (~op:isNull(!obj) && act:isBreakable(!stepX,!stepY)) {
      op:log("info","    Block in path. Breaking object: !obj");
      // TODO: this will fail
      // TODO: this will cause a discrepancy in moveForward
      act:breakBlock();
    }

    act:moveForward();
  }

  op:log("debug", "Done following path.");
}

// this action assumes that the agent is currently one step away from the target location
// TODO: add this assumption as a pre-condition
() = faceLocation(Symbol ?targetX, Symbol ?targetY) {
  Symbol !startX;
  Symbol !startY;
  Symbol !direction;

  conditions : {
    pre infer : at(?actor, !startX, !startY);
  }

  op:log("debug", "[faceLocation] [?targetX, ?targetY] from [!startX, !startY]");

  (!direction) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getDirectionToFace", !startX, !startY, ?targetX, ?targetY);
  act:faceDirection(!direction);
}

() = faceDirection(Symbol ?direction) {
  Symbol !currDir;
  Symbol !degreesToTurn;
  Symbol !zeroDegrees;

  conditions : {
    pre infer : facing(?actor, !currDir);
  }

  op:log("debug", "[faceDirection] curr: !currDir new: ?direction");

  (!degreesToTurn) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getDegreesToTurn", !currDir, ?direction);

  (!zeroDegrees) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "0");
  if (op:notEquals(!degreesToTurn, !zeroDegrees)) {
    op:log("debug", "[faceDirection] degrees to turn: !degreesToTurn");
    act:turn(!degreesToTurn);
  } else {
    op:log("debug", "[faceDirection] no need to turn: !degreesToTurn");
  }

}
