import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

// Approach ?actor01, trying to get ?distance away, and face ?actor01
// ?actor is assumed to currently be facing ?physical02, at a distance of ?distance02
() = approach_actor(Symbol ?actor01, Symbol ?distance01, Symbol ?physical02, Symbol ?distance02) {
  Symbol !x;
  Symbol !y;

  conditions : {
    pre infer : fluent_geq(world(?actor01), 1);
    pre infer : facing_obj(?actor, ?physical02, ?distance02);
  }
  effects : {
    success infer : facing_obj(?actor, ?actor01, ?distance01);
    success infer : not(facing_obj(?actor, ?physical02, ?distance02));
    success obs : not(discrepancy(!x,!y));
  }

  op:log("info", " >> approach_actor ?actor01");

  act:approach_actor_helper(?actor01, ?distance01);
}

() = approach_actor_helper(Symbol ?actor01, Symbol ?distance01) {
  Symbol !beliefX;
  Symbol !beliefY;
  Predicate !retraction;
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

  op:log("info", "[approach_actor_helper] ?actor01.");

  // retract (potentially) old ?actor01 location from belief
  (!retraction) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor01, X, Y)");
  act:retractBelief(!retraction);

  // observe new ?actor01 location (which also assert it to belief)
  obs:at(?actor01, !targetX, !targetY);
  op:log("debug", "[approach_actor_helper] at(?actor01, !targetX, !targetY).");

  (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?actor01, ?distance01);
  while (op:isNull(!path) && op:lt(!count, 5)) {
    (!count) = op:++(!count);
    act:nop();
    act:nop();

    (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?actor01, ?distance01);
  }

  if (op:isNull(!path)) {
    exit(FAIL_PRECONDITIONS, found(pathTo, ?actor01));
  }

  // remove the last element of the path (don't want to move into same location as target object)
  (!size) = op:invokeMethod(!path, "size");
  op:--(!size,!size);
  (!targetLoc)  = op:invokeMethod(!path, "remove", !size);

  foreach (!step : !path) {
    // if the targetLocation changes, then re-plan
    if(~obs:at(?actor01, !targetX, !targetY)) {
      op:log("info", "[approach_actor_helper] ?actor01 moved...");
      act:approach_actor_helper(?actor01, ?distance01);
      op:log("info", "[approach_actor_helper] returning from at(?actor01, !targetX, !targetY)");
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
  op:log("info", "[approach_actor_helper] Done following path.");
}

// Approach ?physobj01, trying to get ?distance away, and face ?physobj01
// ?actor is assumed to currently be facing ?physical02, at a distance of ?distance02
() = approach_object(Symbol ?physobj01, Symbol ?distance01, Symbol ?physical02, Symbol ?distance02) {
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

  conditions : {
    pre infer : fluent_geq(world(?physobj01), 1);
    // don't really care what these values are in the real world (i.e., don't obs) -- only here for planner
    pre infer : facing_obj(?actor, ?physical02, ?distance02);
  }
  effects : {
    success infer : not(facing_obj(?actor, ?physical02, ?distance02));
    success infer : facing_obj(?actor, ?physobj01, ?distance01); // not obs so that post-cond recovery mechanism isn't used
    success obs : not(discrepancy(!x,!y));
  }

  op:log("info", " >> approach_object ?physobj01");

  // Gets the location of ?physobj01 (in case of more than 1, gets closest)
  // TODO: make this a pre-condition observation ?
  //  when more than one binding exists (will only return a random, not closest)
  (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?physobj01, ?distance01);
  while (op:isNull(!path) && op:lt(!count, 5)) {
    (!count) = op:++(!count);
    act:nop();
    act:nop();

    (!path) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getPathToObject", ?physobj01, ?distance01);
  }

  if (op:isNull(!path)) {
    exit(FAIL_PRECONDITIONS, found(pathTo, ?physobj01));
  }

  // get ?physobj01 location as first element in path
  (!size) = op:invokeMethod(!path, "size");
  op:--(!size,!size);
  (!objLoc) = op:invokeMethod(!path, "remove", !size);
  (!objX) = op:invokeMethod(!objLoc, "get", 1);
  (!objY) = op:invokeMethod(!objLoc, "get", 2);

  // follow path
  op:log("debug", "Found path. Following path ...");
  foreach (!step : !path) {
    // !step preds are in form at(obj,x,y)
    (!stepX) = op:invokeMethod(!step, "get", 1);
    (!stepY) = op:invokeMethod(!step, "get", 2);
    act:faceLocation(!stepX, !stepY);

    // TODO: this is problematic bc we're still breaking objects which might break the plan later on
    // first check if there's an object in the way, and if so, if it's breakable
    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !stepX, !stepY);
    if (~op:isNull(!obj) && act:isBreakable(!stepX,!stepY)) {
      op:log(info,"Block in path. Breaking object: !obj");
      // perform primitive break bc we don't know which break_and_pickup version to call (object dependent)
      // moveForward below will handle updating inventory when picking up objects or location discrepancies
      act:breakBlock();
    }

    act:moveForward();
  }

  act:faceLocation(!objX,!objY);

  // if ?distance01 is two, and there's an object one space away, break the object directly in front of ?actor
  Symbol !currDir;
  Symbol !currX;
  Symbol !currY;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  java.lang.String !distStr;
  (!distStr) = op:invokeMethod(?distance01, "getName");
  if (op:equals(!distStr, "two")) {
    obs:facing(?actor, !currDir); // get current ?actor direction
    obs:at(?actor, !currX, !currY); // get current ?actor location
    (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
    (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
    (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);
    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement","getObjectAt",!inFrontX,!inFrontY);
    if (~op:isNull(!obj)) {
      op:log(info,"Block in between ?actor and ?physobj01. Breaking object: !obj");
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
  Symbol !x;
  Symbol !y;

  conditions : {
    pre infer : facing(?actor, !currDir);
    pre infer : actionCost(smooth_turn,!actionCost); // get expected action cost that will be checked in post-condition
  }
  effects : {
    success infer : not(facing(?actor, !currDir));
    success obs : facing(?actor, !newDir);
    success obs : actionCost(smooth_turn,!actionCost);
    success obs : not(discrepancy(!x,!y));
  }

  op:log("debug", "[turn] degrees: ?degrees currDir: !currDir");
  act:smoothTurn(?degrees);
  (!newDir) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getNextDirection", !currDir, ?degrees);
  op:log("debug", "[turnRight] newDir: !newDir");
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
    pre infer : actionCost(smooth_move,!actionCost); // get expected action cost that will be checked in post-condition
  }
  effects : {
     success infer : not(at(?actor, !currX, !currY));
     success obs : at(?actor, !newX, !newY);
     success obs : facing(?actor, !currDir);
     success obs : actionCost(smooth_move,!actionCost);
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
  Symbol !startX;
  Symbol !startY;

  conditions : {
    pre infer : at(?actor, !startX, !startY);
  }
  effects : {
    success obs : at(?actor, ?targetX, ?targetY);
    success infer : not(at(?actor, !startX, !startY));
  }

  op:log("debug", "Finding clear path ...");
  (!path) = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
  act:findPath(!startX, !startY, ?targetX, ?targetY, !path);

  // follow path
  op:log("debug", "Found clear path. Following path ...");
  foreach (!step : !path) {
    // !step preds are in form at(actor,x,y)
    (!stepX) = op:invokeMethod(!step, "get", 1);
    (!stepY) = op:invokeMethod(!step, "get", 2);
    act:faceLocation(!stepX, !stepY);

    // first check if there's an object in the way, and if so, if it's breakable
    (!obj) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getObjectAt", !stepX, !stepY);
    if (~op:isNull(!obj) && act:isBreakable(!stepX,!stepY)) {
      op:log(info,"Block in path. Breaking object: !obj");
      act:breakBlock();
    }

    act:moveForward();
  }

  op:log("debug", "Done following path.");
}

// this action assumes that the agent is currently one step away from the target location
// TODO: add this assumption as a pre-condition
() = faceLocation(Symbol ?targetX, Symbol ?targetY) {
  Symbol !currX;
  Symbol !currY;
  Symbol !direction;

  conditions : {
    pre infer : at(?actor, !currX, !currY);
  }

  op:log("debug", "[faceLocation] [?targetX, ?targetY] from [!currX, !currY]");

  (!direction) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getDirectionToFace", !currX, !currY, ?targetX, ?targetY);
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
