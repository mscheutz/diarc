import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

() = use_safe(Symbol ?safe:safe) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions: {
        pre obs : facing_obj(?actor,?safe,one);
        pre obs : holding(?actor,key);
        pre infer: fluent_equals(cost_2(use,?safe),!actionCost);
    }
    effects : {
        success infer: propertyOf(?safe, unlocked);
        success obs : fluent_equals(cost_2(use,?safe),!actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(use,?safe));
        success obs : not(discrepancy(!x,!y));
    }

    op:log("info" ,">> use ?safe");
    act:use();
}

() = open_door(Symbol ?door:door) {
    Symbol !actionCost;
    Symbol !currX;
    Symbol !currY;
    Symbol !currDir;
    Symbol !inFrontLoc;
    Symbol !inFrontX;
    Symbol !inFrontY;
    Symbol !x;
    Symbol !y;
    Predicate !accessedPred;

    conditions: {
        pre infer : not(equals(?door, open_door));
        pre infer: fluent_equals(cost_2(use,?door),!actionCost);
        pre infer: at(?actor,!currX,!currY);
        pre infer: facing(?actor,!currDir);
        pre obs : facing_obj(?actor,?door,one);
        pre obs : holding(?actor,air);
    }
    effects : {
        success infer: fluent_decrease(world(?door),1);
        success infer : fluent_increase(world(open_door),1);
        success obs : not(facing_obj(?actor,?door,one));
        success obs : facing_obj(?actor,air,one);
        success obs : not(at(?door,!inFrontX,!inFrontY));
        success obs : at(open_door,!inFrontX,!inFrontY);
        success obs : fluent_equals(cost_2(use,?door),!actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(use,?door));
//        success obs : not(discrepancy(!x,!y)); // commented out so that find_safe can't fail bc of pogoist discrepancy
    }
    op:log ("info",">> open ?door");

    // get location of door (assumed to be inFrontOf actor)
    (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
    (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
    (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);

    act:use(); // open door

    // step into open doorway (without doing discrepancy check) so that we're in same room as the open door
    // so we can sense the at(open_door,!x,!y) post-condition
    act:move("W");

    (!accessedPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "accessed(!inFrontX,!inFrontY)");
    act:assertBelief(!accessedPred);
}

() = enter_door(Symbol ?door:door) {
//    Symbol !x;
//    Symbol !y;
    Symbol !currX;
    Symbol !currY;
    Symbol !currDir;
    Symbol !inFrontLoc;
    Symbol !inFrontX;
    Symbol !inFrontY;
    Predicate !accessedPred;

    conditions : {
        pre infer : at(self,!currX,!currY);
        pre infer: facing(?actor,!currDir);
    }

    effects : {
//        success obs : not(discrepancy(!x,!y));  // commented out so that find_safe can't fail bc of pogoist discrepancy
    }

    op:log ("info",">> enter ?door");

    // step into room (without doing discrepancy check)
    if (~act:move("W")) {
      op:log ("info","can't see in front of me, so blindly trying to open door...");
      act:use(); // open door
      act:move("W"); // step into room
    }

    // update map with new room info
    act:senseNewMap();

    (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
    (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
    (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);
    (!accessedPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "accessed(!inFrontX,!inFrontY)");
    act:assertBelief(!accessedPred);
}

() = delete_item(Symbol ?obj:physobj) {
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?obj), 1);
    }
    effects : {
        success obs : fluent_decrease(inventory(?actor, ?obj), 1);
        success obs : not(discrepancy(!x,!y));
    }
    op: log("info",">> deleting ?obj");
    act: delete(?obj);
}

() = interact_with(Symbol ?otherActor:agent) {
    Symbol !x;
    Symbol !y;

    conditions : {
      pre obs : facing_obj(?actor,?otherActor,one);
    }
    effects : {
      success obs : not(discrepancy(!x,!y));
    }
    op: log("info",">> interacting with ?otherActor");
    act: interact(?otherActor, false);
}

() = get_trades_from(Symbol ?otherActor:agent) {
    Symbol !x;
    Symbol !y;

    conditions : {
      pre obs : facing_obj(?actor,?otherActor,one);
    }
    effects : {
      success obs : not(discrepancy(!x,!y));
    }
    op: log("info",">> get_trade_from ?otherActor");
    act: interact(?otherActor, true);
}

() = collect_from_tree_tap(Symbol ?tree_tap:tree_tap, Symbol ?log:log) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre obs : next_to(?tree_tap, ?log);
        pre obs : facing_obj(?actor, ?tree_tap, one);
        pre infer : fluent_geq(world(?log), 1); // TODO: this type inference is problematic
        pre infer : fluent_geq(world(?tree_tap), 1); // TODO: this type inference is problematic
        pre infer : fluent_equals(cost_2(collect, ?tree_tap), !actionCost);
    }
    effects : {
        success obs : fluent_increase(inventory(?actor, rubber), 1);
        success obs : fluent_equals(cost_2(collect, ?tree_tap), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(collect, ?tree_tap));
        success obs : not(discrepancy(!x,!y));
    }

    op: log("info",">> collecting from ?tree_tap");
    act: collect();
}

() = collect_from_chest(Symbol ?chest:chest) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre obs : facing_obj(?actor, ?chest, one);
        pre obs : holding(?actor,air); // so that we don't need to have not(holding(?actor,?physobj02)) as post-condition
        pre infer : fluent_equals(cost_2(collect, ?chest), !actionCost);
        pre infer : fluent_equals(container(chest, key), 1);
    }
    effects : {
        success infer : fluent_decrease(container(?chest, key), 1);
        success obs : fluent_increase(inventory(?actor, key), 1);
        success obs : not(holding(?actor, air));
        success obs : holding(?actor, key);
        success obs : fluent_equals(cost_2(collect, ?chest), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(collect, ?chest));
        success obs : not(discrepancy(!x,!y));
    }

    op: log("info",">> collecting from ?chest");
    act: collect();
}

() = collect_from_safe(Symbol ?safe:safe) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre obs : facing_obj(?actor, ?safe, one);
        pre infer : propertyOf(?safe, unlocked);
        pre infer : fluent_equals(cost_2(collect, ?safe), !actionCost);
        pre infer : fluent_equals(container(?safe, diamond), 18);
    }
    effects : {
        success infer : fluent_decrease(container(?safe, diamond), 18);
        success obs : fluent_increase(inventory(?actor, diamond), 18);
        success obs : fluent_equals(cost_2(collect, ?safe), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(collect, ?safe));
        success obs : not(discrepancy(!x,!y));
    }

    op: log("info",">> collecting from ?safe");
    act: collect();
}

// generic collect action that can be used in exploration
() = collect_from(Symbol ?obj:physobj) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : not(equals(?obj, chest));
        pre infer : not(equals(?obj, safe));
        pre infer : not(equals(?obj, tree_tap));
        pre obs : facing_obj(?actor, ?obj, one);
        pre infer : fluent_equals(cost_2(collect, ?obj), !actionCost);
    }
    effects : {
        success obs : fluent_equals(cost_2(collect, ?obj), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(collect, ?obj));
        success obs : not(discrepancy(!x,!y));
    }

    op: log("info",">> collecting from ?obj");
    act: collect();
}

// generic collect action that can be used in exploration
() = use(Symbol ?obj:physobj) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : not(equals(?obj, door));
        pre infer : not(equals(?obj, safe));
        pre obs : facing_obj(?actor, ?obj, one);
        pre infer : fluent_equals(cost_2(use, ?obj), !actionCost);
    }
    effects : {
        success obs : fluent_equals(cost_2(use, ?obj), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(use, ?obj));
        success obs : not(discrepancy(!x,!y));
    }

    op: log("info",">> collecting from ?obj");
    act: use();
}
() = explore_rooms() {
  Predicate !closedDoorPred;
  Predicate !unchartedPred;
  java.util.List !uncharted;
  java.util.Map !map;
  edu.tufts.hrilab.fol.Variable !varX;
  edu.tufts.hrilab.fol.Variable !varY;
  Symbol !xloc;
  Symbol !yloc;

  op:log("info",">> exploring rooms...");

  // while haven't explored all entryways (i.e., uncharted(x,y)), enter room, and look inside
  !closedDoorPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(world(door),0)");
  !unchartedPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "uncharted(X,Y)");
  op:log(info, !uncharted);
  !uncharted = op:newObject("java.util.ArrayList");
  op:log(info, !uncharted);
  !uncharted = act:queryBelief(!unchartedPred);
  op:log(info, !uncharted);
  while (~op:isEmpty(!uncharted)) {
    if (~act:querySupport(!closedDoorPred)) {
      goal:open_door(door);
    } else {
      op:log("debug","Finding entryway");
      !map = op:invokeMethod(!uncharted,"get",0);
      !varX = op:newObject("edu.tufts.hrilab.fol.Variable","X");
      !varY = op:newObject("edu.tufts.hrilab.fol.Variable","Y");
      !xloc = op:invokeMethod(!map,"get",!varX);
      !yloc = op:invokeMethod(!map,"get",!varY);
      op:log("debug","Entryway at !xloc,!yloc");
      act:exploration_helper(!xloc,!yloc);
    }
    goal:enter_door(door);
    !uncharted = act:queryBelief(!unchartedPred);
  }

  op:log("info",">> ...done exploring.");
}

// get a sapling in the inventory
() = get_sapling() {
    Symbol !currX;
    Symbol !currY;
    Symbol !objX;
    Symbol !objY;
    Symbol !sapling;
    // TODO: bug when (?facing_obj,?facing_dist) == (air,one) if both post-cond facing_obj are observations
    Symbol ?facing_obj:physical;
    Symbol ?facing_dist:distance;

    conditions : {
        pre infer : fluent_equals(world(oak_log), 0); // TODO: this type inference is problematic
        pre infer : fluent_equals(inventory(?actor,sapling), 0);
        pre infer: at(self,!currX,!currY);
        pre infer: facing_obj(?actor,?facing_obj,?facing_dist);
    }
    effects : {
        success infer : not(facing_obj(?actor,?facing_obj,?facing_dist));
        success infer : facing_obj(?actor,air,one);
        success infer : not(at(sapling,!objX,!objY));
        success obs : fluent_increase(inventory(?actor, sapling), 1);
    }

    op: log("info",">> getting a sapling");

    if (~obs:at(sapling,!objX,!objY)) {
      // if no sapling was observed, perform nop action(s) to advance time to give saplings a chance to
      // sprout in the world before trying to call get_sapling again
      act:nop();
      act:nop();
      exit(FAIL_PRECONDITIONS, not(at(sapling,!objX,!objY)));
    }

    // move to it to pick it up
    act:teleport_to_pickup(!objX, !objY, one);
}

// move from room to room until a safe is found
() = find_safe() {
    Predicate !closedDoorPred;
    Predicate !foundSafePred;
    java.util.List !uncharted;
    java.util.Map !map;
    Predicate !unchartedPred;
    edu.tufts.hrilab.fol.Variable !varX;
    edu.tufts.hrilab.fol.Variable !varY;
    Symbol !xloc;
    Symbol !yloc;
    // TODO: bug when (?facing_obj,?facing_dist) == (air,one) if both post-cond facing_obj are observations
    Symbol ?facing_obj:physical;
    Symbol ?facing_dist:distance;

    conditions : {
        pre infer : equals(?actor,self); // no ?actor in post-condition, so force planner to use self as ?actor
        pre infer : fluent_equals(world(safe), 0);
        pre infer: facing_obj(?actor,?facing_obj,?facing_dist);
        pre obs : holding(?actor,air); // so that we don't need to have not(holding(?actor,?physobj02)) as post-condition
        pre obs : fluent_geq(inventory(?actor,key),1);
    }
    effects : {
        success infer : fluent_increase(world(safe), 1);
        success infer : not(facing_obj(?actor,?facing_obj,?facing_dist));
        success infer : facing_obj(?actor,air,one); // bc of enter_door sub-action
        success obs : holding(?actor,air); // bc of open_door sub-action
    }

    op: log("info",">> finding a safe");

    // while haven't found a safe and haven't opened all closed doors, enter room, and look inside
    (!closedDoorPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(world(door),0)");
    (!foundSafePred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_geq(world(safe),1)");

    //get entryways
    !unchartedPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "uncharted(X,Y)");
    op:log(info, !uncharted);
    !uncharted = op:newObject("java.util.ArrayList");
    op:log(info, !uncharted);
    !uncharted = act:queryBelief(!unchartedPred);
    op:log(info, !uncharted);
    while (~op:isEmpty(!uncharted) && ~act:querySupport(!foundSafePred)) {
      if (~act:querySupport(!closedDoorPred)) {
        goal:open_door(door);
      } else {
        op:log("debug","Finding entryway");
        (!map) = op:invokeMethod(!uncharted,"get",0);
        (!varX) = op:newObject("edu.tufts.hrilab.fol.Variable","X");
        (!varY) = op:newObject("edu.tufts.hrilab.fol.Variable","Y");
        (!xloc) = op:invokeMethod(!map,"get",!varX);
        (!yloc) = op:invokeMethod(!map,"get",!varY);
        op:log("debug","Entryway at !xloc,!yloc");
        act:exploration_helper(!xloc,!yloc);
      }
      goal:enter_door(door);
      (!uncharted) = act:queryBelief(!unchartedPred);
    }
  
    // do an explicit check here instead of post-condition observation so that recovery
    // won't change the post-condition to fluent_increase(world(safe),0) which would break find_safe
    // for all future games in the same tournament
    if (~act:querySupport(!foundSafePred)) {
      op:log("warn", "No safe found and no more rooms to explore.");
      exit(FAIL_POSTCONDITIONS, "fluent_geq(world(safe),1)");
    }
}

() = exploration_helper(Symbol ?xloc, Symbol ?yloc){
  Symbol !currX;
  Symbol !currY;
  Predicate !accessedPred;

  conditions : {
      pre obs: at(self,!currX,!currY);
  }

  if (op:notEquals(!currX,?xloc) || op:notEquals(!currY,?yloc)) {
    act:moveTo(?xloc,?yloc);
    (!accessedPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "accessed(?xloc,?yloc)");
    act:assertBelief(!accessedPred);
  }
}
