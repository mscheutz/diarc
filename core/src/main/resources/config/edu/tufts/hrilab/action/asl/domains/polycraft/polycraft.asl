import edu.tufts.hrilab.fol.Symbol;

() = pickup(Symbol ?obj:physobj) {
    Symbol !objLoc;
    Symbol !objX;
    Symbol !objY;
    Symbol !currX;
    Symbol !currY;
    Symbol !currDir;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre obs : facing_obj(?actor, ?obj, one);
        pre infer : floating(?obj);
        pre infer : at(?actor,!currX,!currY);
        pre infer : facing(?actor,!currDir);
        pre infer : fluent_geq(world(?obj), 1);
    }
    effects : {
        success infer : fluent_decrease(world(?obj), 1);
        success obs : not(facing_obj(?actor, ?obj, one));
        success obs : facing_obj(?actor, air, one);
        success obs : fluent_increase(inventory(?actor, ?obj), 1);
        success obs : not(at(?obj, !objX, !objY));
        success obs : not(discrepancy(!x,!y));
    }
    op: log ("info",">> pickup ?obj");

    // Gets the location of ?obj (in case of more than 1, gets closest)
    // TODO: make this a pre-condition observation
    //  when more than one binding exists (will only return a random, not closest)

    !objLoc = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);

    if (op:isNull(!objLoc)) {
      exit(FAIL_PRECONDITIONS, found(?obj));
    }
    (!objX) = op:invokeMethod(!objLoc, "get", 1);
    (!objY) = op:invokeMethod(!objLoc, "get", 2);

    act: moveToEnsurePickup();
}

() = select(Symbol ?obj:physobj) {
    Symbol !actionCost;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?obj), 1);
        pre infer : holding(?actor, air);
        pre infer : fluent_equals(cost_2(select_item, ?obj), !actionCost);
    }
    effects : {
        success obs : not(holding(?actor, air));
        success obs : holding(?actor, ?obj);
        success obs : fluent_equals(cost_2(select_item, ?obj), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(select_item, ?obj));
    }
    op: log("info",">> selecting ?obj");
    act: selectItem(?obj);
}

() = deselect(Symbol ?obj:physobj) {
    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?obj), 1);
        pre infer : holding(?actor, ?obj);
    }
    effects : {
        success infer : not(holding(?actor, ?obj));
        success infer : holding(?actor, air); // TODO: make this obs once we have a mechanism to account for saplings sprouting up next to actor and causing holding(self,sapling)
    }
    op: log("info",">> deselect ?obj");
    act: selectItem(air);
}

() = break_and_pickup(Symbol ?obj:breakable) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  Symbol !actionCost;
  Symbol !x;
  Symbol !y;
  Symbol !preBreakInv;
  Symbol !postBreakInv;
  Symbol !invPred;

  conditions : {
    pre infer : not(equals(?obj, diamond_ore));
    pre infer : not(equals(?obj, block_of_platinum));
    pre obs : facing_obj(?actor, ?obj, one);
    pre infer : not(floating(?obj));
    pre infer : at(?actor,!currX,!currY);
    pre infer : facing(?actor,!currDir);
    pre infer : fluent_geq(world(?obj), 1);
    pre infer : fluent_equals(cost_2(break_block, ?obj), !actionCost); // get expected action cost that will be checked in post-condition
  }
  effects : {
    success infer : fluent_decrease(world(?obj), 1); // TODO: make this conditional on not(at(?obj,!inFrontX,!inFrontY))
    success obs : not(facing_obj(?actor, ?obj, one));
    success obs : not(at(?obj,!inFrontX,!inFrontY));
    success obs : facing_obj(?actor, air, one);
    success obs : fluent_increase(inventory(?actor, ?obj), 1);
    success obs : fluent_equals(cost_2(break_block, ?obj), !actionCost);
//    success infer : fluent_increase(totalcost(), cost_2(break_block, ?obj));
    //success obs : not(discrepancy(!x,!y));
  }
  // TODO: the "success infer : fluent_decrease(world(?obj), 1);" is problematic in the case when the act:moveToEnsurePickup
  //       fails because inferred effects aren't assumed to be true if an action fails, but in this case the effect
  //       can be true (if the obj was already picked up).

  op: log("info", ">> break_and_pickup ?obj");

  // get location of block (assumed to be inFrontOf actor)
  (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
  (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
  (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);

  obs:fluent_equals(inventory(?actor,?obj),!preBreakInv);
  act:breakBlock();
  act:nop();
  obs:fluent_equals(inventory(?actor,?obj),!postBreakInv);

  // TODO: figure out how to handle the non-determinism in the breakBlock so that we can run other agent's actions
  // breakBlock will put the object in ?actor's inventory with some probability. to ensure the ?actor picks it up,
  // check if object is in inventory (or in the area near where block was broken), if not, moveforward and moveback to pick it up
  if (op:equals(!preBreakInv, !postBreakInv)) {
    act:moveToEnsurePickup();
  } else {
    // set belief to pre-break inventory count so that the fluent_increase observation works (comparing world with belief)
    !invPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(inventory(?actor,?obj),!preBreakInv)");
    act:assertBelief(!invPred);
  }
}

() = break_and_pickup_block_of_platinum(Symbol ?obj:block_of_platinum) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  Symbol !actionCost;
  Symbol !x;
  Symbol !y;
  Symbol !preBreakInv;
  Symbol !postBreakInv;
  Symbol !invPred;

  conditions : {
    pre infer : not(floating(?obj));
    pre obs : facing_obj(?actor, ?obj, one);
    pre obs : holding(?actor, iron_pickaxe);
    pre infer : at(?actor,!currX,!currY);
    pre infer : facing(?actor,!currDir);
    pre infer : fluent_geq(world(?obj), 1);

    pre infer : fluent_equals(cost_2(break_block, ?obj), !actionCost); // get expected action cost that will be checked in post-condition
  }
  effects : {
    success infer : fluent_decrease(world(?obj), 1);
    success obs : not(facing_obj(?actor, ?obj, one));
    success obs : not(at(?obj,!inFrontX,!inFrontY));
    success obs : facing_obj(?actor, air, one);
    success obs : fluent_increase(inventory(?actor, ?obj), 1);
    success obs : fluent_equals(cost_2(break_block, ?obj), !actionCost);
//    success infer : fluent_increase(totalcost(), cost_2(break_block, ?obj));
    success obs : not(discrepancy(!x,!y));
  }

  op: log("info", ">> break_and_pickup_block_of_platinum ?obj");

  // get location of block (assumed to be inFrontOf actor)
  (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
  (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
  (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);

  obs:fluent_equals(inventory(?actor,?obj),!preBreakInv);
  act:breakBlock();
  act:nop();
  obs:fluent_equals(inventory(?actor,?obj),!postBreakInv);

  // TODO: figure out how to handle the non-determinism in the breakBlock so that we can run other agent's actions
  // breakBlock will put the object in ?actor's inventory with some probability. to ensure the ?actor picks it up,
  // check if object is in inventory (or in the area near where block was broken), if not, moveforward and moveback to pick it up
  if (op:equals(!preBreakInv, !postBreakInv)) {
    act:moveToEnsurePickup();
  } else {
    // set belief to pre-break inventory count so that the fluent_increase observation works (comparing world with belief)
    !invPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(inventory(?actor,?obj),!preBreakInv)");
    act:assertBelief(!invPred);
  }
}

() = break_and_pickup_diamond_ore(Symbol ?obj:diamond_ore) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  Symbol !actionCost;
  Symbol !x;
  Symbol !y;
  Symbol !invPred;
  int !preCount;
  int !postCount;
  int !increase;

  conditions : {
    pre obs : facing_obj(?actor, ?obj, one);
    pre infer : not(floating(?obj));
    pre infer : at(?actor,!currX,!currY);
    pre infer : facing(?actor,!currDir);
    pre infer : fluent_geq(world(?obj), 1);
    pre infer : fluent_equals(cost_2(break_block, ?obj), !actionCost); // get expected action cost that will be checked in post-condition
    pre obs : holding(?actor, iron_pickaxe);
  }
  effects : {
    success infer : fluent_decrease(world(?obj), 1);
    success obs : not(facing_obj(?actor, ?obj, one));
    success obs : not(at(?obj,!inFrontX,!inFrontY));
    success obs : facing_obj(?actor, air, one);
    success obs : fluent_increase(inventory(?actor, diamond), 9);
    success obs : fluent_equals(cost_2(break_block, ?obj), !actionCost);
//    success infer : fluent_increase(totalcost(), cost_2(break_block, ?obj));
    success obs : not(discrepancy(!x,!y));
  }

  op: log("info", ">> break_and_pickup_diamond_ore ?obj");

  // get location of block (assumed to be inFrontOf actor)
  (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
  (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
  (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);

  obs:fluent_equals(inventory(?actor,diamond),!preCount);
//  obs:fluent_equals(inventory(?actor,diamond),!preBreakInv);
  act:breakBlock();
  act:nop();
//  obs:fluent_equals(inventory(?actor,diamond),!postBreakInv);
  obs:fluent_equals(inventory(?actor,diamond),!postCount);

  // TODO: figure out how to handle the non-determinism in the breakBlock so that we can run other agent's actions
  // breakBlock will put the object in ?actor's inventory with some probability. to ensure the ?actor picks it up,
  // check if object is in inventory (or in the area near where block was broken), if not, moveforward and moveback to pick it up
  !increase = op:-(!postCount,!preCount);
  op:log("debug", "pre: !preCount post: !postCount inc: !increase");
  if (~op:equals(!increase, 9)) {
    op:log("debug", "Moving forward to get the rest");
    act:moveToEnsurePickup();
  }

  // set belief to pre-break inventory count so that the fluent_increase observation works (comparing world with belief)
  !invPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(inventory(?actor,diamond),!preCount)");
  act:assertBelief(!invPred);
}

() = place(Symbol ?obj:placeable) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  Symbol !actionCost;
  Symbol !x;
  Symbol !y;

  conditions : {
    pre infer : not(equals(?obj, tree_tap)); // use place_tree_tap
    pre infer : not(equals(?obj, sapling)); // use place_sapling
    pre obs : facing_obj(?actor, air, one);
    pre infer : fluent_geq(inventory(?actor, ?obj), 1);
    pre infer : at(?actor,!currX,!currY);
    pre infer : facing(?actor,!currDir);
    pre infer : holding(?actor,air); // to avoid issues when placing a selected item
    pre infer : fluent_equals(cost_2(place, ?obj), !actionCost);

  }
  effects : {
    success infer : fluent_increase(world(?obj), 1);
    success obs : not(facing_obj(?actor, air, one));
    success obs : at(?obj,!inFrontX,!inFrontY);
    success obs : facing_obj(?actor, ?obj, one);
    success obs : fluent_decrease(inventory(?actor, ?obj), 1);
    success obs : fluent_equals(cost_2(place, ?obj), !actionCost);
//    success infer : fluent_increase(totalcost(), cost_2(place, ?obj));
    success obs : not(discrepancy(!x,!y));
  }
  op: log("info",">> place ?obj");

  // get location in front of actor
  (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
  (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
  (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);

  act:placeItem(?obj);
}

//  placeTreeTap is a special case of place() bc a tree_tap needs to be placed in a log so lined_up log needs to be pre-condition
() = place_tree_tap(Symbol ?tree_tap:tree_tap, Symbol ?log:log) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  Symbol !actionCost;
  java.lang.String !objName;
  Symbol !obj;
  Symbol !x;
  Symbol !y;

  conditions : {
    pre obs : facing_obj(?actor, ?log, two);
    pre infer : fluent_geq(inventory(?actor, ?tree_tap), 1);
    pre infer : at(?actor,!currX,!currY);
    pre infer : facing(?actor,!currDir);
    pre infer : holding(?actor,air); // to avoid issues when placing a selected item
    pre infer : fluent_equals(cost_2(place, ?tree_tap), !actionCost);
  }
  effects : {
    success infer : fluent_increase(world(?tree_tap), 1);
    success obs : not(facing_obj(?actor, ?log, two));
    success obs : at(?tree_tap,!inFrontX,!inFrontY);
    success obs : facing_obj(?actor, ?tree_tap, one);
    success obs : fluent_decrease(inventory(?actor, ?tree_tap), 1);
    success obs : fluent_equals(cost_2(place, ?tree_tap), !actionCost);
//    success infer : fluent_increase(totalcost(), cost_2(place, ?tree_tap));
    success infer : next_to(?tree_tap, ?log); // TODO: change to obs after switch to SENSE_ACTIONS
    success obs : not(discrepancy(!x,!y));
  }

  op: log("info", ">> place_tree_tap");

  (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
  (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
  (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);

  act: placeItem("tree_tap");
}

() = place_sapling(Symbol ?sapling:sapling) {
  Symbol !currX;
  Symbol !currY;
  Symbol !currDir;
  Symbol !inFrontLoc;
  Symbol !inFrontX;
  Symbol !inFrontY;
  Symbol !placableLoc;
  Symbol !placableX;
  Symbol !placableY;
  Symbol !actionCost;
  Symbol !x;
  Symbol !y;
  Symbol !location;
  java.util.List !path;

  conditions : {
    pre obs : facing_obj(?actor, air, one);
    pre infer : fluent_geq(inventory(?actor, ?sapling), 1);
    pre infer : at(?actor,!currX,!currY);
    pre infer : facing(?actor,!currDir);
    pre infer : holding(?actor,air); // to avoid issues when placing a selected item
    pre infer : fluent_equals(cost_2(place, ?sapling), !actionCost);
  }
  effects : {
    success infer : fluent_increase(world(oak_log), 1);
    success obs : fluent_decrease(inventory(?actor, ?sapling), 1);
    success obs : not(facing_obj(?actor, air, one));
    success obs : at(oak_log,!inFrontX,!inFrontY);
    success obs : facing_obj(?actor, oak_log, one);
    success obs : fluent_equals(cost_2(place, ?sapling), !actionCost);
//    success infer : fluent_increase(totalcost(), cost_2(place, ?sapling));
    success obs : not(discrepancy(!x,!y));
  }
  op: log("info",">> place_sapling ?sapling");

  // get location in front of actor
  (!inFrontLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement", "getInFront", !currX, !currY, !currDir);
  (!inFrontX) = op:invokeMethod(!inFrontLoc, "get", 1);
  (!inFrontY) = op:invokeMethod(!inFrontLoc, "get", 2);
  op:log("debug", "  facing (!inFrontX,!inFrontY)");

  // ensure inFrontLoc is valid place to plant sapling (i.e. neighbors are all empty)
  // TODO: getClosestAllEmptyAdjacent needs to check diagonal neighbors (e.g., when standing in an entryway)
  (!placableLoc) = op:invokeStaticMethod("edu.tufts.hrilab.polycraft.util.Movement","getClosestAllEmptyAdjacent",!inFrontX,!inFrontY);
  (!placableX) = op:invokeMethod(!placableLoc, "get", 1);
  (!placableY) = op:invokeMethod(!placableLoc, "get", 2);
  if (op:notEquals(!placableX,!inFrontX) || op:notEquals(!placableY,!inFrontY)) {
    op: log("debug", "  moving to face (!placableX,!placableY)");
    // TODO: moveTo breaks here
//    act:moveTo(!currX,!currY,!placableX,!placableY);
//    act:moveBack();
    act:teleport_to(!placableX,!placableY,one);

    // update inFrontLoc for post-condition checks
    (!inFrontX) = op:newObject("edu.tufts.hrilab.fol.Symbol", !placableX);
    (!inFrontY) = op:newObject("edu.tufts.hrilab.fol.Symbol", !placableY);
  }

  act:placeItem(?sapling);
}

() = craft_plank(Symbol ?log:log) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?log), 1);
        pre infer : fluent_equals(cost_2(craft, oak_plank), !actionCost);
    }
    effects : {
        success obs : fluent_increase(inventory(?actor, oak_plank), 4);
        success obs : fluent_decrease(inventory(?actor, ?log), 1);
        success obs : fluent_equals(cost_2(craft, oak_plank), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(craft, oak_plank));
        success obs : not(discrepancy(!x,!y));
    }
    op: log("info", ">> crafting oak_plank");
    act: craft("recipe(log,0,0,0,0,0,0,0,0,plank,4)");
}

() = craft_stick(Symbol ?plank:plank) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?plank), 2);
        pre infer : fluent_equals(cost_2(craft, stick), !actionCost);

    }
    effects : {
        success obs : fluent_increase(inventory(?actor, stick), 4);
        success obs : fluent_decrease(inventory(?actor, ?plank), 2);
        success obs : fluent_equals(cost_2(craft, stick), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(craft, stick));
        success obs : not(discrepancy(!x,!y));
    }
    op: log("info", ">> crafting stick");
    act: craft("recipe(plank,0,0,plank,0,0,0,0,0,stick,4)");
}

() = craft_tree_tap(Symbol ?plank:plank, Symbol ?stick:stick) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?plank), 5);
        pre infer : fluent_geq(inventory(?actor, ?stick), 1);
        pre infer : fluent_geq(world(crafting_table), 1);
        pre obs : facing_obj(?actor, crafting_table, one);
        pre infer : fluent_equals(cost_2(craft, tree_tap), !actionCost);
    }
    effects : {
        success obs : fluent_increase(inventory(?actor, tree_tap), 1);
        success obs : fluent_decrease(inventory(?actor, ?plank), 5);
        success obs : fluent_decrease(inventory(?actor, ?stick), 1);
        success obs : fluent_equals(cost_2(craft, tree_tap), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(craft, tree_tap));
        success obs : not(discrepancy(!x,!y));
    }
    op: log("info", ">> crafting tree_tap");
    act: craft("recipe(plank,stick,plank,plank,0,plank,0,plank,0,tree_tap,1)");
}

() = craft_block_of_diamond(Symbol ?diamond:diamond) {
    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?diamond), 9);
        pre infer : fluent_geq(world(crafting_table), 1);
        pre obs : facing_obj(?actor, crafting_table, one);
        pre infer : fluent_equals(cost_2(craft, block_of_diamond), !actionCost);
    }
    effects : {
        success obs : fluent_increase(inventory(?actor, block_of_diamond), 1);
        success obs : fluent_decrease(inventory(?actor, ?diamond), 9);
        success obs : fluent_equals(cost_2(craft, block_of_diamond), !actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(craft, block_of_diamond));
        success obs : not(discrepancy(!x,!y));
    }
    op: log("info", ">> crafting block_of_diamond");
    act: craft("recipe(diamond,diamond,diamond,diamond,diamond,diamond,diamond,diamond,diamond,block_of_diamond,1)");
}

() = craft_pogo_stick(Symbol ?stick:stick, Symbol ?rubber:rubber,
                      Symbol ?block_of_titanium:block_of_titanium, Symbol ?block_of_diamond:block_of_diamond) {

    Symbol !actionCost;
    Symbol !x;
    Symbol !y;

    conditions : {
        pre infer : fluent_geq(inventory(?actor, ?stick), 2);
        pre infer : fluent_geq(inventory(?actor, ?rubber), 1);
        pre infer : fluent_geq(inventory(?actor, ?block_of_titanium), 2);
        pre infer : fluent_geq(inventory(?actor, ?block_of_diamond), 2);
        pre infer : fluent_geq(world(crafting_table), 1);
        pre obs : facing_obj(?actor, crafting_table, one);
        pre infer : fluent_equals(cost_2(craft, pogo_stick),!actionCost);
    }
    effects : {
        success obs : fluent_increase(inventory(?actor, pogo_stick), 1);
        success obs : fluent_decrease(inventory(?actor, ?stick), 2);
        success obs : fluent_decrease(inventory(?actor, ?rubber), 1);
        success obs : fluent_decrease(inventory(?actor, ?block_of_titanium), 2);
        success obs : fluent_decrease(inventory(?actor, ?block_of_diamond), 2);
        success obs : fluent_equals(cost_2(craft, pogo_stick),!actionCost);
//        success infer : fluent_increase(totalcost(), cost_2(craft, pogo_stick));
        success obs : not(discrepancy(!x,!y));
    }
    op: log("info", ">> crafting pogo stick");
    act: craft("recipe(stick,block_of_titanium,stick,block_of_diamond,block_of_titanium,block_of_diamond,0,rubber,0,pogo_stick,1)");
}
