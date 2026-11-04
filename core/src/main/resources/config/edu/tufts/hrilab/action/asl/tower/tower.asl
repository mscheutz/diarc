import edu.tufts.hrilab.fol.Symbol;

// TODO: where to retract see(?actor,?physobj)

() = go_to_location(Symbol ?dstLoc:location) {
  Symbol ?currLoc:location;

  conditions : {
    pre infer : not(on_elevator(?actor));
    pre: at(?actor,?currLoc);
    pre: fluent_equals(on_floor(?actor),on_floor(?dstLoc));
  }
  effects : {
    success: not(at(?actor,?currLoc));
    success: at(?actor,?dstLoc);
  }

  op:log("info", ">> go_to_location ?dstLoc from ?currLoc");
}

() = find_elevator_outside_button(Symbol ?btn:physobj) {
  Symbol ?currLoc:location;

  conditions : {
    or : {
      pre : property_of(?btn, button_up);
      pre : property_of(?btn, button_down);
    }
    pre : at(?actor, ?currLoc);
    pre : property_of(?currLoc, elevator);
    pre : not(on_elevator(?actor));
  }
  effects : {
    success : see(?actor, ?btn);
  }

  op:log("info", ">> find_elevator_outside_button");
}

() = find_elevator_inside_button(Symbol ?btn:physobj) {
  conditions : {
    pre : on_elevator(?actor);
    pre : not(property_of(?btn, button_up));
    pre : not(property_of(?btn, button_down));
  }
  effects : {
    success : see(?actor, ?btn);
  }

  op:log("info", ">> find_elevator_inside_button");
}

() = press_elevator_button_down(Symbol ?btn:physobj) {
  Symbol ?currLoc:location;
  Symbol ?dstLoc:location;

  conditions : {
    pre : see(?actor, ?btn);
    pre : goal(?actor, ?dstLoc); // is this valid?
    pre : property_of(?btn, button_down);
    pre : at(?actor, ?currLoc);
    pre : property_of(?currLoc, elevator);
    pre : fluent_greater(on_floor(?currLoc),on_floor(?dstLoc));
  }
  effects : {
    success : pressed(?btn);
  }

  op:log("info", ">> press_elevator_button_down");
}

() = press_elevator_button_up(Symbol ?btn:physobj) {
  Symbol ?currLoc:location;
  Symbol ?dstLoc:location;

  conditions : {
    pre : see(?actor, ?btn);
    pre : goal(?actor, ?dstLoc); // is this valid?
    pre : property_of(?btn, button_up);
    pre : at(?actor, ?currLoc);
    pre : property_of(?currLoc, elevator);
    pre : fluent_less(on_floor(?currLoc),on_floor(?dstLoc));
  }
  effects : {
    success : pressed(?btn);
  }

  op:log("info", ">> press_elevator_button_up");
}

() = wait_for_elevator() {
  Symbol ?btn:physobj;
  Symbol ?currLoc:location;

  conditions : {
    pre : pressed(?btn);
    or : {
      pre : property_of(?btn, button_up);
      pre : property_of(?btn, button_down);
    }
    pre : at(?actor, ?currLoc);
    pre : property_of(?currLoc, elevator);
  }
  effects : {
    success : not(pressed(?btn));
    success : property_of(door, open); // TODO: how to represent door (ref?) and open?
  }

  op:log("info", ">> wait_for_elevator");
}

() = enter_elevator() {
  Symbol ?currLoc:location;

  conditions : {
    pre : at(?actor, ?currLoc);
    pre : property_of(?currLoc, elevator);
    pre : not(on_elevator(?actor));
    pre : property_of(door, open);
  }
  effects : {
    success: on_elevator(?actor);
    success: not(at(?actor,?currLoc));
  }

  op:log("info", ">> enter_elevator");
}

() = press_elevator_button_floor(Symbol ?btn:physobj) {
  Symbol ?dstLoc:location;

  conditions : {
    pre : see(?actor, ?btn);
    pre : on_elevator(?actor);
    pre : goal(?actor, ?dstLoc); // is this valid?
    pre : fluent_equals(on_floor(?dstLoc),target_floor(?btn));
  }
  effects : {
    success : pressed(?btn);
  }

  op:log("info", ">> press_elevator_button_floor: ?btn");
}

() = wait_for_elevator_floor() {
  Symbol ?btn:physobj;

  conditions : {
    pre : on_elevator(?actor);
    pre : pressed(?btn);
  }
  effects : {
    success : assign(on_floor(?actor),target_floor(?btn));
    success : not(pressed(?btn));
    success : property_of(door, open);
  }

  op:log("info", ">> wait_for_elevator_floor");
}

() = exit_elevator(Symbol ?exit_loc:location){
  Symbol ?dstLoc:location;

  conditions : {
    pre : property_of(?exit_loc, elevator);
    pre : on_elevator(?actor);
    pre : goal(?actor, ?dstLoc); // is this valid?
    pre : fluent_equals(on_floor(?dstLoc),on_floor(?actor));
    pre : property_of(door, open);
  }
  effects : {
    success : not(on_elevator(?actor));
    success : at(?actor, ?exit_loc);
  }

  op:log("info", ">> exitElevator");
}
