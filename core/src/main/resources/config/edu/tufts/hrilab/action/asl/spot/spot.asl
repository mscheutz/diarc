() = dock() {
   op:log("info", "Attempting to dock...");
   try {
     act:dock(520);
   } catch () {
     act:dock(521);
   }
}

() = move["?actor moves ?direction"](edu.tufts.hrilab.fol.Symbol ?direction) {
    edu.tufts.hrilab.fol.Predicate !failCond;

    try {
      if (op:equalsValue(?direction, straight) || op:equalsValue(?direction, forward)) {
        while(true) {
          act:setTV(0.25);
        }
      } elseif (op:equalsValue(?direction, back)) {
        while(true) {
          act:setTV(-0.25);
        }
      } else {
        !failCond = op:createPredicate("not(know(?actor,meaningOf(moving(?direction))))");
        exit(FAIL, !failCond);
      }
    } finally {
      op:log("debug", "[move(?actor, ?direction)] in finally block");
      act:stop();
    }
}

() = turn(edu.tufts.hrilab.fol.Symbol ?direction, edu.tufts.hrilab.fol.Symbol ?theta = infinite) {
    edu.tufts.hrilab.fol.Predicate !failCond;
    java.lang.Float !degToRad = 0.01745; // pi/180
    java.long.Float !rads;
    java.long.Float !radPerSec = 0.3;
    try {
      // determine direction
      if (op:equalsValue(?direction, right)) {
        !degToRad = op:*(!degToRad, -1);
        !radPerSec = op:*(!radPerSec, -1);
      }

      // determine if it is turning forever or not
      if (op:equalsValue(?theta, infinite)) {
        // setVels only sets vels for a limited time horizon
        while (true) {
          act:setRV(!radPerSec);
        }
      } else {
        // determine angle
        if (op:equalsValue(?theta, ninety)) {
          !rad = op:*(!degToRad, 90);
        } elseif (op:equalsValue(?theta, oneeighty)) {
          !rad = op:*(!degToRad, 180);
        } elseif (op:equalsValue(?theta, twoseventy)) {
          !rad = op:*(!degToRad, 270);
        } else {
          op:log("error: unsupported theta value in turn action: ?theta");
          !failCond = op:createPredicate("not(know(?actor,meaningOf(turning(?direction,?theta))))");
          exit(FAIL, !failCond);
        }
        act:moveToBlocking(0.0, 0.0, !rad);
      }
    } finally {
      op:log("debug", "[turn(?actor, ?direction)] in finally block");
      act:stop();
    }
}