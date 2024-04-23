() = moveForward["Robot moves forward"]() {
    edu.tufts.hrilab.fol.Predicate !failCond;
    edu.tufts.hrilab.fol.Variable !x;
    conditions : {
        or : {
            pre obs : not(see(?actor,obstacle));
            pre : bel(?actor,not(propertyOf(obstacle,solid)));
        }
        or : {
            pre obs : see(?actor,support);
            pre : bel(?actor,will(catch(!x,?actor)));
        }
        or : {
            overall obs : not(see(?actor,obstacle));
            overall : bel(?actor,not(propertyOf(obstacle,solid)));
        }
        or : {
            overall obs : see(?actor,support);
            overall : bel(?actor,will(catch(!x,?actor)));
        }
    }

    locks : motionLock;

    act:setTV(0.25);

    op:log("debug", "?actor is moving forward...");
    op:sleep(300);
    if (~act:isMoving()) {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(propertyOf(?actor,standing))");
      exit(FAIL, !failCond);
    }

    // moveTo is non-blocking. Loop here to keep checking overall conditions
    while (act:isMoving()) {
      op:sleep(100);
    }
}

(java.util.List ?return) = checkFor(edu.tufts.hrilab.fol.Predicate ?predicate) {
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Symbol !x;

    observes : see(?actor,!x);

    ?return = op:newArrayList("java.util.Map");
    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    if (op:equalsValue(!x, obstacle)) {
      if (act:checkObstacle()) {
        op:log("debug", "?actor: found an obstacle.");
        op:add(?return, !bindings);
      } else {
        op:log("debug", "?actor: no obstacle found.");
      }
    } elseif (op:equalsValue(!x, support)) {
      if (act:checkFloorSupport()) {
        op:log("debug", "?actor: found floor support.");
        op:add(?return, !bindings);
      } else {
        op:log("debug", "?actor: no floor support found.");
      }
    } else {
      op:log("debug", "?actor: can't check for: !x");
    }
}

() = move["?actor moves ?direction"](edu.tufts.hrilab.fol.Symbol ?direction) {
    edu.tufts.hrilab.fol.Predicate !failCond;

    locks : motionLock;

    try {
      if (op:equalsValue(?direction, straight) || op:equalsValue(?direction, forward)) {
        act:moveForward();
      } elseif (op:equalsValue(?direction, back)) {
        act:moveBack();
      } else {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,meaningOf(moving(?direction))))");
        exit(FAIL, !failCond);
      }
    } finally {
      op:log("debug", "[move(?actor, ?direction)] in finally block");
      act:stop();
    }
}

() = stand() {
    act:goToPosture("Stand");
}

() = crouch() {
    act:goToPosture("Crouch");
}

() = sit() {
    act:goToPosture("Sit");
}

() = lieDown() {
    act:goToPosture("LyingBack");
}

() = moveBack["?actor moves back"]() {
    edu.tufts.hrilab.fol.Predicate !failCond;

    conditions : {
      or : {
        pre : behind(?actor,sensor);
        pre : bel(?actor,propertyOf(behind(area,?actor),safe));
      }
      or : {
        overall : behind(?actor,sensor);
        overall : bel(?actor,propertyOf(behind(area,?actor),safe));
      }
    }

    locks : motionLock;

    act:setTV(-0.25);

    op:sleep(300);
    if (~act:isMoving()) {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(propertyOf(?actor,standing))");
      exit(FAIL, !failCond);
    }

    // moveTo is non-blocking. Loop here to keep checking overall conditions
    while (act:isMoving()) {
      op:sleep(100);
    }
}

() = look["?actor looks in ?direction"](edu.tufts.hrilab.fol.Symbol ?direction) {
    edu.tufts.hrilab.fol.Predicate !failCond;

    locks : motionLock;

    op:log("debug", "In look(?direction) script for NAO.");
    if (op:equalsValue(?direction, straight) || op:equalsValue(?direction, forward)) {
      act:pointHeadTo(0.2, 0.0, 0.5);
    } elseif (op:equalsValue(?direction, left)) {
      act:pointHeadTo(0.2, 0.4, 0.5);
    } elseif (op:equalsValue(?direction, right)) {
      act:pointHeadTo(0.2, -0.4, 0.5);
    } elseif (op:equalsValue(?direction, up)) {
      act:pointHeadTo(0.2, 0.0, 0.6);
    } elseif (op:equalsValue(?direction, down)) {
      act:pointHeadTo(0.2, 0.0, 0.4);
    } else {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,meaningOf(looking(?direction))))");
      exit(FAIL, !failCond);
    }
}

() = pause["?actor pauses for 1 second"]() {
    op:sleep(1000);
}

() = standUp() {
    java.util.List !joints;
    java.util.List !angles;
    java.lang.Float !duration = 2.0;

    op:log("debug", "standUp");

    !joints = op:newArrayList("java.lang.String");
    !angles = op:newArrayList("java.lang.String");
    op:add(!joints, "LHipYawPitch");
    op:add(!joints, "LHipPitch");
    op:add(!joints, "RHipPitch");
    op:add(!joints, "LKneePitch");
    op:add(!joints, "RKneePitch");
    op:add(!joints, "LAnklePitch");
    op:add(!joints, "RAnklePitch");
    op:add(!joints, "LAnkleRoll");
    op:add(!joints, "RAnkleRoll");
    op:add(!joints, "LHipRoll");
    op:add(!joints, "RHipRoll");
    op:add(!angles, -.1);
    op:add(!angles, 0.1);
    op:add(!angles, 0.1);
    op:add(!angles, 0);
    op:add(!angles, 0);
    op:add(!angles, 0);
    op:add(!angles, 0);
    op:add(!angles, -0.1);
    op:add(!angles, 0.1);
    op:add(!angles, 0.1);
    op:add(!angles, -0.1);
    act:angleInterpolation(!joints, !angles, !duration, true);
}

() = crouchDown() {
    java.util.List !joints;
    java.util.List !angles;
    java.lang.Float !duration = 2.0;

    op:log("debug", "crouchDown");

    !joints = op:newArrayList("java.lang.String");
    !angles = op:newArrayList("java.lang.String");
    op:add(!joints, "LHipYawPitch");
    op:add(!joints, "LHipPitch");
    op:add(!joints, "RHipPitch");
    op:add(!joints, "LKneePitch");
    op:add(!joints, "RKneePitch");
    op:add(!joints, "LAnklePitch");
    op:add(!joints, "RAnklePitch");
    op:add(!joints, "LAnkleRoll");
    op:add(!joints, "RAnkleRoll");
    op:add(!joints, "LHipRoll");
    op:add(!joints, "RHipRoll");
    op:add(!angles, -.1);
    op:add(!angles, -1);
    op:add(!angles, -1);
    op:add(!angles, 1.8);
    op:add(!angles, 1.8);
    op:add(!angles, -.8);
    op:add(!angles, -.8);
    op:add(!angles, -0.1);
    op:add(!angles, 0.1);
    op:add(!angles, 0.1);
    op:add(!angles, -0.1);
    act:angleInterpolation(!joints, !angles, !duration, true);
}

() = raise(edu.tufts.hrilab.fol.Symbol ?arm) {
    java.util.List !joints;
    java.util.List !angles;
    java.lang.Float !duration = 1.5;
    edu.tufts.hrilab.fol.Predicate !failCond;

    op:log("debug", "raise ?arm");

    if (op:equalsValue(?arm, left) || op:equalsValue(?arm, leftarm) || op:equalsValue(?arm, lefthand)) {
      act:angleInterpolation(LShoulderPitch, 0, !duration, true);
    } elseif (op:equalsValue(?arm, right) || op:equalsValue(?arm, rightarm) || op:equalsValue(?arm, righthand)) {
      act:angleInterpolation(RShoulderPitch, 0, !duration, true);
    } elseif (op:equalsValue(?arm, hands) || op:equalsValue(?arm, arms)) {
      !joints = op:newArrayList("java.lang.String");
      !angles = op:newArrayList("java.lang.String");
      op:add(!joints, "LShoulderPitch");
      op:add(!joints, "RShoulderPitch");
      op:add(!angles, 0);
      op:add(!angles, 0);
      act:angleInterpolation(!joints, !angles, !duration, true);
    } else {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,meaningOf(raising(?arm))))");
      exit(FAIL, !failCond);
    }
}

() = lower(edu.tufts.hrilab.fol.Symbol ?arm) {
    java.util.List !joints;
    java.util.List !angles;
    java.lang.Float !duration = 1.5;
    edu.tufts.hrilab.fol.Predicate !failCond;

    op:log("debug", "lower ?arm");

    if (op:equalsValue(?arm, left) || op:equalsValue(?arm, leftarm) || op:equalsValue(?arm, lefthand)) {
      act:angleInterpolation(LShoulderPitch, 1.6, !duration, true);
    } elseif (op:equalsValue(?arm, right) || op:equalsValue(?arm, rightarm) || op:equalsValue(?arm, righthand)) {
      act:angleInterpolation(RShoulderPitch, 1.6, !duration, true);
    } elseif (op:equalsValue(?arm, hands) || op:equalsValue(?arm, arms)) {
      !joints = op:newArrayList("java.lang.String");
      !angles = op:newArrayList("java.lang.String");
      op:add(!joints, "LShoulderPitch");
      op:add(!joints, "RShoulderPitch");
      op:add(!angles, 1.6);
      op:add(!angles, 1.6);
      act:angleInterpolation(!joints, !angles, !duration, true);
    } else {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,meaningOf(lowering(?arm))))");
      exit(FAIL, !failCond);
    }
}

() = ready() {
    int !counter = 0;
    while (op:lt(!counter, 12)) {
      act:ledOff(EarLeds);
      op:sleep(250);
      act:ledOn(EarLeds);
      op:sleep(250);
      !counter = op:++(!counter);
    }
}

() = turn(edu.tufts.hrilab.fol.Symbol ?direction, edu.tufts.hrilab.fol.Symbol ?theta = infinite) {
    java.lang.Float !rad = 0.01745;
    try {
      // determine direction
      if (op:equalsValue(?direction, right)) {
        !rad = op:*(!rad, -1);
      }

      // determine angle
      if (op:equalsValue(?theta, ninety)) {
        !rad = op:*(!rad, 90);
      } elseif (op:equalsValue(?theta, oneeighty)) {
        !rad = op:*(!rad, 180);
      } elseif (op:equalsValue(?theta, twoseventy)) {
        !rad = op:*(!rad, 270);
      }

      // determine if it is turning forever or not
      if (op:equalsValue(?theta, infinite)) {
        act:setRV(!rad);
      } else {
        act:moveToBlocking(0.0, 0.0, !rad);
      }

      // setting vels is non-blocking. Loop here to keep action alive
      while (act:isMoving()) {
        op:sleep(100);
      }
    } finally {
      op:log("debug", "[turn(?actor, ?direction)] in finally block");
      act:stop();
      act:stand();
    }
}
