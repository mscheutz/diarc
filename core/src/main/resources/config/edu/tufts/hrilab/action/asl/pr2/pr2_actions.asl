() = look["?actor looks in ?direction"](edu.tufts.hrilab.fol.Symbol ?direction) {
    javax.vecmath.Point3d !location;
    edu.tufts.hrilab.fol.Predicate !failCond;

    locks : motionLock;

    op:log("debug", "In look(?direction) script for PR2.");

    if (op:equalsValue(?direction, straight) || op:equalsValue(?direction, forward)) {
      !location = op:newObject("javax.vecmath.Point3d", 1.0, 0.0, 1.2);
    } elseif (op:equalsValue(?direction, left)) {
      !location = op:newObject("javax.vecmath.Point3d", 1.0, 0.7, 1.2);
    } elseif (op:equalsValue(?direction, right)) {
      !location = op:newObject("javax.vecmath.Point3d", 1.0, -0.7, 1.2);
    } elseif (op:equalsValue(?direction, up)) {
      !location = op:newObject("javax.vecmath.Point3d", 0.35, -0.1, 1.5);
    } elseif (op:equalsValue(?direction, down)) {
      !location = op:newObject("javax.vecmath.Point3d", 0.35, -0.1, 1.0);
    } else {
      op:log("debug", "Exiting look ?direction.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "unable(look(?direction))");
      exit(FAIL, !failCond);
    }

    act:pointHeadTo(!location);
}

() = raise(edu.tufts.hrilab.fol.Symbol ?arm) {
    java.lang.String !arms_pr2;
    edu.tufts.hrilab.fol.Predicate !failCond;

    op:log("debug", "raise ?arm");
    if (op:equalsValue(?arm, left) || op:equalsValue(?arm, leftArm) || op:equalsValue(?arm, leftHand)) {
      !arms_pr2 = op:newObject("java.lang.String", "leftArm");
    } elseif (op:equalsValue(?arm, right) || op:equalsValue(?arm, rightArm) || op:equalsValue(?arm, rightHand)) {
      !arms_pr2 = op:newObject("java.lang.String", "rightArm");
    } elseif (op:equalsValue(?arm, hands) || op:equalsValue(?arm, arms)) {
      !arms_pr2 = op:newObject("java.lang.String", "arms");
    } else {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,meaningOf(raising(?arm))))");
      exit(FAIL, !failCond);
    }

    if (act:goToPose(!arms_pr2, "raiseArms")) {
      op:log("debug", "Raised ?arms.");
    } else {
      op:log("debug", "Couldn't plan path to raise ?arm.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(find(?actor, path))");
      exit(FAIL, !failCond);
    }
}

() = lower(edu.tufts.hrilab.fol.Symbol ?arm) {
    java.lang.String !arms_pr2;
    edu.tufts.hrilab.fol.Predicate !failCond;

    op:log("debug", "lower ?arm");

    if (op:equalsValue(?arm, left) || op:equalsValue(?arm, leftArm) || op:equalsValue(?arm, leftHand)) {
      !arms_pr2 = op:newObject("java.lang.String", "leftArm");
    } elseif (op:equalsValue(?arm, right) || op:equalsValue(?arm, rightArm) || op:equalsValue(?arm, rightHand)) {
      !arms_pr2 = op:newObject("java.lang.String", "rightArm");
    } elseif (op:equalsValue(?arm, hands) || op:equalsValue(?arm, arms)) {
      !arms_pr2 = op:newObject("java.lang.String", "arms");
    } else {
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(know(?actor,meaningOf(lowering(?arm))))");
      exit(FAIL, !failCond);
    }

    if (act:goToPose(!arms_pr2, "lowerArms")) {
      op:log("debug", "Lowered ?arms.");
    } else {
      op:log("debug", "Couldn't plan path to lower ?arm.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(find(?actor, path))");
      exit(FAIL, !failCond);
    }
}
