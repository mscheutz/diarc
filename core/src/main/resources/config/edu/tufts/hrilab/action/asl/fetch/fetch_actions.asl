() = look["?actor looks in ?direction"](edu.tufts.hrilab.fol.Symbol ?direction) {
    javax.vecmath.Point3d !location;
    locks : motionLock;

    op:log("debug", "In look(?direction) script for PR2.");

    if (op:equalsValue(?direction, straight) || op:equalsValue(?direction, forward)) {
        !location = op:newObject("javax.vecmath.Point3d", 1.0, 0.0, 1.2);
    }
    elseif (op:equalsValue(?direction, left)) {
        !location = op:newObject("javax.vecmath.Point3d", 1.0, 0.7, 1.2);
    }
    elseif (op:equalsValue(?direction, right)) {
        !location = op:newObject("javax.vecmath.Point3d", 1.0, -0.7, 1.2);
    }
    elseif (op:equalsValue(?direction, up)) {
        !location = op:newObject("javax.vecmath.Point3d", 0.35, -0.1, 1.5);
    }
    elseif (op:equalsValue(?direction, down)) {
        !location = op:newObject("javax.vecmath.Point3d", 0.35, -0.1, 1.0);
    }
    else {
        exit(FAIL, "unable(look(?direction))");
    }
    act:pointHeadTo(!location);
}

() = raise(edu.tufts.hrilab.fol.Symbol ?arm) {
    java.lang.String !arms_pr2;

    op:log("debug", "raise ?arm");

    if (op:equalsValue(?arm, arm) || op:equalsValue(?arm, hand)) {
        !arms_pr2 = op:newObject("java.lang.String", "arm");
    } else {
        exit(FAIL, "not(know(?actor,meaningOf(raising(?arm))))");
    }

    if (act:moveTo(?arm, ${new, javax.vecmath.Point3d(.2,0,0)}, ${new, javax.vecmath.Quat4d(0,0,0,1)})) {
        op:log("debug", "Raised ?arms.");
    } else {
        op:log("debug", "Couldn't plan path to raise ?arm.");
        exit(FAIL, ${Packages.edu.tufts.hrilab.edu.tufts.hrilab.fol.Factory.createPredicate("not(find(?actor,, path))")});
    }
}

() = lower(edu.tufts.hrilab.fol.Symbol ?arm) {
    java.lang.String !arms_pr2;

    op:log("debug", "lower ?arm");

    if (op:equalsValue(?arm, arm) || op:equalsValue(?arm, hand)) {
        !arms_pr2 = op:newObject("java.lang.String", "arm");
    } else {
        exit(FAIL, "not(know(?actor,meaningOf(raising(?arm))))");
    }

    if (act:moveTo(?arm, ${new, javax.vecmath.Point3d(-.2,0,0)}, ${new, javax.vecmath.Quat4d(0,0,0,1)})) {
        op:log("debug", "Lowered ?arms.");
    } else {
        op:log("debug", "Couldn't plan path to lower ?arm.");
        exit(FAIL, "not(find(?actor,, path))");
    }
}

