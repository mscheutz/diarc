//TODO: Will: Not sure where changes specific to the moveItComponent should be  - so moving these actions to the demos/pour.asl script
////TODO:brad: I think the names of grab and pickup are switched?
() = pickup["?actor grabs ?physobj"](edu.tufts.hrilab.fol.Symbol ?objectRef) {
    conditions : {
        pre : free(?actor);
    }
    effects : {
        success : holding(?actor, ?objectRef);
        success : not(free(?actor));
    }

    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    java.lang.String !arm = "manipulator";

    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,0,0,0.15);

    op:log(info, "pickup");
    //act:openGripper("arm");
    //act:findGraspableObject(?physobj);
    //act:moveTo("arm", ?physobj);
    //act:closeGripper("arm");
    //act:findGraspableObject(?physobj);
    //TODO:brad/will: this change is so it can put thinks back where it picked them up from, we maybe want a more general solution moving on.
    //act:pauseCapture();
//    act:grab(?physobj);

    float !closePosition = 0.0f;
    edu.tufts.hrilab.fol.Predicate !failCond;

    // TODO: this probably shouldn't be an explicit step here, maybe pre-condition?
    act:findGraspableObject(?objectRef);
    act:openGripper(!arm);

    op:log(debug, "Trying to grasp ?objectRef with !arm.");
    if (act:moveTo(!arm, ?objectRef)) {
      op:log(debug, "Moved to ?objectRef with !arm.");
    } else {
      op:log(debug, "Couldn't plan path to grasp ?objectRef with !arm.");
      !failCond = op:invokeStaticMethod(edu.tufts.hrilab.util.Util, createPredicate, "not(find(?actor, path))");
      exit(fail, !failCond);
    }
   //   if (act:graspObject(?arm, !closePosition, ?objectRef)) {
   if (act:closeGripper(!arm)) {
      op:log(debug, "Grasped ?objectRef with !arm.");
    } else {
      op:log(debug, "Couldn't grasp ?objectRef with !arm.");
      !failCond = op:invokeStaticMethod(edu.tufts.hrilab.util.Util, createPredicate, "not(graspObject(?objectRef))");
      exit(fail, !failCond);
    }
    op:log(info, "Ended grab and now about to pick up");
    act:moveToRelative(!arm, !pt, !qt);

}
//
//() = release["?actor releases ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj) {
//    conditions : {
//        pre : holding(?actor, ?physobj);
//    }
//    effects : {
//        success : not(holding(?actor, ?physobj));
//        success : free(?actor);
//    }
//
//    javax.vecmath.Point3d !pt;
//    javax.vecmath.Quat4d !qt;
//    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
//    (!pt) = op:newObject(javax.vecmath.Point3d,0,0,-0.15);
//
//    op:log(info, "release");
//    act:moveToRelative("manipulator", !pt, !qt);
//    act:openGripper("manipulator");
//    //act:putDown(?physobj);
//    //act:moveBackward();
//    (!pt) = op:newObject(javax.vecmath.Point3d,0,0,0.25);
//    act:moveToRelative("manipulator", !pt, !qt);
//    act:resumeCapture();
//}

() = putDown["?actor releases ?physobj"]() {
       edu.tufts.hrilab.fol.Predicate !goalPredicate;
       !goalPredicate = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "free(?actor)");
       goal:!goalPredicate;
}


() = moveUp["?actor moves object up"](java.lang.String ?link = "manipulator") {
    op:log(debug, "move up");
    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,0,0,0.20);
    act:moveToRelative(?link, !pt, !qt);
}

() = moveDown["?actor moves ee down"](java.lang.String ?link = "manipulator") {
    op:log(debug, "move down");
    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,0,0,-0.20);
    act:moveToRelative(?link, !pt, !qt);
}

() = moveLeft["?actor moves object left"](java.lang.String ?link = "manipulator") {
    op:log(debug, "move left");
    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,0,0.20,0);
    act:moveToRelative(?link, !pt, !qt);
}

() = moveRight["?actor moves object right"](java.lang.String ?link = "manipulator") {
    op:log(debug, "move right");
    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,0,-0.20,0);
    act:moveToRelative(?link, !pt, !qt);
}

() = moveForward["?actor moves object forward"](java.lang.String ?link = "manipulator") {
    op:log(debug, "move forward");
    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,0.20,0,0);
    act:moveToRelative(?link, !pt, !qt);
}

() = moveBackward["?actor moves object backward"](java.lang.String ?link = "manipulator") {
    op:log(debug, "move backward");
    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject(javax.vecmath.Quat4d,0,0,0,1);
    (!pt) = op:newObject(javax.vecmath.Point3d,-0.20,0,0);
    act:moveToRelative(?link, !pt, !qt);
}

() = tilt["?actor tilts ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj) {
    op:log(debug, "tilting");
//    act:tilt(physobj);
}
