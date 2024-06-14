import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

/**
 * ?actor moves ?objectRef in ?direction with ?arm (assumed that object is already being grasped)
 */
() = moveObjectInDirection(Symbol ?objectRef, Symbol ?direction, Symbol ?arm = arm, double ?dist = 0.15) {
    conditions : {
      pre : grasping(?actor,?objectRef,?arm);
    }
    effects : {
      success : grasping(?actor,?objectRef,?arm);
    }

    op:log("debug", "moving ?objectRef ?direction");
    act:moveToRelative(?direction, ?arm, ?dist);
}

/**
 * ?actor moves ?arm in ?direction a distance of ?dist.
 */
() = moveToRelative(Symbol ?direction, Symbol ?arm = arm, double ?dist = 0.15) {
    double !negDist = op:*(?dist, -1.0);
    javax.vecmath.Point3d !relativeLoc;
    javax.vecmath.Quat4d !relativeOrient = op:newObject("javax.vecmath.Quat4d",0,0,0,1);
    Predicate !failCond;

    op:log("debug", "moving ?arm ?direction");
    if (op:equalsValue(?direction, up)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", 0, 0, ?dist);
    } elseif (op:equalsValue(?direction, down)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", 0, 0, !negDist);
    } elseif (op:equalsValue(?direction, left)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", 0, ?dist, 0);
    } elseif (op:equalsValue(?direction, right)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", 0, !negDist, 0);
    } elseif (op:equalsValue(?direction, forward)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", ?dist, 0, 0);
    } elseif (op:equalsValue(?direction, backward)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", !negDist, 0, 0);
    } elseif (op:equalsValue(?direction, forwardDown)) {
      !relativeLoc = op:newObject("javax.vecmath.Point3d", ?dist, 0, !negDist);
    } else {
      op:log("error", "moveToRelative can't handle ?direction. Exiting.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "unknown(?direction)");
      exit(FAIL, !failCond);
    }

    op:log("debug", "moving ?arm !relativeLoc !relativeOrient");
    act:moveToRelative(?arm, !relativeLoc, !relativeOrient);
    op:log("debug", "end moving");
}

/**
 * ?actor uses ?arm to move ?objectRef_1 ?relation ?objectRef_2
 */
() = moveObjectRelativeTo(Symbol ?objectRef_1, Symbol ?relation, Symbol ?objectRef_2, Symbol ?arm = arm, double ?dist = 0.5) {
    java.util.List !tokens;
    edu.tufts.hrilab.vision.stm.MemoryObject !token_2;
    org.apache.commons.lang3.tuple.Pair !armPose;
    javax.vecmath.Point3d !armLoc;
    javax.vecmath.Point3d !mo2Loc;
    javax.vecmath.Vector3d !dirVec;
    javax.vecmath.Point3d !zDir;
    javax.vecmath.Point3d !targetLoc;
    javax.vecmath.Quat4d !targetOrient = op:newObject("javax.vecmath.Quat4d",0,0,0,1);
    Predicate !failCond;

    op:log("debug", "moving ?objectRef_1 ?relation ?objectRef_2");

    // get pose of ?arm, instead of ?objectRef_1 (robot might be not be looking at the object)
    !armPose =  act:getPose(?arm);

    // find objectRef_2
    act:findObject(?objectRef_2);

    // get MemoryObject for ?bjectRef_2
    op:log("debug", "Found ?objectRef_2");
    !tokens = act:getTokens(?objectRef_2);
    !token_2 = op:get(!tokens, 0);

    // transform object to base coordinate frame
    op:invokeMethod(!token_2, "transformToBase");

    if (op:equalsValue(?relation, toward)) {
      // calculate direction vector
      !armLoc = op:invokeMethod(!armPose, "getLeft");
      !mo2Loc = op:invokeMethod(!token_2, "getLocation");
      !dirVec = op:newObject("javax.vecmath.Vector3d");
      op:invokeMethod(!dirVec, "sub", !mo2Loc, !armLoc);
      op:invokeMethod(!dirVec, "normalize");

      // calculate final relative location by scaling direction by dist to move
      !targetLoc = op:newObject("javax.vecmath.Point3d", !dirVec);
      op:invokeMethod(!targetLoc, "scale", ?dist);

      // move arm and object
      op:log("debug", "moving relative: ?arm !targetLoc !targetOrient");
      act:moveToRelative(?arm, !targetLoc, !targetOrient);
    } elseif (op:equalsValue(?relation, above)) {
      // calculate target location
      !targetLoc = op:invokeMethod(!token_2, "getLocation");
      !zDir = op:newObject("javax.vecmath.Point3d" ,0,0,0.1); // add 0.1m to z-height of target object's location
      op:invokeMethod(!targetLoc, "add", !zDir);

      // move arm and object
      !targetOrient = op:invokeMethod(!armPose, "getRight");
      op:log("debug", "moving to target location: ?arm !targetLoc !targetOrient");
      act:moveTo(?arm, !targetLoc, !targetOrient);
    } else {
      op:log("error", "moveObject can't handle ?relation. Exiting.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "unknown(?relation)");
      exit(FAIL, !failCond);
    }

    op:log("debug", "end moving");
}

/**
 * ?actor picks up ?objectRef with ?arm
 */
() = pickUp(Symbol ?objectRef, Symbol ?arm = "arm") {
    Symbol !otherObj;

    conditions : {
      pre : not(grasping(?actor,!otherObj,?arm));
    }
    effects : {
      success : holding(?actor,?objectRef,?arm);
    }

    act:graspObject(?objectRef, ?arm);
    act:moveObjectInDirection(?objectRef, up, ?arm);
    op:log("debug", "[graspObject] successfully grasped ?objectRef");
}

() = graspObject(Symbol ?objectRef, Symbol ?arm = "arm") {
    float !closePosition = 0.0f;

    effects : {
      success : grasping(?actor,?objectRef,?arm);
      success infer : touching(?actor,?objectRef);
    }

    act:findObject(?objectRef);
    act:openGripper(?arm);
    act:moveTo(?arm, ?objectRef);
    act:graspObject(?arm, ?objectRef, !closePosition);
    act:stopAllSearches();
    op:log(debug, "[graspObject] successfully grasped ?objectRef");
}

/**
 * ?actor releases object ?objectRef
 */
() = releaseObject(Symbol ?objectRef, Symbol ?arm = "arm") {
    conditions : {
      pre : grasping(?actor,?objectRef,?arm);
    }
    effects : {
      success : not(grasping(?actor,?objectRef,?arm));
      success : not(grasping(?actor,?objectRef));
      success infer : not(touching(?actor,?objectRef));
      success infer : not(holding(?actor,?objectRef,?arm));
    }

    op:log("debug", "Trying to release ?objectRef.");
    act:releaseObject(?arm, ?objectRef, 1.0);
    act:moveToRelative(up, ?arm);
    act:goToPose(carry);
}

/**
 * ?actor hands over ?objectRef to ?actor2 with ?arm
 */
() = handOverObject(Symbol ?actor, Symbol ?actor2, Symbol ?objectRef) {
    Symbol !arm;
    javax.vecmath.Point3d !point; // = op:newObject("javax.vecmath.Point3d",0.7,0.0,1.2);
    javax.vecmath.Quat4d !orient; // = op:newObject("javax.vecmath.Quat4d",-0.5,-0.5,0.5,-0.5);
    javax.vecmath.Point3d !pointHeadLoc; // = op:newObject("javax.vecmath.Point3d",0.7,0.0,1.2);

    conditions : {
      pre : grasping(?actor,?objectRef,!arm);
    }
    effects : {
      success : not(have(?actor,?objectRef));
      success : have(?actor2,?objectRef);
      success : touching(?actor2,?objectRef);
    }

    !point = op:newObject("javax.vecmath.Point3d",0.7,0.0,1.2);
    !orient = op:newObject("javax.vecmath.Quat4d",-0.5,-0.5,0.5,-0.5);

    op:log("debug", "graspObject actor: ?actor actor2: ?actor2 object: ?objectRef");

    // TODO: find location of ?actor2 in order to look at and hand it to him/her
    // point head
    // !pointHeadLoc = op:newObject("javax.vecmath.Point3d",1.0,0.0,1.5);
    // act:pointHeadTo(!pointHeadLoc);

    // move arm to hand over pose
    act:moveTo(!arm, !point, !orient);

    // sleep
    // op:sleep(1000);
    ?actor2.act:graspObject(?objectRef);

    // release object
    act:releaseObject(!arm, ?objectRef, 1.0);

    // sleep
    // op:sleep(1000);

    // go to start pose
    act:goToPose(!arm, start);

    op:log("debug", "Done with hand over.");
}

/**
 * ?actor points to ?objectRef with ?arm
 */
() = pointTo(Symbol ?objectRef:physobj, Symbol ?arm:arm = arm) {
    java.lang.Long !typeId;
    Predicate !failCond;

    conditions : {
      pre infer : see(?actor,?objectRef);
    }

    // find object
    act:findObject(?objectRef, !typeId);

    // close gripper(s)
    act:moveGripper(?arm, 0.0f);

    // pause camera capture bc arm will likely occlude scene
    act:pauseCapture();

    // point to object
    if (act:pointTo(?arm, ?objectRef)) {
      op:log("debug", "pointing to ?objectRef");

      // sleep for a few seconds
      op:sleep(3000);

      // go back to start pose
      act:goToStartPose(true);

      // resume camera capture after arm is out of the way
      act:resumeCapture();
      op:log("debug", "done pointing to ?objectRef");
    } else {
      op:log("debug", "failed to point to ?objectRef");
      act:resumeCapture();
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(find(?actor, path))");
      exit(FAIL, !failCond);
    }
}
