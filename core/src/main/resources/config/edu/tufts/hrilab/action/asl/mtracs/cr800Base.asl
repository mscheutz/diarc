// The cr800Base script contains all actions that are core to interacting
// with the underlying primitives in the cr800 component. Writing multi step
// actions which will work for our compilation system must be done here.

//====================== Robot arm movement action scripts ============
() = goToPose["goes to pose without adjustment"](edu.tufts.hrilab.fol.Symbol ?pose) {
    edu.tufts.hrilab.mtracs.util.MPose !goalPose;

    op:log(debug, "[goToPose] pose only");
    (!goalPose) = tsc:getPoseFromSymbol(?pose);
    tsc:goToPose(!goalPose);
}

() = goToPoseLong["goes to pose without adjustment the long way around"](edu.tufts.hrilab.fol.Symbol ?pose) {
    edu.tufts.hrilab.mtracs.util.MPose !goalPose;
    java.lang.Boolean !success;

    (!goalPose) = tsc:getPoseFromSymbol(?pose);
    (!success) = tsc:goToPoseLong(!goalPose);
}

() = goToPose["goes to pose with adjustment to the given cameraHeight"](edu.tufts.hrilab.fol.Symbol ?pose, edu.tufts.hrilab.fol.Symbol ?cameraHeight) {
    edu.tufts.hrilab.mtracs.util.MPose !goal;
    (!goal) = tsc:getPoseFromSymbol(?pose);
    (!goal) = tsc:adjustPoseToCameraHeight(!goal, ?cameraHeight);
    tsc:goToPose(!goal);
}

() = goToPoseLong["goes to pose with adjustment to the given cameraHeight the long way around"](edu.tufts.hrilab.fol.Symbol ?pose, edu.tufts.hrilab.fol.Symbol ?cameraHeight) {
    edu.tufts.hrilab.mtracs.util.MPose !goal;
    (!goal) = tsc:getPoseFromSymbol(?pose);
    (!goal) = tsc:adjustPoseToCameraHeight(!goal, ?cameraHeight);
    tsc:goToPoseLong(!goal);
}


() = dip["moves down and up by the distance given"](java.lang.Double ?dist) {
    java.lang.Double !downDist;
    (!downDist) = op:*(?dist, -1.0);
    tsc:moveZRelative(?dist);
    tsc:moveZRelative(!downDist);
}

//====================== Save a pose and its offset ============
() = recordCameraPoseAsk["records current pose and asks for an off set"](edu.tufts.hrilab.fol.Symbol ?poseName){
  //answer bindings
  java.util.Map !bindings;
  edu.tufts.hrilab.fol.Symbol !z;
  edu.tufts.hrilab.fol.Variable !x = "X";
  //TODO:brad: this is like this for explicit actor based reasons. passing in ?actor for a role other than actor causes issues in ActionContext.java line 86 which I think is itself a work around...
  edu.tufts.hrilab.fol.Symbol !speaker;

 (!speaker) = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

  (!bindings) = act:askQuestion(!speaker,"what is its offset", val(X,Y));
  (!z) = op:get(!bindings, !x);
  tsc:recordPose(?poseName,!z);

   act:submitTTSRequest(!speaker,!speaker,"okay");
}

//====================== Gripper Switching =====================
() = rotateToEE["changes TCP of robot to refer to a new EE attached to the cuff at a different offset"](edu.tufts.hrilab.fol.Symbol ?gripperType) {
    edu.tufts.hrilab.mtracs.util.MPose !currentPose;
    edu.tufts.hrilab.mtracs.util.MPose !newTCP;
    (!currentPose) = tsc:getCurrentPose();
     op:log(debug, "getting tcp for ?gripperType in rotate");
    (!newTCP) = tsc:getTCPForEE(?gripperType);
    tsc:alternateEE(!newTCP);
    tsc:goToPose(!currentPose);
}

() = putAwayGripper(edu.tufts.hrilab.fol.Symbol ?gripperType) {
    java.lang.Double !asc = 0.10;
    edu.tufts.hrilab.mtracs.util.MPose !pose;
    edu.tufts.hrilab.mtracs.util.MPose !approachPose;
    (!pose) = tsc:getGripDropoff(?gripperType);
    (!approachPose) = op:invokeMethod(!pose, "getHorizontalApproachPose");
    tsc:goToPose(!approachPose);
    tsc:closeGripper();
    tsc:goToPose(!pose);
    tsc:openGripper();
    tsc:ejectEE();
    tsc:moveZRelative(!asc);
    tsc:moveAwayFromJointLimit();
}

() = getGripper(edu.tufts.hrilab.fol.Symbol ?gripperType) {
    java.lang.Double !proj = 0.10;
    java.lang.Double !dly = 0.5;
    edu.tufts.hrilab.mtracs.util.MPose !pose;
    edu.tufts.hrilab.mtracs.util.MPose !approachPose;
    (!pose) = tsc:getGripDropoff(?gripperType);
    (!approachPose) = op:invokeMethod(!pose, "getVerticalApproachPose");
    tsc:goToPose(!approachPose);
    tsc:openGripper();
    tsc:goToPose(!pose);
    tsc:closeGripper();
    tsc:delay(!dly);
    tsc:acceptEE();
    tsc:moveXRelative(!proj);
    tsc:moveAwayFromJointLimit();
}
