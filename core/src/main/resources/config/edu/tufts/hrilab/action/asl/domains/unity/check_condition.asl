(java.lang.Boolean ?isAllowed) = checkAllowedUnity(edu.tufts.hrilab.fol.Predicate ?state){
  java.lang.Boolean !finishedRepair = False;
  java.lang.Boolean !hasRepairTube;
  java.lang.String !actorStr;
  java.util.List !actorList;
  java.lang.String !stringThisRobot;
  edu.tufts.hrilab.fol.Symbol !currentRobot;
  java.lang.Class !getClass;
  java.util.List !globalPlace;
  java.lang.Boolean !wait = False;

  op:log("info", "This is what is passed into checkCondition as state:");
  op:log("info", ?state);

  !stringThisRobot = op:invokeMethod(?actor, "getName");
  op:log("info", !stringThisRobot);

  // Hard coded for Unity simulation as of 11/21
  !actorList = op:newArrayList("java.util.List");
  op:add(!actorList, "robot1");
  op:add(!actorList, "robot2");

  op:remove(!actorList, !stringThisRobot);

  // Right now, this will just loop util a repair is done. It wont say anything or check anything else.
  op:log("info", !finishedRepair);
  // These only run if the robot is currently repairing a tube. I am not going to bother optimizing which other
  // robot since we wont have more in this case and it's pretty specific
  if (op:equals(!wait, True)) {
    while (~op:equals(!finishedRepair, True)) {
      !finishedRepair = op:newObject(java.lang.Boolean, True);
      op:log("info", "hi");
      !hasRepairTube = act:checkRepairing(?actor);
      op:log("info", !hasRepairTube);
      if (op:equals(!hasRepairTube, True)) {
        !finishedRepair = op:newObject("java.lang.Boolean", False);
      }
    }
  } else {
    // When not waiting, checks if any other robot is free.
    !hasRepairTube = act:checkRepairing(?actor);
    //This currently only works when there are two robots max. It can work with more if it breaks from the loop
    // when one is reassigned
    if (op:equals(!hasRepairTube, True)){
      (?isAllowed) = op:newObject("java.lang.Boolean", False);
      foreach(!currentRobot: !actorList){
        !hasRepairTube = act:checkRepairing(!currentRobot);
        if (op:equals(!hasRepairTube, False)){
          act:askAssignmentQuestion("Brad", ?state, !currentRobot);
        }
      }
    }
  }

  //TODO: Add a section here about assigning another robot to the wing if it's closer

  ?isAllowed = op:newObject("java.lang.Boolean", True);
}

//["Returns True if the actor is repairing a tube"]
(java.lang.Boolean ?repairing) = checkRepairing(edu.tufts.hrilab.fol.Symbol ?actorToCheck){
  edu.tufts.hrilab.fol.Predicate !pred;
  java.lang.String !predString;
  java.lang.Boolean !hasRepairTube;
  java.util.List !goalPreds;

  (!goalPreds) = act:getCurrentGoals(?actorToCheck);
  op:log("info", "?actorToCheck: initial goals: !goalPreds");
  ?repairing = op:newObject("java.lang.Boolean", False);
  foreach(!pred : !goalPreds) {
    !predString = op:newObject("java.lang.String", "");
    !predString = op:invokeMethod(!pred, "toString");
    !hasRepairTube = op:invokeMethod(!predString, "contains", "repairTube");
    if (op:==(!hasRepairTube, true)) {
      op:log("debug", "goalObserver: ?actorToCheck: has goal : !pred");
      ?repairing = op:newObject("java.lang.Boolean", True);
    }
  }
}