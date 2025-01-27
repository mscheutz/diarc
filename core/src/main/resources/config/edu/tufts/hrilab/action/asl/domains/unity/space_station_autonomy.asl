import java.lang.String;
import java.lang.Boolean;
import java.lang.Integer;
import java.lang.Long;
import java.util.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.Map;
import java.util.HashMap;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;

import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.NLPacket;

import edu.tufts.hrilab.action.justification.Justification;
/**
 * AUTONOMY BEHAVIORS
 **/
 () = autonomyTest() {
   while (true) {
     op:log("info", "Running autonomy temp loop.");
    op:sleep(1000);
   }
 }

//["This is the high level autonomy manager function that will task a robot with monitoring an area if not already busy"]
() = autonomy(boolean ?isAdaptive) {
  Predicate !autonomy_goal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomy(robot1,true)");
  onInterrupt :{
    cancel: act:generateResponseFromString("Cancelling.");
    suspend: act:cancelGoal(!autonomy_goal);
    resume:  act:submitGoal(!autonomy_goal);
  }

  int !wait = 1000;
  Boolean !isHighWorkload;
  Symbol !nextArea;

  // wait for n milliseconds before starting autonomy. This is to give users a chance to override autonomy with
  // instructions without the robot instantly continuing autonomy between instructions.
  op:sleep(15000);
  act:generateResponseFromString("Starting autonomy action.");

  while (true) {
//    !isHighWorkload = act:isWorkloadAboveThreshold();
    !isHighWorkload = op:newObject("java.lang.Boolean", true); // for testing
    !nextArea = act:getNextAreaToMonitor();
    act:monitor(!nextArea, ?isAdaptive, !isHighWorkload);
  }
}

(Boolean ?isHighWorkload) = isWorkloadAboveThreshold () {
  Symbol !levelSymbol;
  String !levelString;
  Map !workloadBelief;

  Predicate !workloadPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "workload(X,Y)");
  List !workloadBeliefs = act:queryBelief(!workloadPredicate);
  Variable !var = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "Y");

  ?isHighWorkload = op:newObject("java.lang.Boolean", false);

  if (op:isEmpty(!workloadBeliefs)) {
    ?isHighWorkload = op:newObject("java.lang.Boolean", false);
  } else {
    foreach (!workloadBelief : !workloadBeliefs) {
      !levelSymbol = op:get(!workloadBelief, !var);
      !levelString = op:bind("!levelSymbol");
      if (op:equals(!levelString, "1")) {
        ?isHighWorkload = op:newObject("java.lang.Boolean", true);
        return;
      }
    }
  }
}

(Symbol ?nextArea) = getNextArea(Symbol ?lastArea, List ?currentAreaBeliefs, Symbol ?currentArea) {
  int !size = 2;

  op:log("info", "getNextArea curr: ?currentArea last: ?lastArea");

  List !areas = op:newArrayList("java.lang.String");
  op:add(!areas, "alpha");
  op:add(!areas, "beta");
  op:add(!areas, "gamma");

  if (~op:isEmpty(?currentAreaBeliefs)) {
    if (~op:equals(?currentArea, ?lastArea)) {
      !areaStr = op:invokeMethod(?currentArea, "toString");
      ?nextArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaStr);
      return;
    }
  }

  String !areaStr = op:invokeMethod(?lastArea, "toString");
  int !index = op:invokeMethod(!areas, "indexOf", !areaStr);

  if (op:lt(!index, !size)) {
    !index = op:++(!index);
    !areaStr = op:invokeMethod(!areas, "get", !index);
  } else {
    !areaStr = op:invokeMethod(!areas, "get", 0);
  }

  ?nextArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaStr);
}

//["This is the action that will determine the next area to monitor"]
(Symbol ?nextArea) = getNextAreaToMonitor() {
  Symbol !lastArea;
  Variable !keyX = "X";
  Map !currentAreaMap;
  Map !lastAreaMap;

  String !areaStr = "alpha";

  op:log("info", "activeMonitor()");

  Predicate !currentAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
  Predicate !lastAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "lastMonitored(?actor,X)");

  List !currentAreaBeliefs = act:queryBelief(!currentAreaBeliefPredicate);
  List !lastAreaBeliefs = act:queryBelief(!lastAreaBeliefPredicate);

  //temp
  Symbol !currentArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");

  if (~op:isEmpty(!lastAreaBeliefs)) {
    !lastAreaMap = op:get(!lastAreaBeliefs, 0);
    !lastArea = op:get(!lastAreaMap, !keyX);

    if (~op:isEmpty(!currentAreaBeliefs)) {
      !currentAreaMap = op:get(!currentAreaBeliefs, 0);
      !currentArea = op:get(!currentAreaMap, !keyX);
    }

    ?nextArea = act:getNextArea(!lastArea, !currentAreaBeliefs, !currentArea);
  } else {
    //first time only
    ?nextArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaStr);
  }

}

//["This will monitor an area and move to all tubes to "]
() = monitor(Symbol ?area, boolean ?isAdaptive, boolean ?highWorkload) {
  effects : {
    success : lastMonitored(?actor,?area);
  }

  // retract old last monitored belief
  Predicate !lastAreaMonitoredRetraction = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "lastMonitored(?actor,X)");
  act:retractBelief(!lastAreaMonitoredRetraction);

  Symbol !tubeToRepair;
  int !damagedTubesSize;

  act:goToAreaSilent(?area);

  List !damagedTubes = act:getSpaceStationTubesDamaged(?area);

  if (~op:isEmpty(!damagedTubes)) {
    if (op:equals(?isAdaptive, true) && op:equals(?highWorkload, true)) {
      // attempt repair of damaged tube(s)
      // TODO: are these ordered by health --> repairing most damaged?
      foreach(!tubeToRepair : !damagedTubes) {
        op:log("info", "Repairing tube !tubeToRepair");
        act:autonomyRepair(!tubeToRepair);
      }
    } else {
      // report damaged tubes and move on
      act:check(?area); // use NLU part of check action when there are 1+ tubes damaged
    }
    op:log("info", "damagedTubes !damagedTubes");
  } else {
    op:log("info", "No tubes damaged in area ?area");
  }
}

() = autonomyRepair(Symbol ?fullTube) {
  List !tubeParts;
  Symbol !area;
  Symbol !side;
  Symbol !number;

  Boolean !isTubeOff = false;
  long !currTime;
  long !nextMsgTime = 0;
  long !repeatMsgTime = 10000; // milliseconds

  op:log("info", "fullTube ?fullTube");
  !tubeParts = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "fromFullTube", ?fullTube);
  !area = op:get(!tubeParts, 0);
  !side = op:get(!tubeParts, 1);
  !number = op:get(!tubeParts, 2);

  String !messageString = op:bind("Please turn off tube !area !side !number so that it can be repaired");

  act:goToTubeSilent(!area, !side, !number);

  // while tube isn't off, wait
  while (op:equals(!isTubeOff, false)) {

    // observe if tube is broken
    if (obs:propertyOf(tube(!area,!side,!number),broken)) {
      act:generateResponseFromString("Tube !area !side !number is now broken. Resuming monitoring.");
      op:log("info", "Cannot repair !area !side !number because tube is broken (unity)");
      return;
    }

    // observe if tube is damaged
    if (~obs:propertyOf(tube(!area,!side,!number),damaged(X))) {
      op:log("info", "Cannot repair !area !side !number because tube is not damaged (belief,unity)");
      return;
    }

    // observe if tube is off
    if (~obs:propertyOf(tube(!area,!side,!number),off)) {
      !currTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
      if (op:gt(!currTime, !nextMsgTime)) {
        !nextMsgTime = op:invokeStaticMethod("java.lang.Long", "sum", !currTime, !repeatMsgTime);
        act:generateResponseFromString(!messageString);
        op:log("info", "Cannot repair !area !side !number because tube is not off (belief,unity)");
      }
    } else {
      !isTubeOff = op:newObject("java.lang.Boolean", true);
    }
  }

  act:repairTubeAction(!area, !side, !number);
}
