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
() = autonomy() {
  int !wait = 1000;
  Boolean !workloadLevel;
  List !autonomyRepairingTubeBeliefs;

  Predicate !autonomyRepairingTubePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomyRepairing(?actor,X)");
  while (true) {
    !workloadLevel = act:isWorkloadAboveThreshold();
    !autonomyRepairingTubeBeliefs = act:queryBelief(!autonomyRepairingTubePredicate);

//    if (op:equals(!workloadLevel, false)) {
//      op:log("info", "Not continuing because workload level is below threshold");
//    } else

//    if (~op:isEmpty(!autonomyRepairingTubeBeliefs)) {
//      act:continueAutonomyRepair();
//    } else {
      act:monitorNextArea();
//    }
    op:log("debug", "Delaying for !wait");
    op:sleep(!wait);
  }
}

(Boolean ?workloadLevel) = isWorkloadAboveThreshold () {
  Predicate !workloadPredicate;
  List !workloadBeliefs;
  Symbol !levelSymbol;
  String !levelString;
  Variable !var;
  Map !workloadBelief;

  !workloadPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "workload(X,Y)");
  !workloadBeliefs = act:queryBelief(!workloadPredicate);
  !var = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "Y");

  ?workloadLevel = op:newObject("java.lang.Boolean", false);

  if (op:isEmpty(!workloadBeliefs)) {
    ?workloadLevel = op:newObject("java.lang.Boolean", false);
  } else {
    foreach (!workloadBelief : !workloadBeliefs) {
      !levelSymbol = op:get(!workloadBelief, !var);
      !levelString = op:bind("!levelSymbol");
      if (op:equals(!levelString, "1")) {
        ?workloadLevel = op:newObject("java.lang.Boolean", true);
        return;
      }
    }
  }
}

(Symbol ?nextArea) = getNextArea (Symbol ?lastArea, List ?currentAreaBeliefs, Symbol ?currentArea) {
  List !areas;

  String !areaStr;

  int !index;
  int !size = 2;

  !areas = op:newArrayList("java.lang.String");
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

  !areaStr = op:invokeMethod(?lastArea, "toString");
  !index = op:invokeMethod(!areas, "indexOf", !areaStr);

  if (op:lt(!index, !size)) {
    !index = op:++(!index);
    !areaStr = op:invokeMethod(!areas, "get", !index);
  } else {
    !areaStr = op:invokeMethod(!areas, "get", 0);
  }

  ?nextArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaStr);
}

//["This is the action that will determine the next area to monitor"]
() = monitorNextArea () {
  Symbol !nextArea;
  Symbol !currentArea;
  Symbol !lastArea;

  Variable !keyX = "X";

  Predicate !currentAreaBeliefPredicate;
  Predicate !lastAreaBeliefPredicate;

  List !currentAreaBeliefs;
  List !lastAreaBeliefs;

  Map !currentAreaMap;
  Map !lastAreaMap;

  String !areaStr = "alpha";

  op:log("info", "activeMonitor()");

  !currentAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
  !lastAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "lastMonitored(?actor,X)");

  !currentAreaBeliefs = act:queryBelief(!currentAreaBeliefPredicate);
  !lastAreaBeliefs = act:queryBelief(!lastAreaBeliefPredicate);

  //temp
  !currentArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");

  if (~op:isEmpty(!lastAreaBeliefs)) {
    !lastAreaMap = op:get(!lastAreaBeliefs, 0);
    !lastArea = op:get(!lastAreaMap, !keyX);

    if (~op:isEmpty(!currentAreaBeliefs)) {
      !currentAreaMap = op:get(!currentAreaBeliefs, 0);
      !currentArea = op:get(!currentAreaMap, !keyX);
    }

    !nextArea = act:getNextArea(!lastArea, !currentAreaBeliefs, !currentArea);
  } else {
    //first time only
    !nextArea = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !areaStr);
  }

  act:monitor(!nextArea);
}

//["This will monitor an area and move to all tubes to "]
() = monitor (Symbol ?area) {
  Symbol !tubeToRepair;
  Predicate !lastAreaMonitoredPredicate;
  List !damagedTubes;
  int !damagedTubesSize;

  !lastAreaMonitoredPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "lastMonitored(?actor,?area)");

  act:goToAreaSilent(?area);

  !damagedTubes = act:getSpaceStationTubesDamaged(?area);

  if (~op:isEmpty(!damagedTubes)) {
    !tubeToRepair = op:get(!damagedTubes, 0);
    op:log("info", "Repairing tube !tubeToRepair");
    act:autonomyRepair(!tubeToRepair);
    op:log("info", "damagedTubes !damagedTubes");
  } else {
    op:log("info", "No tubes damaged in area ?area");
    act:assertBelief(!lastAreaMonitoredPredicate);
  }
}

() = continueAutonomyRepair () {
  Symbol !currentTube;

  Predicate !autonomyRepairingTubePredicate;

  Variable !keyX = "X";

  List !autonomyRepairingTubeBeliefs;

  Map !autonomyRepairingTubeMap;

  !autonomyRepairingTubePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomyRepairing(?actor,X)");
  !autonomyRepairingTubeBeliefs = act:queryBelief(!autonomyRepairingTubePredicate);

  if (~op:isEmpty(!autonomyRepairingTubeBeliefs)) {
    !autonomyRepairingTubeMap = op:get(!autonomyRepairingTubeBeliefs, 0);
    !currentTube = op:get(!autonomyRepairingTubeMap, !keyX);
    act:autonomyRepair(!currentTube);
  }
}

() = autonomyRepair (Symbol ?fullTube) {
  Symbol !area;
  Symbol !side;
  Symbol !number;
  Symbol !tubeHealthSymbol;

  Predicate !autonomyRepairingTubePredicate;
  Predicate !tubeBrokenPredicate;
  Predicate !tubeDamagedPredicate;
  Predicate !tubeOffPredicate;

  List !tubeParts;
  List !tubeBrokenBeliefs;
  List !tubeDamagedBeliefs;
  List !tubeOffBeliefs;

  float !tubeHealth;

  Boolean !isTubeBroken = false;
  Boolean !isTubeDamaged = false;
  Boolean !isTubeOff = false;

  op:log("info", "fullTube ?fullTube");
  !tubeParts = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "fromFullTube", ?fullTube);
  !area = op:get(!tubeParts, 0);
  !side = op:get(!tubeParts, 1);
  !number = op:get(!tubeParts, 2);

  !autonomyRepairingTubePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomyRepairing(?actor,?fullTube)");
  !tubeBrokenPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,broken)");
  !tubeDamagedPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,damaged(X))");
  !tubeOffPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,off)");

  String !messageString = op:bind("Please turn off tube !area !side !number so that it can be repaired");

  act:assertBelief(!autonomyRepairingTubePredicate);

  act:goToTubeSilent(!area, !side, !number);

  //is tube broken
  !tubeBrokenBeliefs = act:queryBelief(!tubeBrokenPredicate);
  if (~op:isEmpty(!tubeBrokenBeliefs)) {
    act:retractBelief(!autonomyRepairingTubePredicate);
    op:log("info", "Cannot repair !area !side !number because tube is broken (belief)");
    return;
  }

  !isTubeBroken = act:getSpaceStationTubeBroken(?fullTube);
  if (op:equals(!isTubeBroken, true)) {
    act:assertBelief(!tubeBrokenPredicate);
    act:retractBelief(!autonomyRepairingTubePredicate);
    op:log("info", "Cannot repair !area !side !number because tube is broken (unity)");
    return;
  }

  //is tube damaged
  !tubeDamagedBeliefs = act:queryBelief(!tubeDamagedPredicate);
  if (op:isEmpty(!tubeDamagedBeliefs)) {
    act:retractBelief(!autonomyRepairingTubePredicate);
    !isTubeDamaged = act:getSpaceStationTubeDamaged(?fullTube);
    if (op:equals(!isTubeDamaged, false)) {
      act:retractBelief(!autonomyRepairingTubePredicate);
      op:log("info", "Cannot repair !area !side !number because tube is not damaged (belief,unity)");
      return;
    } else {
      !tubeHealth = act:getSpaceStationTubeHealth(?fullTube);
      op:log("info", "Tube !area !side !number is damaged !tubeHealth");
      !tubeHealthSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!tubeHealth");
      !tubeDamagedPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,damaged(!tubeHealthSymbol))");
      act:assertBelief(!tubeDamagedPredicate);
    }
  }

  //is tube off
  !tubeOffBeliefs = act:queryBelief(!tubeOffPredicate);
  if (op:isEmpty(!tubeOffBeliefs)) {
    !isTubeOff = act:getSpaceStationTubeOff(?fullTube);
    if (op:equals(!isTubeOff, false)) {
      act:retractBelief(!tubeOffPredicate);
      act:generateResponseFromString(!messageString);
      op:log("info", "Cannot repair !area !side !number because tube is not off (belief,unity)");
      return;
    } else {
      act:assertBelief(!tubeOffPredicate);
    }
  }

  act:repairTubeSilent(!area, !side, !number);
  act:monitor(!area);
}
