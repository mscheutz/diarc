import java.lang.String;
import java.lang.Boolean;
import java.lang.Integer;
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

//
// LEGACY ACTIONS FOR TLDL
//

() = goToArea(Symbol ?area) {
  act:goto(?area);
}

() = goToTube(Symbol ?area, Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeNumber", ?tubeCombined);
  act:goto(?area, !side, !number);
}

() = goToTube(Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeNumber", ?tubeCombined);
  act:goto(!side, !number);
}

() = repairTube () {
  act:repair();
}

() = repairTube(Symbol ?area, Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeNumber", ?tubeCombined);
  act:repair(?area, !side, !number);
}

() = repairTube(Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "getTubeNumber", ?tubeCombined);
  act:repair(!side, !number);
}

//
// NEW ACTIONS
//

//["This instructs the robot to go to a tube location. Usage: goto(robot1, area, side, number) -llm"]
() = goto(Symbol ?area, Symbol ?side, Symbol ?number) {
  Symbol !message;

  String !messageString = "I am going to ?area ?side ?number";
  String !successMessageString = "Made it to ?area ?side ?number";
  String !failureMessageString = "Did not make it to ?area ?side ?number";

  !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);

  act:sayText(!message);
  op:log("info", "goto: ?area ?side ?number");

  if (act:goToTubeSilent(?area, ?side, ?number)) {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !successMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Made it to target destination: ?area ?side ?number");
  } else {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !failureMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Didn't make it to target destination: ?area ?side ?number");
  }
}

//["This instructs the robot to go to a tube if within a location. Usage: goto(robot1, side, number) -llm"]
() = goto(Symbol ?side, Symbol ?number) {
  Symbol !area;
  Symbol !message;

  String !messageString = "I am going to ?side ?number";
  String !transitMessageString = "Now going to ?side ?number";
  String !successMessageString = "Made it to ?side ?number";
  String !failureMessageString = "Did not make it to ?side ?number";

  Predicate !transitBeliefPredicate;

  List !transitBeliefs;

  conditions : {
    pre infer: amIn(?actor, !area);
  }

  !transitBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,prep(transit,to(?side,?number)))");
  !transitBeliefs = act:queryBelief(!transitBeliefPredicate);

   if (~op:isEmpty(!transitBeliefs)) {
      !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);
   } else {
      !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !transitMessageString);
   }

  if (act:goToTubeSilent(!area, ?side, ?number)) {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !successMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Made it to target destination: !area");
  } else {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !failureMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Didn't make it to target destination: ?area");
  }
}

//["This instructs the robot to go to an area location. Usage: goto(robot1, area) -llm"]
() = goto(Symbol ?area) {
  Symbol !message;

  String !messageString = "I am going to ?area";
  String !transitMessageString = "Now going to ?area";
  String !successMessageString = "Made it to ?area";
  String !failureMessageString = "Did not make it to ?area";

  Predicate !transitBeliefPredicate;

  List !transitBeliefs;

  !transitBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(X)))");
  !transitBeliefs = act:queryBelief(!transitBeliefPredicate);

   if (~op:isEmpty(!transitBeliefs)) {
      !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);
   } else {
      !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !transitMessageString);
   }

  act:sayText(!message);
  op:log("info", "goto: ?area");

  if (act:goToAreaSilent(?area)) {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !successMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Made it to target destination: ?area");
  } else {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !failureMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Didn't make it to target destination: ?area");
  }
}

//["This instructs the robot to go to an area without any of the hardcoded natural language output"]
(Justification ?goToJustification) = goToAreaSilent(Symbol ?area) {
  Symbol !currentArea;
  Symbol !areaLocation;

  Predicate !currentAreaBeliefPredicate;
  Predicate !transitAreaBeliefPredicate;
  Predicate !anyAreaBeliefPredicate;
  Predicate !anyTubeBeliefPredicate;
  List !currentAreaBeliefs;

  conditions : {
    pre infer: not(repairing(?actor,X,Y,Z));
  }
  effects : {
    success infer: amIn(?actor,?area);
    failure infer: amIn(?actor,X);
    always infer: not(amIn(?actor,prep(transit,to(X))));
  }

  !currentAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,?area)");
  !currentAreaBeliefs = act:queryBelief(!currentAreaBeliefPredicate);

  if (~op:isEmpty(!currentAreaBeliefs)) {
    op:log("info", "Already in area ?area");
  } else {
    op:log("info", "Not in area ?area");

    !anyAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
    !anyTubeBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X,Y)");

    act:retractBelief(!anyAreaBeliefPredicate);
    act:retractBelief(!anyTubeBeliefPredicate);

    !transitAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(?area)))");

    act:assertBelief(!transitAreaBeliefPredicate);

    !areaLocation = act:resolveReferenceLLM(?area);
    ?goToJustification = act:goToLocation(!areaLocation, true);

    act:notInTransit();
  }
}

//["This instructs the robot to go to a particular tube without the hardcoded natural language output"]
(Justification ?goToJustification) = goToTubeSilent(Symbol ?area, Symbol ?side, Symbol ?number) {
  Symbol !fullTube;
  Symbol !fullTubeLocation;
  Predicate !currentAreaBeliefPredicate;
  Predicate !currentTubeBeliefPredicate;
  Predicate !transitTubeBeliefPredicate;
  Predicate !anyTubeBeliefPredicate;
  List !currentAreaBeliefs;
  List !currentTubeBeliefs;

  conditions : {
    pre infer: not(repairing(?actor,X,Y,Z));
  }
  effects : {
    success infer: amAt(?actor,?side,?number);
    failure infer: not(amAt(?actor,X,Y));
    always infer: not(amAt(?actor,prep(transit,to(X,Y))));
  }

  !currentAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,?area)");
  !currentAreaBeliefs = act:queryBelief(!currentAreaBeliefPredicate);

  !anyTubeBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X,Y)");
  act:retractBelief(!anyTubeBeliefPredicate);
  !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "toFullTube", ?area, ?side, ?number);
  !transitTubeBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,prep(transit,to(?side,?number)))");

  if (op:isEmpty(!currentAreaBeliefs)) {
    act:goToAreaSilent(?area);
  } else {
    op:log("info", "Already at area ?area");
  }

  act:notInTransit();

  !currentTubeBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,?side,?number)");
  !currentTubeBeliefs = act:queryBelief(!currentTubeBeliefPredicate);

  act:assertBelief(!transitTubeBeliefPredicate);

  op:log("info", "!currentTubeBeliefs");
  if (~op:isEmpty(!currentTubeBeliefs)) {
    op:log("info", "Already at tube ?side ?number");
  } else {
    op:log("info", "Not at tube ?side ?number ( !fullTube )");
    !fullTubeLocation = act:resolveReferenceLLM(!fullTube);
    ?goToJustification = act:goToLocation(!fullTubeLocation, true);
  }

  act:notInTransit();
}

//["This is an observer action that determines whether a particular tube is broken"]
(List ?return) = checkTubeBroken(Predicate ?predicate) {
    Symbol !area;
    Symbol !side;
    Symbol !number;
    Map !bindings;
    Boolean !isTubeBroken = false;
    Symbol !fullTube;

    op:log("info", "Checking if tube is broken");

    observes : propertyOf(!area,!side,!number,broken);

    ?return = op:newArrayList("java.util.Map");
    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "toFullTube", !area, !side, !number);

    !isTubeBroken = act:getSpaceStationTubeBroken(!fullTube);

    if (op:equals(!isTubeBroken, true)) {
      op:log("info", "Tube is broken");
      op:add(?return, !bindings);
    } else {
      op:log("info", "Tube is not broken");
    }
}

//["This is an observer action that determines whether a particular tube is off"]
(List ?return) = checkTubeOff(Predicate ?predicate) {
    Symbol !area;
    Symbol !side;
    Symbol !number;
    Map !bindings;
    Boolean !isTubeOff = false;
    Symbol !fullTube;

    op:log("info", "Checking if tube is off");

    observes : propertyOf(!area,!side,!number,off);

    ?return = op:newArrayList("java.util.Map");
    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "toFullTube", !area, !side, !number);

    !isTubeOff = act:getSpaceStationTubeOff(!fullTube);

    if (op:equals(!isTubeOff, true)) {
      op:log("info", "Tube !area !side is off");
      op:add(?return, !bindings);
    } else {
      op:log("info", "Tube !area !side !number is not off");
    }
}

//["This is an observer action that determines whether a particular tube is damaged and how much health it has"]
(List ?return) = checkTubeDamaged(Predicate ?predicate) {
    Symbol !area;
    Symbol !side;
    Symbol !number;
    Map !bindings;
    Boolean !isTubeDamaged = false;
    float !tubeHealth;
    Symbol !fullTube;
    Variable !damageAmount;
    Symbol !tubeHealthSymbol;
    Predicate !tubeDamagePredicate;

    op:log("info", "Checking if tube is damaged");

    observes : propertyOf(!area,!side,!number,damaged(!damageAmount));

    ?return = op:newArrayList("java.util.Map");
    !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
    !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "toFullTube", !area, !side, !number);

    !tubeDamagePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,damaged(X))");
    act:retractBelief(!tubeDamagePredicate);

    !isTubeDamaged = act:getSpaceStationTubeDamaged(!fullTube);

    if (op:equals(!isTubeDamaged, true)) {
      !tubeHealth = act:getSpaceStationTubeHealth(!fullTube);
      op:log("info", "Tube !area !side !number is damaged !tubeHealth");
      !tubeHealthSymbol = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!tubeHealth");
      op:put(!bindings, !damageAmount, !tubeHealthSymbol);
      op:add(?return, !bindings);
    } else {
      op:log("info", "Tube !area !side !number is not off");
    }
}

//["This instructs the robot to go to a tube and to repair it if damaged. Usage: repairTube(robot1, area, side, number) -llm"]
() = repair (Symbol ?area, Symbol ?side, Symbol ?number) {
  String !repairingMessageString = "Repairing tube ?area ?side ?number";
  String !repairedMessageString = "Repaired tube ?area ?side ?number";
  Symbol !repairingMessage;
  Symbol !repairedMessage;

  op:log("info", "Repair ?area ?side ?number");

  act:goToTubeSilent(?area, ?side, ?number);
  !repairingMessage = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !repairingMessageString);
  !repairedMessage = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !repairedMessageString);
  act:sayText(!repairingMessage);
  act:repairTubeSilent(?area, ?side, ?number);
  act:sayText(!repairedMessage);
}

() = repair () {
  Symbol !area;
  Symbol !side;
  Symbol !number;

  conditions : {
    pre infer : not(amIn(?actor,prep(transit,to(X))));
    pre infer : not(amAt(?actor,prep(transit,to(X,Y))));
    pre infer : amIn(?actor,!area);
    pre infer : amAt(?actor,!side,!number);
  }

  act:repair(!area, !side, !number);
}

() = repairTubeSilent(Symbol ?area, Symbol ?side, Symbol ?number) {
  Symbol !fullTube;
  Integer !waitTime = 100;
  Boolean !done = false;
  Boolean !isTubeDamaged = true;
  Predicate !repairBeliefPredicate;
  Symbol !damageAmount;

  conditions : {
    pre infer: not(repairing(?actor,X,Y,Z));
    pre infer: amIn(?actor,?area);
    pre infer: amAt(?actor,?side,?number);
    pre obs : not(propertyOf(?area,?side,?number,broken));
    pre obs : propertyOf(?area,?side,?number,damaged(!damageAmount));
    pre obs : propertyOf(?area,?side,?number,off);
  }
  effects : {
    success infer : not(propertyOf(?area,?side,?number,damaged(X)));
  }

  !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM", "toFullTube", ?area, ?side, ?number);
  !repairBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "repairing(?actor,?area,?side,?number)");

  act:startPR2Repair(!fullTube);
  act:assertBelief(!repairBeliefPredicate);

  op:log("info", "Repairing started");

  while (op:equals(!done, false)) {
    !isTubeDamaged = act:getSpaceStationTubeDamaged(!fullTube);
    if (op:equals(!isTubeDamaged, false)) {
      !done = op:equals(!isTubeDamaged, false);
    } else {
      op:sleep(!waitTime);
    }
  }

  act:stopPR2Repair(!fullTube);

  act:retractBelief(!repairBeliefPredicate);
  op:log("info", "Repair complete");
}

//["This is the action to choose if you cannot determine whether to use one of the others. Usage: dontKnow(robot1) -llm"]
() = dontKnow () {
  String !messageString = "I do not understand";
  Symbol !message;

  !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);
  op:log("info", "dontKnow: !messageString");
  act:sayText(!message);
}

() = initializeTrial () {
  effects : {
    success: not(amIn(?actor,X));
    success: not(amAt(?actor,X,Y));
    success: not(lastIn(?actor,X));
    success infer: not(repairing(?actor,X,Y,Z));
    success infer: not(propertyOf(?actor,X,Y,damaged(Z)));
    success infer: not(propertyOf(?actor,X,Y,off));
    success infer: not(propertyOf(?actor,X,Y,broken));
  }
  op:log("info", "State reset for ?actor");
}

/**
 * AUTONOMY BEHAVIORS
 **/

 () = startAutonomy () {
  int !wait = 5000;
  op:sleep(!wait);
  op:log("info", "Starting autonomy...");
  act:autonomy();
 }

//["This is the high level autonomy manager function that will task a robot with monitoring an area if not already busy"]
() = autonomy () {
  int !wait = 10000;
  Predicate !atPredicate;
  Predicate !repairingPredicate;
  Predicate !atTransitPredicate;
  Predicate !inTransitPredicate;
  Predicate !autonomyRepairingTubePredicate;
  List !atBeliefs;
  List !repairingBeliefs;
  List !atTransitBeliefs;
  List !inTransitBeliefs;
  List !autonomyRepairingTubeBeliefs;

  !atPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X,Y)");
  !repairingPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "repairing(?actor,X,Y,Z)");
  !atTransitPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,prep(transit,to(X,Y)))");
  !inTransitPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(X)))");
  !autonomyRepairingTubePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomyRepairing(?actor,X)");

  while (true) {
    !atBeliefs = act:queryBelief(!atPredicate);
    !repairingBeliefs = act:queryBelief(!repairingPredicate);
    !atTransitBeliefs = act:queryBelief(!atTransitPredicate);
    !inTransitBeliefs = act:queryBelief(!inTransitPredicate);
    !autonomyRepairingTubeBeliefs = act:queryBelief(!autonomyRepairingTubePredicate);
    if (~op:isEmpty(!atBeliefs)) {
      op:log("debug", "Not continuing autonomy because ?actor is at tube");
    } elseif (~op:isEmpty(!repairingBeliefs)) {
      op:log("debug", "Not continuing autonomy because ?actor is repairing");
    } elseif (~op:isEmpty(!atTransitBeliefs)) {
      op:log("debug", "Not continuing autonomy because ?actor is in transit to tube");
    } elseif (~op:isEmpty(!inTransitBeliefs)) {
      op:log("debug", "Not continuing autonomy because ?actor is in transit to area");
    } elseif (~op:isEmpty(!autonomyRepairingTubeBeliefs)) {
      act:continueAutonomyRepair();
    } else {
      act:determineAreaToMonitor();
    }
    op:log("debug", "Delaying for !wait");
    op:sleep(!wait);
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
() = determineAreaToMonitor () {
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

  act:monitorArea(!nextArea);
}

() = notInTransit () {
  effects : {
    success infer: not(amIn(?actor,prep(transit,to(X))));
    success infer: not(amAt(?actor,prep(transit,to(X,Y))));
  }
  op:log("debug", "notInTransit");
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
  !tubeParts = op:invokeStaticMethod("edu.tufts.hrilab.llm.LLMListener", "fromFullTube", ?fullTube);
  !area = op:get(!tubeParts, 0);
  !side = op:get(!tubeParts, 1);
  !number = op:get(!tubeParts, 2);

  !autonomyRepairingTubePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomyRepairing(?actor,?fullTube)");
  !tubeBrokenPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,broken)");
  !tubeDamagedPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,damaged(X))");
  !tubeOffPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!area,!side,!number,off)");

  String !messageString = "Please turn off tube !area !side !number so that it can be repaired";

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
      act:sayText(!messageString);
      op:log("info", "Cannot repair !area !side !number because tube is not off (belief,unity)");
      return;
    } else {
      act:assertBelief(!tubeOffPredicate);
    }
  }

  act:repairTubeSilent(!area, !side, !number);
  act:monitor(!area);
}
