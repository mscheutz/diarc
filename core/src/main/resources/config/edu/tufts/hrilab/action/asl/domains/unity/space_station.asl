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

//
// LEGACY ACTIONS FOR TLDL
//

() = goToArea(Symbol ?area) {
  act:goto(?area);
}

() = goToTube(Symbol ?area, Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeNumber", ?tubeCombined);
  act:goto(?area, !side, !number);
}

() = goToTube(Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeNumber", ?tubeCombined);
  act:goto(!side, !number);
}

() = repairTube () {
  act:repair();
}

() = repairTube(Symbol ?area, Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeNumber", ?tubeCombined);
  act:repair(?area, !side, !number);
}

() = repairTube(Symbol ?tubeCombined) {
  Symbol !side = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeSide", ?tubeCombined);
  Symbol !number = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "getTubeNumber", ?tubeCombined);
  act:repair(!side, !number);
}

//
// NEW ACTIONS
//

//["This instructs the robot to go to a tube location. Usage: goto(robot1, area, side, number) -llm"]
() = goto(Symbol ?area, Symbol ?side, Symbol ?number) {

  String !messageString = op:bind("I am going to ?area ?side ?number");
  String !successMessageString = op:bind("Made it to ?area ?side ?number");
  String !failureMessageString = op:bind("Did not make it to ?area ?side ?number");

  act:generateResponseFromString(!messageString);
  op:log("info", "goto: ?area ?side ?number");

  if (act:goToTubeSilent(?area, ?side, ?number)) {
    act:generateResponseFromString(!successMessageString);
    op:log("info", "?actor: Made it to target destination: ?area ?side ?number");
  } else {
    act:generateResponseFromString(!failureMessageString);
    op:log("info", "?actor: Didn't make it to target destination: ?area ?side ?number");
  }
}

//["This instructs the robot to go to a tube if within a location. Usage: goto(robot1, side, number) -llm"]
() = goto(Symbol ?side, Symbol ?number) {
  Symbol !area;

  String !messageString = op:bind("I am going to ?side ?number");
  String !transitMessageString = op:bind("Now going to ?side ?number");
  String !successMessageString = op:bind("Made it to ?side ?number");
  String !failureMessageString = op:bind("Did not make it to ?side ?number");

  Predicate !transitBeliefPredicate;

  List !transitBeliefs;

  conditions : {
    pre infer: amIn(?actor, !area);
  }

  !transitBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,prep(transit,to(?side,?number)))");
  !transitBeliefs = act:queryBelief(!transitBeliefPredicate);

   if (~op:isEmpty(!transitBeliefs)) {
      act:generateResponseFromString(!messageString);
   } else {
      act:generateResponseFromString(!transitMessageString);
   }

  if (act:goToTubeSilent(!area, ?side, ?number)) {
    act:generateResponseFromString(!successMessageString);
    op:log("info", "?actor: Made it to target destination: !area ?side ?number");
  } else {
    act:generateResponseFromString(!failureMessageString);
    op:log("info", "?actor: Didn't make it to target destination: !area ?side ?number");
  }
}

//["This instructs the robot to go to an area location. Usage: goto(robot1, area) -llm"]
() = goto(Symbol ?area) {

  String !messageString = op:bind("I am going to ?area");
  String !transitMessageString = op:bind("Now going to ?area");
  String !successMessageString = op:bind("Made it to ?area");
  String !failureMessageString = op:bind("Did not make it to ?area");

  Predicate !transitBeliefPredicate;

  List !transitBeliefs;

  !transitBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(X)))");
  !transitBeliefs = act:queryBelief(!transitBeliefPredicate);

   if (~op:isEmpty(!transitBeliefs)) {
      act:generateResponseFromString(!messageString);
   } else {
      act:generateResponseFromString(!transitMessageString);
   }

  op:log("info", "goto: ?area");

  if (act:goToAreaSilent(?area)) {
    act:generateResponseFromString(!successMessageString);
    op:log("info", "?actor: Made it to target destination: ?area");
  } else {
    act:generateResponseFromString(!failureMessageString);
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

    !areaLocation = act:getSpaceStationReference(?area);
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
  !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "toFullTube", ?area, ?side, ?number);
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
    !fullTubeLocation = act:getSpaceStationReference(!fullTube);
    ?goToJustification = act:goToLocation(!fullTubeLocation, true);
  }

  act:notInTransit();
}

//["This instructs the robot to go to a tube and to repair it if damaged. Usage: repairTube(robot1, area, side, number) -llm"]
() = repair (Symbol ?area, Symbol ?side, Symbol ?number) {
  String !repairedMessageString = op:bind("Repaired tube ?area ?side ?number");

  op:log("info", "Repair ?area ?side ?number");
  act:goToTubeSilent(?area, ?side, ?number);
  act:repairTubeAction(?area, ?side, ?number);
  act:generateResponseFromString(!repairedMessageString);
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

() = repairTubeAction(Symbol ?area, Symbol ?side, Symbol ?number) {
  Symbol !fullTube;
  Integer !waitTime = 100;
  Boolean !done = false;
  Boolean !isTubeDamaged = true;
  Predicate !repairBeliefPredicate;
  Symbol !damageAmount;
  String !repairingMessageString = op:bind("Repairing tube ?area ?side ?number");

  conditions : {
    pre infer: not(repairing(?actor,X,Y,Z));
    pre infer: amIn(?actor,?area);
    pre infer: amAt(?actor,?side,?number);
    pre obs : not(propertyOf(tube(?area,?side,?number),broken));
    pre obs : propertyOf(tube(?area,?side,?number),damaged(!damageAmount));
    pre obs : propertyOf(tube(?area,?side,?number),off);
  }
  effects : {
    success infer : not(propertyOf(tube(?area,?side,?number),damaged(X)));
  }

  act:generateResponseFromString(!repairingMessageString);

  !fullTube = op:invokeStaticMethod("edu.tufts.hrilab.unity.space_station.TubeNameHelpers", "toFullTube", ?area, ?side, ?number);
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

  op:log("info", "dontKnow: !messageString");
  act:generateResponseFromString(!messageString);
}

() = initializeTrial(Symbol ?initArea, Boolean ?adaptive) {
  effects : {
    success infer: init_area(?initArea);
    success infer: adaptive_autonomy(?adaptive);
  }
  op:log("info", "Initialized ?actor to go to ?initArea with adaptive: ?adaptive");
}


() = resetTrial() {
  effects : {
    success infer: not(amIn(?actor,X));
    success infer: not(amAt(?actor,X,Y));
    success infer: not(lastIn(?actor,X));
    success infer: not(repairing(?actor,X,Y,Z));
    success infer: not(propertyOf(TUBE,damaged(Z)));
    success infer: not(propertyOf(TUBE,off));
    success infer: not(propertyOf(TUBE,broken));
  }
  Symbol !initArea;
  Boolean !adaptive;
  Variable !var = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");

  Predicate !initAreaPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "init_area(X)");
  List !initAreaBeliefs = act:queryBelief(!initAreaPredicate);
  Map !initAreaBelief = op:get(!initAreaBeliefs, 0);
  Symbol !initAreaSymbol = op:get(!initAreaBelief, !var);

  Predicate !adaptivePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "adaptive_autonomy(X)");
  List !adaptiveBeliefs = act:queryBelief(!adaptivePredicate);
  Map !adaptiveBelief = op:get(!adaptiveBeliefs, 0);
  Symbol !adaptiveSymbol = op:get(!adaptiveBelief, !var);

  act:submitGoal("autonomy(?actor, !adaptiveSymbol, !initAreaSymbol)");
  op:log("info", "State reset for ?actor");
}

() = endTrial () {
  List !currentGoals;
  Long !goalID;
  String !goalStr;
  Predicate !goalPred;
  java.lang.Boolean !containsSelfGoal;
  Predicate !workloadBeliefsPredicate;
  Predicate !workloadPlayerOnePredicate;
  Predicate !workloadPlayerTwoPredicate;

  op:log("info", "endTrial(?actor)");

  !workloadBeliefsPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "workload(X,Y)");
  !workloadPlayerOnePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "workload(player1,0)");
  !workloadPlayerTwoPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "workload(player2,0)");
  !currentGoals = tsc:getCurrentGoals(?actor);

  if (op:isEmpty(!currentGoals)) {
    op:log("info", "No goals to cancel for ?actor");
  } else {
    foreach (!currGoal : !currentGoals) {
      !goalStr = op:invokeMethod(!currGoal, "toString");
      !containsSelfGoal = op:invokeMethod(!goalStr, "contains", "endTrial(");
      if (op:==(!containsSelfGoal, true)) {
        op:log("debug", "Skipping goal: !goalStr");
      } else {
        !goalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !goalStr);
        op:log("info", "Cancelling goal: !goalStr");
        !goalID = tsc:getGoalId(!goalPred);
        op:log("debug", "Cancelling goal: !goalID");
        tsc:cancelGoal(!goalID);
      }
    }
  }

  act:retractBelief(!workloadBeliefsPredicate);
  act:assertBelief(!workloadPlayerOnePredicate);
  act:assertBelief(!workloadPlayerTwoPredicate);

  op:log("info", "Trial ended for ?actor");
}

() = notInTransit () {
  effects : {
    success infer: not(amIn(?actor,prep(transit,to(X))));
    success infer: not(amAt(?actor,prep(transit,to(X,Y))));
  }
  op:log("debug", "notInTransit");
}

//["This will check an area for damaged tubes"]
() = check(Symbol ?area) {
  conditions : {
    pre infer: not(repairing(?actor,X,Y,Z));
  }
  op:log("info", "Checking ?area for damaged tubes");

  String !failureMessageString = op:bind("There are no damaged tubes in ?area");
  String !successSingleMessageString = op:bind("There is one damaged tube in ?area");
  String !successMessageString = op:bind("There are damaged tubes in ?area");
  Symbol !tubeToRepair;
  List !damagedTubes;
  int !damagedTubesSize;
  Predicate !currentAreaBeliefPredicate;
  List !currentAreaBeliefs;

  !currentAreaBeliefPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,?area)");
  !currentAreaBeliefs = act:queryBelief(!currentAreaBeliefPredicate);

  if (~op:isEmpty(!currentAreaBeliefs)) {
    op:log("info", "Already in area ?area");
  } else {
    op:log("info", "Not in area ?area, going there");
    act:goToAreaSilent(?area);
  }

  !damagedTubes = act:getSpaceStationTubesDamaged(?area);

  if (~op:isEmpty(!damagedTubes)) {
    !damagedTubesSize = op:invokeMethod(!damagedTubes, "size");
    if (op:==(!damagedTubesSize, 1)) {
      act:generateResponseFromString(!successSingleMessageString);
      op:log("info", "!successSingleMessageString");
    } else {
      act:generateResponseFromString(!successMessageString);
      op:log("info", "!successMessageString");
    }
  } else {
    op:log("info", "!failureMessageString");
    act:generateResponseFromString(!failureMessageString);
  }
}
