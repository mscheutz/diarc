import java.lang.String;
import java.lang.Boolean;
import java.lang.Integer;
import java.util.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;

import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.NLPacket;

import edu.tufts.hrilab.action.justification.Justification;

import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;


() = goToTube["This instructs the robot to go to a tube location. Usage: goToTube(robot1, area, side, number) -llm"](Symbol ?area, Symbol ?side, Symbol ?number) {
  Symbol !message;

  String !messageString = "I am going to ?area ?side ?number";
  String !successMessageString = "Made it to ?area ?side ?number";
  String !failureMessageString = "Did not make it to ?area ?side ?number";

  Justification !goToJustification;

  !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);
  
  act:sayText(!message);
  op:log("info", "goToTube: ?area ?side ?number");

  !goToJustification = act:goToTubeSilent(?area, ?side, ?number);

  if (op:invokeMethod(!goToJustification, "getValue")) {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !successMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Made it to target destination: ?area ?side ?number");
  } else {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !failureMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Didn't make it to target destination: ?area ?side ?number");
    exit(CANCEL);
  }
}

(Justification ?goToJustification) = goToTubeSilent(Symbol ?area, Symbol ?side, Symbol ?number) {
  Symbol !location;
  !location = op:invokeStaticMethod("edu.tufts.hrilab.llm.LLMListener", "toLocation", ?area, ?side, ?number);
  ?goToJustification = act:goToLocation(!location, true);
}

() = goToArea["This instructs the robot to go to an area location. Usage: goToArea(robot1, area) -llm"](Symbol ?area) {
  Symbol !message;

  String !messageString = "I am going to ?area";
  String !successMessageString = "Made it to ?area";
  String !failureMessageString = "Did not make it to ?area";

  Justification !goToJustification;

  !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);
  
  act:sayText(!message);
  op:log("info", "goToTube: ?area");

  !goToJustification = act:goToAreaSilent(?area);

  if (op:invokeMethod(!goToJustification, "getValue")) {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !successMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Made it to target destination: ?area");
  } else {
    !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !failureMessageString);
    act:sayText(!message);
    op:log("info", "?actor: Didn't make it to target destination: ?area");
    exit(CANCEL);
  }
}

(Justification ?goToJustification) = goToAreaSilent(Symbol ?area) {
  ?goToJustification = act:goToLocation(?area, true);
}

() = repairTube["This instructs the robot to go to a tube and to repair it if damaged. Usage: repairTube(robot1, area, side, number) -llm"] (Symbol ?area, Symbol ?side, Symbol ?number) {
  String !repairingMessageString = "Repairing tube ?area ?side ?number";
  String !notDamagedMessageString = "Tube ?area ?side ?number is not damaged";
  String !turnOffMessageString = "Turn off tube ?area ?side ?number to repair";
  Symbol !repairingMessage;
  Symbol !notDamagedMessage;
  Symbol !turnOffMessage;
  Boolean !isTubeDamaged = false;
  Boolean !isTubeBroken = false;
  Boolean !isTubeOff = false;
  Symbol !location;

  !location = op:invokeStaticMethod("edu.tufts.hrilab.llm.LLMListener", "toLocation", ?area, ?side, ?number);

  op:log("info", "repairTube: ?area ?side ?number");

  !isTubeBroken = act:getSpaceStationTubeBroken(!location);

  if (op:equals(!isTubeBroken, true)) {
    op:log("info", "Tube is broken: ?area ?side ?number");
    exit(CANCEL);
  }

  !isTubeDamaged = act:getSpaceStationTubeDamaged(!location);

  !turnOffMessage = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !turnOffMessageString);

  op:log("info", "Checking if tube damaged: ?area ?side ?number");
  if (op:equals(!isTubeDamaged, true)) {
    op:log("info", "Tube is damaged: ?area ?side ?number");

    op:log("info", "Checking if tube off: ?area ?side ?number");
    !isTubeOff = act:getSpaceStationTubeOff(!location);
    if (op:equals(!isTubeOff, false)) {
      op:log("info", "Tube is not off: ?area ?side ?number");
      act:sayText(!turnOffMessage);
    } else {
      op:log("info", "Tube is off: ?area ?side ?number");
    }

    act:goToTubeSilent(?area, ?side, ?number);
    op:log("info", "Checking if tube off: ?area ?side ?number");
    !isTubeOff = act:getSpaceStationTubeOff(!location);
    if (op:equals(!isTubeOff, true)) {
      op:log("info", "Tube is off: ?area ?side ?number");
      !repairingMessage = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !repairingMessageString);
      act:sayText(!repairingMessage);
      op:log("info", "Repairing: ?area ?side ?number");
      act:repair(!location);
    } else {
      op:log("info", "Tube is not off: ?area ?side ?number");
      act:sayText(!turnOffMessage);
    }
  } else {
    op:log("info", "Tube is not damaged: ?area ?side ?number");
    !notDamagedMessage = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !notDamagedMessageString);
    act:sayText(!notDamagedMessage);
  }
}

() = repair(Symbol ?location) {
  Integer !waitTime = 100;
  Boolean !done = false;
  Boolean !isDamaged = true;
  act:startPR2Repair(!location);
  while (op:equals(!done, false)) {
    !isTubeDamaged = act:getSpaceStationTubeDamaged(?location);
    if (op:equals(!isTubeDamaged, false)) {
      !done = op:equals(!isTubeDamaged, false);
    } else {
      op:sleep(!waitTime);
    }
  }
  op:log("info", "Repair complete");
}

() = repair["This instructs the robot to repair the tube they are currently at. Usage: repair(robot1) -llm"]() {
  Integer !waitTime = 100;
  Boolean !done = false;
  Boolean !isDamaged = true;

  act:startPR2Repair(!location);
  while (op:equals(!done, false)) {
    !isTubeDamaged = act:getSpaceStationTubeDamaged(?location);
    if (op:equals(!isTubeDamaged, false)) {
      !done = op:equals(!isTubeDamaged, false);
    } else {
      op:sleep(!waitTime);
    }
  }
  op:log("info", "Repair complete");
}

() = dontKnow["This is the action to choose if you cannot determine whether to use one of the others. Usage: dontKnow(robot1) -llm"] () {
  String !messageString = "I do not understand";
  Symbol !message;

  !message = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !messageString);
  op:log("dontKnow: !messageString");
  act:sayText(!message);
}

() = initializeSpaceStation () {
  Prompt !prompt;
  String !message = "Robot, please go to alpha";
  Chat !chat;
  Completion !response;
  
  !prompt = op:invokeStaticMethod("edu.tufts.hrilab.llm.Prompts", "getActionPrompt");
  !chat = op:newObject("edu.tufts.hrilab.llm.Chat", !prompt);
  op:invokeMethod(!chat, "addUserMessage", !message);
  !response = act:chatCompletion(!chat);
  act:toGoal(!response);
}
