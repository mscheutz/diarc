import java.lang.String;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

//["Replacing the autonomyAction, should not goal manage (or state keep?)"]
() = newAutonomy(){
  java.lang.String !wing;
  edu.tufts.hrilab.fol.Predicate !response;
  java.util.List !damagedTubes;
  String !semanticType = "direct";
  String !tubeSideNum;
  java.lang.Integer !damagedTubesSize;
  Symbol !x;
  edu.tufts.hrilab.fol.Predicate !tube;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !sym;
  String !symString;
  java.lang.Integer !brokenInt;
  java.lang.String !mostDamagedTube;
  edu.tufts.hrilab.fol.Predicate !questionPred;
  edu.tufts.hrilab.fol.Predicate !infoPred;
  java.util.List !bindings;
  java.lang.String !generatedText;
  java.lang.String !tubeSubstring;
  java.lang.Integer !indexOfSplit;
  op:newArrayList(java.lang.String, !bindings);

  conditions : {
    pre : not(amIn(?actor,prep(transit,to(!x))));
  }

  // Check area and check when each area was last patrolled
  !wing = act:chooseNextAreaAutonomy();
  op:log(info, "Before move");
  act:announceMovementToArea(!wing);
  act:goToArea(!wing);
  op:log(info, "After move");

  // What to do in a wing. 1) patrol 2) report 3) question
  !damagedTubes = act:checkDamagedTubesInAreaAutonomy();
  op:log(info, "checked damaged tubes");
  act:announcePatrolledArea(!wing);
  !damagedTubesSize = op:invokeMethod(!damagedTubes, "size");
  op:log(info, !damagedTubes);
  op:log(info, "got damaged tubes size: !damagedTubesSize");
  //If there are damaged tubes
  if ( op:ge(!damagedTubesSize, 1)) {
    act:informTeammatesOfBrokenTubes(!wing, !damagedTubes);
    //op:sleep(3000);
    act:autonomySleep(3000);
    act:initiateRepairTubeQuery(!wing, !damagedTubes);
  }
  // Sort by timestamp
  //act:askQuestion(brad, repairTube(!area, ?tubeSideNum));
  act:holUpAMinute();

  // TODO: In the patrol wing action, have some belief submitted when it finishes being patrolled. Right now this is
  // TODO: just decided by amIn, which is not accurate to finishing patrolling.
  // TODO: See if any off tubes are broken and repair them
}

// TODO: Make a function with input damaged tubes that decides what question(s) to ask
() = initiateRepairTubeQuery(java.lang.String ?area, java.util.List ?damagedTubes){
  // Ask to repair most damaged tube.
  java.lang.String !question;
  java.lang.String !mostDamagedTube;
  Predicate !tubePred1;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  java.lang.Integer !brokenInt;
  //java.util.List !beliefs1;
  java.lang.String !tubeString;
  edu.tufts.hrilab.fol.Predicate !waitPred;
  //edu.tufts.hrilab.fol.Predicate !questionPred;
  java.lang.Long !goalID;
  edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
  java.util.List !bindingsList;
  java.util.List !failureList;
  edu.tufts.hrilab.action.justification.Justification !failureConditions;
  java.lang.Boolean !answer;
  edu.tufts.hrilab.fol.Predicate !tubePred;
  java.lang.Boolean !retVal;
  java.util.List !beliefs;

  op:log(info, "Before finding teh most damaged tube");
  !mostDamagedTube = act:findMostDamagedTube(?damagedTubes);
  !tubePred1 = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!mostDamagedTube,damaged(X))");

  op:log(info, "Printing from proactive");
  !brokenInt = act:extractIntegerFromBelief(!keyX, !tubePred1);
  if ( op:le(!brokenInt, 95)) {
    op:log(info, "The return of mostdamagedtube in InitiatRepairTubeQuery: !mostDamagedTube and tubestring of !tubeString");
    !question = op:newObject("java.lang.String", "repairTube(?area, !mostDamagedTube)");
    // Goal justification didnt work. The goal is too far removed. The question didnt fail so it doesn
    // This will ask if the user wants to initiate a repair tube task. If the initial attempt fails, it will wait at the tube
    op:log(info, "Asked to repair !mostDamagedTube in area ?area.");
    !answer = act:askQuestionForAnswer("brad", !question);
    op:log(info, "Got answer from question");
    // This is not getting anything back?
    op:log(info, "it is !answer");

    if ( op:equals(!answer, true)) {
      !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!mostDamagedTube,off)");
      !beliefs = act:queryBelief(!tubePred);
      act:repairTube(!mostDamagedTube);
    }
  }
}

() = aaaTestWording(){
  Predicate !question;
  java.util.List !bindings;
  java.lang.String !generatedResponse;
  java.lang.Boolean !response;

  !bindings = op:newArrayList("java.lang.String");
  !question = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, alphaRightThree)");
  !generatedResponse = act:submitNLGRequest(?actor, "brad", !question, !bindings);
  act:sayText(!generatedResponse);
  act:autonomySleep(4000);

  !question = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "come(brad,to(alphaRightThree))");
  !generatedResponse = act:submitNLGRequest(?actor, brad, !question, !bindings);
  act:sayText(!generatedResponse);

  act:autonomySleep(4000);

  !response = act:askStringForAnswer("brad", "Can you turn off tube alphaRightThree?");
  op:log(info, "!response");

  if (op:equals(!response, false)){
    act:sayText("I will return to patrolling");
  } else{
    act:sayText("I am waiting");
  }
}


() = waitToRepairAndCoordinate(java.lang.String ?wing, java.lang.String ?brokenTube, java.lang.Integer ?waitTime){
  //Occurs after initiating a repair tube action. Want to catch on the fail to repair case
  //This will have the robot waiting

  edu.tufts.hrilab.fol.Predicate !tubePred;
  java.lang.Boolean !retVal;
  java.util.List !beliefs;
  java.lang.Integer !loopTime = 100;
  java.lang.Long !longLoopTime = 100;
  java.lang.Integer !timeElapsed = 0;
  edu.tufts.hrilab.fol.Predicate !innerDid;
  java.util.List !bindings;
  java.lang.String !generatedResponse;
  java.lang.Boolean !response;
  java.lang.String !tubeString;
  java.lang.String !question;
  edu.tufts.hrilab.fol.Symbol !brad = brad;

  op:log(info, "2");
  ?brokenTube = act:capitalizeTubeName(?brokenTube);
  op:log(info, "2");
  !tubeString = op:invokeMethod(?wing, "concat", ?brokenTube);
  op:log(info, "2");
  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!tubeString,off)");
  op:log(info, "2 ?waitTime");
  // This is ruining. Target exception is null?
  //op:invokeStaticMethod(java.lang.Integer, divideUnsigned, ?waitTime, 1.3, ?waitTime);
  op:log(info, "2");

  !beliefs = act:queryBelief(!tubePred);
  op:log(info, " in wait before while loop ");
  op:log(info, " predicate :: !tubePred");
  op:log(info, " beliefs :: !beliefs");
  while (op:invokeMethod(!beliefs, isEmpty) && op:le(!timeElapsed, ?waitTime)) {
    // TODO: Should also add something here checking it is not broken yet
    act:getWhichTubesAreOff();
    !beliefs = act:queryBelief(!tubePred);
    op:log(info, "wait !timeElapsed");
    !timeElapsed = op:invokeStaticMethod("java.lang.Integer", "sum", !loopTime, !timeElapsed);
    act:autonomySleep(!longLoopTime);
  }
  op:log(info, " in wait after passed loop");

  // This one is for if time went out and the tube is still off
  if (op:invokeMethod(!beliefs, "isEmpty")) {
    !bindings = op:newArrayList("java.lang.String");
    op:log(info, "Before asking the string question");
    op:log(info, "Wing ?wing and tube ?brokenTube that match with !tubeString");
    !question = op:newObject("java.lang.String", "Can you turn off tube ?wing ?brokenTube");
    !response = act:askStringForAnswer(!brad, !question);

    if (op:equals(!response, true)){
      act:waitToRepairAndCoordinate(?wing, ?brokenTube, ?waitTime);
    } else {
      op:log(info, "exiting");
      exit(FAIL, not(!tubePred));
    }
  }
}

