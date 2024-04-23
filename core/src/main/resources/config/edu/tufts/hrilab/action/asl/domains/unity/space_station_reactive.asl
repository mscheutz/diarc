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
  op:newArrayList("java.lang.String", !bindings);

  // I grabbed this from the other autonomyAction. Hoping it just makes sure I am not transiting as it seems to.
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
  act:holUpAMinute();

  // TODO: Update status of wing patrolled
  // TODO: In the patrol wing action, have some belief submitted when it finishes being patrolled. Right now this is
  // TODO: just decided by amIn, which is not accurate to finishing patrolling.
  // TODO: See if any off tubes are broken and repair them
  // TODO: Inform human if desired

  // Exit
}

() = waitToRepairAndCoordinate(java.lang.String ?wing, java.lang.String ?brokenTube, java.lang.Integer ?waitTime){
  //Occurs after initiating a repair tube action. Want to catch on the fail to repair case
  //This will have the robot waiting

  edu.tufts.hrilab.fol.Predicate !tubePred;
  java.lang.Boolean !retVal;
  java.util.List !beliefs;
  java.lang.Integer !loopTime = 100;
  java.lang.Integer !timeElapsed = 0;
  java.lang.String !tubeString;

  ?brokenTube =act:capitalizeTubeName(?brokenTube);
  !tubeString = op:invokeMethod(?wing, "concat", ?brokenTube);
  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!tubeString,off)");

  !beliefs = act:queryBelief(!tubePred);
  op:log(info, "before whileloop !beliefs");

  while (op:invokeMethod(!beliefs, "isEmpty") && op:le(!timeElapsed, ?waitTime)) {
    // TODO: Should also add something here checking it is not broken yet
    act:getWhichTubesAreOff();
    !beliefs = act:queryBelief(!tubePred);
    op:log(info, "wait !timeElapsed");

    !timeElapsed = op:invokeStaticMethod("java.lang.Integer", "sum", !loopTime, !timeElapsed);
    op:sleep(!loopTime);
  }

  // This will return to repair tube
}
