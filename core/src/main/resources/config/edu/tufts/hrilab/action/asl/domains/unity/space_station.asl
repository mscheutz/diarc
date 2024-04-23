import java.lang.String;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.action.justification.Justification;

//["Replacement method for edu.tufts.hrilab.util.Util.titleCase() which was removed from current version of Util"]
(String ?capitalized) = capitalizeTubeName (String ?tubeName) {
  String !firstLetter;
  String !restOfString;
  !firstLetter = op:invokeMethod(?tubeName, "charAt", 0);
  !firstLetter = op:invokeMethod(!firstLetter, "toUpperCase");
  !restOfString = op:invokeMethod(?tubeName, "substring", 1);

  ?capitalized = op:invokeMethod(!firstLetter, "concat", !restOfString);
}

//List<Term> damagedTubes, String area, Symbol actor
//(Symbol ?mostDamagedTube) = findMostDamagedTubeHelper() {}

//["asserts or retracts a particular belief to override previous belief"]
() = changeBelief(Predicate ?belief) {
  String !name;
  java.util.List !args;
  String !subject;
  Predicate !pred;
  String !property;
  String !actorStr;
  Predicate !tmp;
  String !semanticType = "direct";
  Symbol !believer;
  java.util.List !bindings;
  java.util.Map !map;

  !name = op:invokeMethod(?belief, "getName");
  !args = op:invokeMethod(?belief, getArgs);
  !tmp = op:invokeMethod(!args, "get", 0);
  !subject = op:invokeMethod(!tmp, "getName");
  op:log("debug", "?actor should believe: ?belief, !name, !subject");
  if (op:invokeMethod(!name, "equals", "amIn")) {
    op:log("debug", "belief topic !ret");
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(!subject,X)");
    op:log("debug", "belief topic: !pred");
  } elseif (op:invokeMethod(!name, "equals", "amAt")) {
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(!subject,X)");
  } elseif (op:invokeMethod(!name, "equals", "monitoring")) {
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "monitoring(!subject,X)");
  } elseif (op:invokeMethod(!name, "equals", "workload")){
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "workload(X)");
  } elseif(op:invokeMethod(!name, "equals", "distraction")){
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "distraction(X)");
  } elseif (op:invokeMethod(!name, "equals", "propertyOf")) {
    !tmp = op:invokeMethod(!args, "get", 1);
    !property = op:invokeMethod(!tmp, "getName");
    if (op:invokeMethod(!property, "equals", "damaged")) {
      !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!subject,damaged(X))");
    }
  } else {
    !pred = op:newObject("edu.tufts.hrilab.fol.Predicate", ?belief);
  }

  !actorStr = op:invokeMethod(?actor, "toString");
  if (op:invokeMethod(!subject, "equals", !actorStr)) {
    if (~act:querySupport(?belief)) {
      !bindings = act:queryBelief(!pred);
      foreach(!map : !bindings) {
        !pred = op:invokeMethod(!pred, "copyWithNewBindings", !map);
      }
      op:log("debug", "fail predicate: !pred");
      exit(FAIL, !pred);
    }
    act:endif();
    op:log("debug", "predicate to retract: !pred");
    op:log("debug", "belief to assert: ?belief");
    act:retractBelief(!pred);
    act:assertBelief(?belief);
  }
}

//["asserts which tubes are off in the current area. "]
() = getWhichTubesAreOff() {
  java.util.List !tubeList;
  java.util.List !predList;
  java.util.List !beliefs;
  Term !tube;
  Predicate !tubePred;
  String !tubeString;
  String !area;
  java.lang.Boolean !inArea;
  java.lang.Boolean !nothingOff;
  Predicate !noTubesOff;

  !noTubesOff = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(noTube,off)");
  act:retractBelief(!noTubesOff);
  !predList = op:newArrayList("java.util.List");
  !tubeList = act:getSpaceStationTubesOff();
  !area = act:getCurrentArea();
  act:updateOffTubesInArea();

  foreach(!tube : !tubeList) {
    !tubeString = op:invokeMethod(!tube, "toString");
    !inArea = op:invokeMethod(!tubeString, "contains", !area);
    if (op:==(!inArea, true)) {
      !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !tubeString);
      op:log("debug", "?actor: tube is off: !tube");
      op:add(!predList, !tubePred);
      act:assertBelief(!tubePred);
    }
  }

  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,off)");
  !beliefs = act:queryBelief(!tubePred);
  if (op:invokeMethod(!beliefs, "isEmpty")) {
      act:assertBelief(!noTubesOff);
      op:log("info", "?actor: no tubes are known to be off");
  }
}

//["updates Belief by retracting off tubes in the current area if they've been switched back on"]
() = updateOffTubesInArea() {
  java.util.List !tubeList;
  Predicate !offPred;
  String !area;
  java.util.List !beliefs;
  Term !tube;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  Symbol !sym;
  String !symString;
  String !tubeString;
  java.lang.Boolean !break = false;
  java.lang.Boolean !inArea;

  !tubeList = act:getSpaceStationTubesOff();
  !area = act:getCurrentArea();
  op:log("debug", "?actor: started updating off tubes in area");
  !offPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,off)");
  !beliefs = act:queryBelief(!offPred);

  foreach(!elem : !beliefs) {
    !sym = op:invokeMethod(!elem, "get", !keyX);
    !symString = op:invokeMethod(!sym, "toString");
    !inArea = op:invokeMethod(!symString, "contains", !area);
    op:log("debug", "?actor: symString is: !symString");
    op:log("debug", "?actor: inArea true or false: !inArea");
    if (op:==(!inArea, true)) {
      !break = op:newObject("java.lang.Boolean", false);
      foreach(!tube : !tubeList) {
        if (op:==(!break, false)) {
          !tubeString = op:invokeMethod(!tube, "toString");
          op:log("debug", "?actor: tubeString is: !tubeString");
          if (op:invokeMethod(!tubeString, "contains", !symString)) {
            !break = op:newObject("java.lang.Boolean", true);
            op:log("debug", "?actor: symString is: !symString, tubeString is: !tubeString");
          }
        }
      }
      if (op:==(!break, false)) {
        !offPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!sym,off)");
        op:log("debug", "?actor: retracting: !offPred");
        act:retractBelief(!offPred);
      }
    }
  }
}

//["updates Belief by retracting damaged tubes in the current area if they've been repaired"]
() = updateDamagedTubesInArea() {
  java.util.List !tubeList;
  Predicate !damagePred;
  String !area;
  java.util.List !beliefs;
  Term !tube;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  Symbol !sym;
  String !symString;
  String !tubeString;
  java.lang.Boolean !break = false;
  java.lang.Boolean !inArea;

  !tubeList = act:getSpaceStationTubesDamaged();
  op:log("info", !tubeList);
  !area = act:getCurrentArea();
  op:log("debug", "?actor: started updating damaged tubes in area");
  !damagePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,damaged(Y))");
  !beliefs = act:queryBelief(!damagePred);
  foreach(!elem : !beliefs) {
    !sym = op:invokeMethod(!elem, "get", !keyX);
    !symString = op:invokeMethod(!sym, "toString");
    !inArea = op:invokeMethod(!symString, "contains", !area);
    op:log("debug", "?actor: symString is: !symString");
    op:log("debug", "?actor: inArea true or false: !inArea");
    if (op:==(!inArea, true)) {
      !break = op:newObject("java.lang.Boolean", false);
      foreach(!tube : !tubeList) {
        if (op:==(!break, false)) {
          !tubeString = op:invokeMethod(!tube, "toString");
          op:log("debug", "?actor: tubeString is: !tubeString");
          if (op:invokeMethod(!tubeString, "contains", !symString)) {
            !break = op:newObject("java.lang.Boolean", true);
            op:log("debug", "?actor: symString is: !symString, tubeString is: !tubeString");
          }
        }
      }
      if (op:==(!break, false)) {
        !damagePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!sym,damaged(Y))");
        op:log("debug", "?actor: retracting: !damagePred");
        act:retractBelief(!damagePred);
      }
    }
  }
}

//["For NLG. Checks if any tubes are known to be damaged. If not, assert a noTubes,damaged(none) predicate"]
() = checkDamagedTubesInBelief() {
  Predicate !noTubesDamaged;
  Predicate !tubePred;
  java.util.List !beliefs;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !sym;
  String !tmp;

  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,damaged(Y))");
  !noTubesDamaged = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(noTube,damaged(noTube))");
  !beliefs = act:queryBelief(!tubePred);

  if (op:invokeMethod(!beliefs, "isEmpty")) {
    act:assertBelief(!noTubesDamaged);
    op:log("info", "?actor: no tubes are known to be damaged");
  }
}

//["Updates Belief with tube damage in current area"]
() = checkDamagedTubesInArea() {
  String !area;
  java.util.List !tubeList;
  Term !tube;
  String !tubeString;
  Symbol !tubeSym;
  Predicate !tubePred;
  Predicate !tubePred2;
  Predicate !damagePred;
  boolean !isDamaged;
  boolean !isBroken;
  String !fullTube;
  java.util.Arrays !tubeArray;
  java.util.List !beliefs;
  Symbol !speaker;
  Predicate !response;
  String !semanticType = "direct";
  java.lang.Boolean !noTubesDamaged = true;

  !area = act:getCurrentArea();
  if (op:invokeMethod(!area, "contains", "transit")) {
    !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tubePred);
  }

  op:log("info", "?actor: Started checking damaged tubes in area");
  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,damaged(noTube))");
  act:retractBelief(!tubePred);
  act:updateOffTubesInArea();
  act:updateDamagedTubesInArea();
  !tubeList = act:getSpaceStationTubesDamaged();

  foreach(!tube : !tubeList) {
    !tubeSym = op:invokeMethod(!tube, "get", 0);
    !fullTube = op:invokeMethod(!tubeSym, "toString");
    op:log("debug", "?actor: tube is: !fullTube");
    !isDamaged = op:invokeMethod(!fullTube, "contains", !area);
    if (op:==(!isDamaged, true)) {
      !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!fullTube,damaged(X))");
      !beliefs = act:queryBelief(!tubePred);
      if (op:invokeMethod(!beliefs, "isEmpty")) {
        !noTubesDamaged = op:newObject("java.lang.Boolean", false);
        act:assertBelief(!tube);
        !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
        !damagePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(noTube,damaged(noTube))");
        if (act:querySupport(!damagePred)) {
          act:retractBelief(!damagePred);
        }
        !tubeString = op:invokeMethod(!tube, "toString");
        !isBroken = op:invokeMethod(!tubeString, "contains", "(100)");
        if (op:==(!isBroken, true)) {
          !tubeSym = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !fullTube);
          !response = op:newObject("edu.tufts.hrilab.fol.Predicate", "isBrokenBeyondRepair", !tubeSym);
          op:log("debug", "?actor: statement is: !response");
          act:generateResponse(!speaker, !response, !semanticType);
        } else {
          !response = op:newObject("edu.tufts.hrilab.fol.Predicate", !tube, !response);
          op:log("debug", "?actor: statement is: !response");
          act:generateResponse(!speaker, !response, !semanticType);
        }
      } else {
        !noTubesDamaged = op:newObject("java.lang.Boolean", false);
        op:log("debug", "?actor: Already in Belief. Time to update.");
        !tubeString = op:invokeMethod(!tube, "toString");
        !isBroken = op:invokeMethod(!tubeString, "contains", "(100)");
        !tubePred2 = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!fullTube,damaged(100))");
        !retVal2 = act:querySupport(!tubePred2);
        if (op:==(!retVal2, false)) {
          act:retractBelief(!tubePred);
          act:assertBelief(!tube);
        }
        if (op:==(!isBroken, true) && op:==(!retVal2, false)) {
          !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
          !tubeSym = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !fullTube);
          !response = op:newObject("edu.tufts.hrilab.fol.Predicate", "hasBrokenBeyondRepair", !tubeSym);
          op:log("debug", "?actor: statement is: !response");
          act:generateResponse(!speaker, !response, !semanticType);
        }
      }
    }
  }

  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,damaged(Y))");
  !retVal = act:querySupport(!tubePred);
  if (op:==(!noTubesDamaged, true) && op:==(!retVal, false)) {
    !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(noTube,damaged(noTube))");
    act:assertBelief(!tubePred);
  }
  act:announcePatrolledArea(!area);
}

//["What is the current workload, using this to simplify a future call"]
(java.lang.Integer ?extracted) = extractIntegerFromHashmap(edu.tufts.hrilab.fol.Variable ?key, java.util.HashMap ?element){
    Symbol !inner2;
    String !inner3;

    op:log("info", "before get 2");
    !inner2 = op:invokeMethod(?element, "get", ?key);
    op:log("info", "after get 2");
    !inner3 = op:invokeMethod(!inner2, "toString");
    ?extracted = op:invokeStaticMethod("java.lang.Integer", "valueOf", !inner3);
}

//["What is the current workload, using this to simplify a future call"]
(java.lang.Integer ?extracted) = extractIntegerFromBelief(edu.tufts.hrilab.fol.Variable ?key, Predicate ?extractPred){
  java.util.List !extractList;
  java.util.HashMap !inner1;

  !extractList = act:queryBelief(?extractPred);
  op:log("info", !extractList);
  if (op:isNull(!extractList) || op:invokeMethod(!extractList, "isEmpty")) {
    // If the belief doesnt exist, still want a return.
    ?extracted = op:newObject("java.lang.Integer", -42);
  } else{
    op:log("info", !extractList);
    //get is a nonstatic method of a list, so we call it from the list rather than the package
    op:log("info", "before get 1");
    !inner1 = op:invokeMethod(!extractList, "get", 0);
    op:log("info", "after get 1");
    ?extracted = act:extractIntegerFromHashmap(?key, !inner1);
  }
}

//["Updates Belief with tube damage in current area and returns list of damaged tubes"]
(java.util.List ?damagedTubesInArea) = checkDamagedTubesInAreaAutonomy() {
  String !area;
  java.util.List !tubeList;
  Term !tube;
  String !tubeString;
  Symbol !tubeSym;
  Predicate !tubePred;
  Predicate !monitoringPred;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  java.lang.Integer !brokenInt;
  boolean !isDamaged;
  boolean !isBroken;
  boolean !retVal;
  boolean !retVal2;
  boolean !inList;
  boolean !inTransit;
  java.lang.Integer !timer = 0;
  String !fullTube;
  java.util.Arrays !tubeArray;
  java.util.List !beliefs;
  Symbol !speaker;
  Predicate !response;
  String !semanticType = "direct";

  !area = act:getCurrentArea();
  !inTransit = op:invokeMethod(!area, "contains", "transit");
  if (op:==(!area, "none") || op:==(!inTransit, true)) {
    !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tubePred);
  }
  act:getWhichTubesAreOff();
  act:updateOffTubesInArea();
  act:updateDamagedTubesInArea();
  !tubeList = act:getSpaceStationTubesDamaged();
  op:log("info", !tubeList);
  ?damagedTubesInArea = op:newArrayList("java.util.List");
  !retVal = op:invokeMethod(!tubeList, "isEmpty");
  op:log("info", "?actor: I am checking damaged tubes in: !area");
  op:log("info", "?actor: damaged tube list is empty?: !retVal");
  !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
  !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "progressive(did(?actor,monitor(!area)))");
  !timer = op:newObject("java.lang.Integer", 0);
  !monitoringPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "monitoring(?actor,!area)");
  act:assertBelief(!monitoringPred);

  while (~op:gt(!timer, 10)) {
    if (op:==(!retVal, false)) {
      foreach(!tube : !tubeList) {
        !tubeSym = op:invokeMethod(!tube, "get", 0);
        !fullTube = op:invokeMethod(!tubeSym, "toString");
        !isDamaged = op:invokeMethod(!fullTube, "contains", !area);
        if (op:==(!isDamaged, true)) {
          !inList = op:invokeMethod(?damagedTubesInArea, "contains", !tube);
          if (op:==(!inList, false)) {
            op:add(?damagedTubesInArea, !tube);
            !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!fullTube,damaged(X))");
            !beliefs = act:queryBelief(!tubePred);
            op:log("info", "Printing from unity asl");
            !brokenInt = act:extractIntegerFromBelief(!keyX, !tubePred);
            if ( op:ge(!brokenInt, 95)){
              op:log("info", "I am removing a broken tube from the list.");
              op:remove(?damagedTubesInArea, !tube);
            }
            op:log("debug",!beliefs);
            if (op:invokeMethod(!beliefs, "isEmpty")) {
              !tubeString = op:invokeMethod(!tube, "toString");
              !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !tubeString);
              act:assertBelief(!tubePred);
            } else {
              act:retractBelief(!tubePred);
              !tubeString = op:invokeMethod(!tube, "toString");
              !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !tubeString);
              act:assertBelief(!tubePred);
              op:log("info", "asserted !tubePred");
            }
          }
        }
      }
    }
    op:++(!timer);
  }
  act:retractBelief(!monitoringPred);
}

//["Queries Belief to get robot's current area"]
(Symbol ?area) = getCurrentArea() {
  edu.tufts.hrilab.fol.Variable !key = X;
  Predicate !areaPred;
  java.util.List !beliefs;
  java.util.Map !binding;

  op:log("debug", "?actor: Getting current area");
  !areaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
  !beliefs = act:queryBelief(!areaPred);
  op:log("debug", "?actor: querying !areaPred gives !beliefs");
  if (op:invokeMethod(!beliefs, "isEmpty")) {
    ?area = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
  } else {
    !binding = op:get(!beliefs, 0);
    ?area = op:get(!binding, !key);
  }
  op:log("debug", "?actor: Current area is ?area");
}

//["Queries Belief to get robot's current tube location (if any)"]
(String ?fullTube) = getCurrentTube() {
  edu.tufts.hrilab.fol.Variable !key = X;
  Predicate !tubePred;
  Symbol !tubeSym;
  java.util.List !beliefs;
  java.util.Map !binding;

  op:log("debug", "?actor: Getting current tube");
  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X)");
  !beliefs = act:queryBelief(!tubePred);
  if (op:invokeMethod(!beliefs, "isEmpty")) {
    ?fullTube = op:newObject("java.lang.String", "none");
  } else {
    !binding = op:get(!beliefs, 0);
    !tubeSym = op:get(!binding, !key);
    ?fullTube = op:invokeMethod(!tubeSym, "toString");
    op:log("debug", "?actor: Current tube is ?fullTube");
  }
}

//["move to ?tubeSideNum in the current area"]
() = goToTube(String ?tubeSideNum) {
  String !currentTube = "none";
  String !tmp;
  String !area;
  String !fullTube = "none";
  Predicate !tubePred;
  Predicate !tmpPred;
  Symbol !tube;

  conditions : {
    pre : not(amAt(?actor,?tubeSideNum));
    pre : not(repairing(?actor,!tube));
  }
  effects : {
    success : amAt(?actor,?tubeSideNum);
  }

  op:log("info", "goToTube with tube side num called");
  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X)");
  act:retractBelief(!tubePred);
  !area = act:getCurrentArea();
  if (op:invokeMethod(!area, "contains", "transit")) {
    !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tubePred);
  }
  if (op:invokeMethod(!area, "contains", "none")) {
    !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tubePred);
  }
  op:log("info", "?tubeSideNum");
  !tmp = act:capitalizeTubeName(?tubeSideNum);
  !fullTube = op:invokeMethod(!area, "concat", !tmp);
  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(!fullTube)))");
  act:assertBelief(!tmpPred);
  act:goToHelper(!fullTube);
}

//["move to tube ?area ?tubeSideNum"]
() = goToTube(String ?area, String ?tubeSideNum) {
  Predicate !tubePred;
  String !prevTube = "none";
  String !fullTube = "none";
  Predicate !tmpPred;
  String !tmp;
  Symbol !tube;

  conditions : {
    or: {
      pre : not(amIn(?actor,?area));
      pre : not(amAt(?actor,?tubeSideNum));
    }
    pre : not(repairing(?actor,!tube));
  }
  effects : {
    success : amIn(?actor,?area);
    success : amAt(?actor,?tubeSideNum);
  }

  op:log("info", "goToTube with area and tube side num called");
  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
  act:retractBelief(!tmpPred);
  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X)");
  act:retractBelief(!tmpPred);
  op:log("info", "tubeSideNum: ?tubeSideNum");
  !tmp = act:capitalizeTubeName(?tubeSideNum);
  !fullTube = op:invokeMethod(?area, "concat", !tmp);
  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(!fullTube)))");
  act:assertBelief(!tmpPred);
  act:retractHasVisitedTubes();
  act:goToHelper(!fullTube);
}

//["move/go to ?area"]
() = goToArea(String ?area) {
  String !currentArea = "none";
  String !currentTube = "none";
  String !nextArea;
  Predicate !tmpPred;
  String !tubeSideNum = "none";
  Symbol !tube;

  op:log("info", "go to area called! ************");

  conditions : {
    pre : not(repairing(?actor,!tube));
  }
  effects : {
    success : amIn(?actor,?area);
  }

  !currentArea = act:getCurrentArea();
  if (op:==(!currentArea, ?area)) {
    !currentTube = act:getCurrentTube();
    if (op:==(!currentTube, "none")) {
      !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,?area)");
      exit(SUCCESS, !tmpPred);
    } else {
      !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
      act:retractBelief(!tmpPred);
      !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(?area)))");
      act:assertBelief(!tmpPred);
      op:log("info", "?actor: 1. Retracting: !tmpPred");
      !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X)");
      act:retractBelief(!tmpPred);
      op:log("info", "?actor: 1. Retracting: !tmpPred");
      act:goToHelper(?area);
    }
  } else {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,X)");
    op:log("info", "?actor: 2. Retracting: !tmpPred");
    act:retractBelief(!tmpPred);
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(?actor,X)");
    op:log("info", "?actor: 2. Retracting: !tmpPred");
    act:retractBelief(!tmpPred);
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(?area)))");
    op:log("info", "?actor: 2. asserting: !tmpPred");
    act:assertBelief(!tmpPred);
    act:retractHasVisitedTubes();
    act:goToHelper(?area);
  }
}

() = retractHasVisitedTubes() {
  java.lang.Boolean !hasVisited = false;
  Predicate !tmpPred;

  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "hasVisited(?actor,X)");
  !hasVisited = act:querySupport(!tmpPred);
  while (op:==(!hasVisited, true)) {
    op:log("info", "?actor: Retracting: !tmpPred");
    act:retractBelief(!tmpPred);
    !hasVisited = act:querySupport(!tmpPred);
  }
}

() = goToHelper(java.lang.String ?location) {
  Predicate !tmpPred;
  java.util.List !beliefs;
  java.util.Map !binding;
  edu.tufts.hrilab.fol.Variable !key = X;
  Symbol !area;
  Justification !goToJustification;

  effects : {
    success : not(amIn(?actor,prep(transit,to(?location))));
    failure : not(amIn(?actor,prep(transit,to(?location))));
  }

  !area = op:newObject("edu.tufts.hrilab.fol.Symbol", ?location);
  op:log("info", "In goToHelper with location of !area");

  !goToJustification = act:goToLocation(!area, true);
  if (op:invokeMethod(!goToJustification, "getValue")) {
    op:log("info", "?actor: Made it to target destination: ?location");
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(X)");
    !beliefs = act:queryBelief(!tmpPred);
    if (~op:invokeMethod(!beliefs, "isEmpty")) {
      act:retractBelief(!tmpPred);
      op:log("debug", "?actor: retracted old belief pred!");
    }
  } else {
    op:log("info", "?actor: Didn't make it to target destination: ?location");
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(?location)))");
    act:retractBelief(!tmpPred);
    exit(CANCEL);
  }
}

//["repair tube ?area ?tubeSideNum"]
() = repairTube(String ?area, String ?tubeSideNum) {
  op:log("info", "repair tube with full parameters called! with area and tube **********");
  Predicate !tubePred;
  Predicate !repairingPred;
  String !fullTube = "none";
  java.util.List !damagedTubes;
  java.util.List !offTubes;
  Term !tube;
  String !tubeString;
  String !tubeOffString;
  java.lang.Boolean !isDamaged = false;
  java.lang.Boolean !isOff = false;
  java.lang.Boolean !isBroken;
  java.lang.Integer !tubeCounter = 0;
  Symbol !speaker;
  Symbol !tubeSym;
  Predicate !response;
  String !semanticType = "direct";
  Symbol !x;
  String !currentArea;
  String !currentTube;
  edu.tufts.hrilab.fol.Predicate !waitPred;
  java.lang.Long !goalID;
  edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
  java.lang.Boolean !equalsIgnoreCase;

  conditions : {
    pre : not(amIn(?actor,prep(transit,to(!x))));
  }
  effects : {
    success : not(propertyOf(!fullTube,damaged(!x)));
    success : not(repairing(?actor,!fullTube));
    failure : not(repairing(?actor,!fullTube));
  }

  !currentArea = act:getCurrentArea();
  op:log("debug", "Current Area: !currentArea, Desired Area: ?area");
  if ( ~op:equals(!currentArea, ?area)) {
    op:log("debug", "nope, going to ?area ?");
    act:goToArea(?area);
  }

  !currentTube = act:getCurrentTube();
  !currentTube = act:capitalizeTubeName(!currentTube);
  op:log("debug", "current tube  !currentTube, desired tube ?tubeSideNum");
  // Here it says current RightNine and desired rightNine
  !equalsIgnoreCase = op:invokeMethod(!currentTube, "equalsIgnoreCase", ?tubeSideNum);
  if (~op:equals(!equalsIgnoreCase, true)) {
    op:log("debug", "going to ?area ?tubeSideNum ?");
    act:goToTube(?tubeSideNum);
    op:log("debug", "done moving");
  }

  ?tubeSideNum = act:capitalizeTubeName(?tubeSideNum);
  !fullTube = op:invokeMethod(?area, "concat", ?tubeSideNum);
  !repairingPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "repairing(?actor,!fullTube)");
  act:assertBelief(!repairingPred);

  act:waitToRepairAndCoordinate(?area, ?tubeSideNum, 7500);
  op:log("debug", "?area + ?tubeSideNum = !fullTube");
  !damagedTubes = act:getSpaceStationTubesDamaged();
  op:log("info", "The returned damaged tubes are: !damagedTubes");
  !offTubes = act:getSpaceStationTubesOff();
  op:log("info", "The returned off tubes are: !offTubes");

  op:log("info", "exited waitToRepairAndCoordinate ");
  foreach(!tube : !damagedTubes) {
    !tubeString = op:invokeMethod(!tube, "toString");
    !isDamaged = op:invokeMethod(!tubeString, "contains", !fullTube);
    op:log("debug", "?actor: fullTube is: !fullTube");
    foreach(!tube : !offTubes) {
      !tubeOffString = op:invokeMethod(!tube, "toString", !tubeOffString);
      op:log("debug", "?actor: tubeOffString is: !tubeOffString");
      if (op:invokeMethod(!tubeOffString, "contains", !fullTube)) {
        !isOff = op:newObject("java.lang.Boolean", true);
        op:log("debug", "?actor: isOff is: !isOff");
      }
    }
    !isBroken = op:invokeMethod(!tubeString, "contains", "(100)");
    if (op:==(!isBroken, true) && op:==(!isDamaged, true)) {
      !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "isBroken(!fullTube)");
      exit(FAIL, !tubePred);
    }
    if (op:==(!isDamaged, true) && op:==(!isBroken, false)) {
      if (op:==(!isOff, false)) {
        !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!fullTube,damaged(X))");
        act:retractBelief(!tubePred);
        !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!fullTube,damaged(100))");
        act:assertBelief(!tubePred);
        !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
        !tubeSym = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !fullTube);
        !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(past(not(is(!fullTube,turnedOff))),is(it,now,broken))");
        act:generateResponse(!speaker, !response, !semanticType);
        !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(repairing(?actor,!fullTube))");
        act:assertBelief(!tubePred);
        exit(FAIL);
      }
      op:log("info", "?actor: Repairing tube !fullTube");
      !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "repairing(?actor,!fullTube)");
      op:++(!tubeCounter);
      act:assertBelief(!tubePred);
      act:startPR2Repair(!fullTube);
      op:log("info", "?actor: Repaired it");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "informbrokentube(?area,!fullTube)");
      act:retractBelief(!response);
    }
  }
  if (op:lt(!tubeCounter, 1)) {
    !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(isDamaged(!fullTube))");
    exit(FAIL, !tubePred);
  }
}

//["repair tube ?tubeSideNum"]
() = repairTube(String ?tubeSideNum) {
  op:log("info",  "repair tube with just the tubenum ?tubeSideNum called **********");
  String !area;
  String !fullTube = "none";
  Predicate !tmpPred;
  Symbol !x;
  String !currentTube;

  effects : {
    success : not(propertyOf(!fullTube,damaged(!x)));
    success : not(repairing(?actor,!fullTube));
    failure : not(repairing(?actor,!fullTube));
  }

  !area = act:getCurrentArea();
  if (op:invokeMethod(!area, "contains", "transit")) {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tmpPred);
  }
  !currentTube = act:getCurrentTube();
  !currentTube = act:capitalizeTubeName(!currentTube);
  op:log("debug", "current tube  !currentTube, desired tube ?tubeSideNum");
  if (~op:equals(!currentTube, ?tubeSideNum)) {
    op:log("info", "going to ?tubeSideNum ?");
    act:goToTube(?tubeSideNum);
    op:log("debug", "done moving");
  }

  ?tubeSideNum = act:capitalizeTubeName(?tubeSideNum);
  !fullTube = op:invokeMethod(!area, "concat", ?tubeSideNum);
  act:repairTube(!area, ?tubeSideNum);
}

//["repair tube that robot is at"]
() = repairTube() {
  op:log("info", "repair tube with no parameters called that robot is at called **********");
  String !area;
  String !currentTube;
  Predicate !tmpPred;

  !currentTube = act:getCurrentTube();
  op:log("debug", "currentTube: !currentTube");
  if (op:==(!currentTube, "none")) {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amAt(?actor,aTube))");
    exit(FAIL, !tmpPred);
  }
  !area = act:getCurrentArea();
  op:log("debug", "area: !area");
  if (op:invokeMethod(!area, "contains", "transit")) {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tmpPred);
  }
  op:log("info", "before specific repairTube");
  act:repairTube(!area, !currentTube);
}

//["get list of current goals and assert it to belief "]
() = getRobotGoals() {
  java.util.List !goals;
  java.util.List !beliefs;
  Predicate !currentGoals;
  Predicate !oldGoals;
  Predicate !tmpPred;
  edu.tufts.hrilab.fol.Variable !tmpVar;
  java.lang.Integer !goalSize;
  Predicate !movingQuery;
  Predicate !repairingQuery;
  Predicate !monitoringQuery;
  Predicate !pred;
  java.util.Map !binding;
  edu.tufts.hrilab.fol.Variable !key = X;

  edu.tufts.hrilab.fol.Symbol !x;

  op:log("debug", "?actor: start get current goals");
  !goals = op:newArrayList("java.util.List");
  !movingQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(X)))");
  !repairingQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "repairing(?actor,X)");
  !monitoringQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "monitoring(?actor,X)");
  !belief = act:queryBelief(!movingQuery);
  if (~op:invokeMethod(!beliefs, "isEmpty")) {
    !binding = op:get(!beliefs, 0);
    !x = op:get(!binding, !key);
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(!x)");
    op:add(!goals, !pred);
  }
  !beliefs = act:queryBelief(!repairingQuery);
  if (~op:invokeMethod(!beliefs, "isEmpty")) {
    !binding = op:get(!beliefs, 0);
    !x = op:get(!binding, !key);
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "startPR2Repair(!x)");
    op:add(!goals, !pred);
  }
  !beliefs = act:queryBelief(!monitoringQuery);
  if (~op:invokeMethod(!beliefs, "isEmpty")) {
    !binding = op:get(!beliefs, 0);
    !x = op:get(!binding, !key);
    !pred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "monitor(!x)");
    op:add(!goals, !pred);
  }
  !tmpVar = op:newObject("edu.tufts.hrilab.fol.Variable", "X");
  !oldGoals = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "currently(is(?actor,!tmpVar))");
  act:retractBelief(!oldGoals);
  op:log("info", "?actor: goals are: !goals");
  !goalSize = op:invokeMethod(!goals, "size");
  if (op:gt(!goalSize, 1)) {
    !tmpPred = op:newObject("edu.tufts.hrilab.fol.Predicate", "and", !goals);
  } elseif (op:isEmpty(!goals)) {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "do(nothing))");
  } else {
    !tmpPred = op:invokeMethod(!goals, "get", 0);
  }
  !currentGoals = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "currently(is(?actor,!tmpPred))");
  act:assertBelief(!currentGoals);
  op:log("info", "?actor: !currentGoals");
  op:log("debug", "?actor: end get current goals");
}

//["monitor tubes in ?area"]
() = monitorArea(String ?area) {
  java.util.List !tubeList;
  Predicate !monitorPred;
  Predicate !tubePred;
  java.lang.Integer !goalSize;
  Symbol !tube;
  java.lang.String !currentArea;

  conditions : {
    pre : not(repairing(?actor,!tube));
  }

  op:log("info", "monitor area called!");
  !currentArea = act:getCurrentArea();
  op:log("info", "are we in the correct area?");
  if ( ~op:equals(!currentArea, ?area)) {
    op:log("info", "nope, going to ?area ?");
    act:goToLocation(?area);
  }

  act:startMonitoring();
  op:log("info", "?actor: Started monitoring: ?area");
  !monitorPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "monitoring(?actor,?area)");
  act:assertBelief(!monitorPred);
  act:filterActiveGoals(!goalSize);
  while (~op:gt(!goalSize, 1)) {
    try {
      act:checkDamagedTubesInArea();
    } catch(FAIL) {
      op:log("debug", "?actor: checkDamagedTubes failed because robot is not in area");
      act:retractBelief(!monitorPred);
      act:stopMonitoring();
      op:log("debug", "?actor: canceling monitor goal because movement command was issued");
      exit(CANCEL);
    }
    act:announcePatrolledArea(?area);
    act:filterActiveGoals(!goalSize);
  }
  act:retractBelief(!monitorPred);
  act:stopMonitoring();
  op:log("debug", "?actor: canceling monitor goal to avoid failure explanation");
  exit(CANCEL);
}

//["call getCurrentGoals and filter for anything that's not a goTo or repair, so that only these can break monitoring"]
(java.lang.Integer ?goalSize) = filterActiveGoals() {
  Predicate !goal;
  java.util.List !goals;
  java.lang.Boolean !isMove = false;
  java.lang.Boolean !isRepair = false;
  java.lang.Boolean !hasActor = false;
  String !goalStr;

  !goals = act:getCurrentGoals();
  op:log("debug", "?actor: goals during monitoring are: !goals");
  foreach(!goal : !goals) {
    op:log("debug", "filterActiveGoals: ?actor: !goalStr");
    !goalStr = op:invokeMethod(!goal, "toString");
    !isMove = op:invokeMethod(!goalStr, "contains", "goTo");
    !isRepair = op:invokeMethod(!goalStr, "contains", "repair");
    if (op:==(!isMove, false) && op:==(!isRepair, false)) {
      op:log("debug", "?actor: removing goal: !goal");
      op:remove(!goals, !goal);
    }
  }
  ?goalSize = op:invokeMethod(!goals, "size");
  op:log("debug", "filterActiveGoals: ?actor: goals after filter are : !goals");
  op:log("debug", "filterActiveGoals: ?actor: goal size after filter is : ?goalSize");
}

//["monitor tubes"]
() = monitorArea() {
  String !currentArea = "none";
  Predicate !tmpPred;

  !currentArea = act:getCurrentArea();
  if (op:invokeMethod(!currentArea, "contains", "transit")) {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(FAIL, !tmpPred);
  }
  act:monitorArea(!currentArea);
}

//["move to nearest damaged tube in the current area (if any)"]
(String ?mostDamagedTube) = moveToMostDamagedTubeAutonomy(java.util.List ?damagedTubesInArea) {
  Symbol !tubeSym;
  String !area;
  Term !tube;
  String !fullTube;
  java.lang.Boolean !isEmpty;

  op:log("info", "?actor: damaged tubes in area: ?damagedTubesInArea");
  !isEmpty = op:invokeMethod(?damagedTubesInArea, "isEmpty", !isEmpty);
  if (op:==(!isEmpty, false)) {
    ?mostDamagedTube = act:findMostDamagedTube(?damagedTubesInArea, ?mostDamagedTube);
    ?mostDamagedTube = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", lowerCaseFirstChar, ?mostDamagedTube);
    op:log("info", "?actor: most damaged tube is: ?mostDamagedTube");
  }
}

//["finds the most damaged tube in the current area (should be sorted by proximity too). Prioritizes off tubes"]
(String ?mostDamagedTubeStr) = findMostDamagedTube(java.util.List ?damagedTubesInArea) {
  String !area;
  Predicate !tmpPred;
  java.lang.Integer !size;
  java.lang.Boolean !keepGoing = True;

  !area = act:getCurrentArea();
  if (op:invokeMethod(!area, "contains", "transit")) {
    !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(amIn(?actor,anArea))");
    exit(CANCEL);
  }
  // This one is in unity.java. So I'll remove from the main list until there is a relevant tube to ask about.
  !size = op:invokeMethod(?damagedTubesInArea, "size");
  while (op:ge(!size, 1) && op:equals(!keepGoing, True)){
    op:log("info", "I am cycling through tubes. I am currently looking at ?mostDamagedTubeStr");
    ?mostDamagedTubeStr = act:findMostDamagedTubeHelper(?damagedTubesInArea, !area);
    !keepGoing = act:isTubeBad(?mostDamagedTubeStr);
    op:remove(?damagedTubesInArea, ?mostDamagedTubeStr);
  }
  // Now, this will try to not give the tube being repaired or at. But will if that's the only option
}

//["This checks if there is already a robot going to, at, or repairing the tube"]
(java.lang.Boolean ?isGood) = isTubeBad(String ?mostDamagedTubeStr){
  //***Return true if bad and false if good
  Predicate !areaPred;
  Predicate !repairPred;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  java.lang.Integer !size;
  java.util.List !beliefs;

  ?isGood = op:newObject("java.lang.Boolean", false);
  !areaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amAt(X,?mostDamagedTubeStr)");
  !beliefs = act:queryBelief(!areaPred);
  !size = op:invokeMethod(!beliefs, "size");
  if ( op:ge(!size, 1)) {
    ?isGood = op:newObject("java.lang.Boolean", true);
  }
  // Not sure if it has the X or not. Gotta double check
  !repairPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "repairing(X, ?mostDamagedTubeStr)");
  !beliefs = act:queryBelief(!repairPred);
  !size = op:invokeMethod(!beliefs, "size");
  if ( op:ge(!size, 1)) {
    ?isGood = op:newObject("java.lang.Boolean", true);
  }

  !repairPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goingTo(X, ?mostDamagedTubeStr)");
  !beliefs = act:queryBelief(!repairPred);
  !size = op:invokeMethod(!beliefs, "size");
  if ( op:ge(!size, 1)) {
    ?isGood = op:newObject("java.lang.Boolean", true);
  }
}

//["Decide the next area based on which two were last patrolled using queryRecency.
//  This works for one or two robots."]
(java.lang.String ?nextArea) = chooseNextAreaAutonomy(){
  Predicate !areaPred;
  Predicate !transitPred;
  Predicate !inPred;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  java.util.List !coveredAreas;
  java.lang.String !searchArea;
  java.lang.Long !areaStartTime;
  java.lang.Long !transitStartTime;
  java.lang.Long !areaEndTime;
  java.lang.Long !transitEndTime;
  java.lang.String !areaStatus;
  java.lang.String !transitStatus;

  java.lang.Long !max = 0.0;
  java.lang.Long !zero = 0.0;
  java.lang.String !oldestPatrolledArea = "None";
  java.lang.String !new_area = "None";
  ?nextArea = op:newObject("java.lang.String", "None");

  !coveredAreas = op:newArrayList("java.lang.String");
  op:add(!coveredAreas, "alpha");
  op:add(!coveredAreas, "gamma");
  op:add(!coveredAreas, "beta");
  !oldestPatrolledArea = op:newObject("java.lang.String", "gamma");

  // Here we will be checking query recency to see which wing hasn't been patrolled longest
  foreach(!searchArea: !coveredAreas){
    // Of note, the autonomy start of each robot needs to be slightly offset for this to work, which is not hard.a
    !areaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "patrolled(!searchArea)");
    !transitPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(!keyX, prep(transit,to(!searchArea)))");
    (!areaStatus, !areaStartTime, !areaEndTime) = act:testRecency(!areaPred);
    (!transitStatus, !transitStartTime, !transitEndTime) = act:testRecency(!transitPred);

    // Go to the area that has never been patrolled first
    if(op:equals(!areaStatus, "None") && op:equals(!transitStatus, "None")){
      ?nextArea = op:newObject("java.lang.String", !searchArea);
    }
    // Go to the area that hasn't been patrolled longest
    if(op:gt(!areaStartTime, !max) && ~op:equals(!transitStatus, "Exists")){
      !max = op:newObject("java.lang.Long", !areaStartTime);
      !oldestPatrolledArea = op:newObject("java.lang.String", !searchArea);
    }
  }

  if (op:equals(?nextArea, "None")) {
    ?nextArea = op:newObject("java.lang.String", !oldestPatrolledArea);
  }
  op:log("debug", "Decided to go to ?nextArea");
}

//["autonomously move to next area based on which one has the most known damaged tubes"]
() = goToNextAreaAutonomy() {
  String !currentArea = "none";
  Predicate !tubePred;
  java.util.List !beliefs;
  java.lang.Boolean !noDamagedTubes = false;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !symX;
  Symbol !symY;
  String !damagedTubeX;
  String !damagedTubeY;
  java.lang.Integer !damagedTubesInAlpha = 0;
  java.lang.Integer !damagedTubesInBeta = 0;
  java.lang.Integer !damagedTubesInGamma = 0;

  act:retractHasVisitedTubes();
  !currentArea = act:getCurrentArea();
  !damagedTubesInAlpha = op:newObject("java.lang.Integer", 0);
  !damagedTubesInBeta = op:newObject("java.lang.Integer", 0);
  !damagedTubesInGamma = op:newObject("java.lang.Integer", 0);
  !tubePred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(X,damaged(Y))");
  !beliefs = act:queryBelief(!tubePred);
  foreach(!elem : !beliefs) {
    !symX = op:invokeMethod(!elem, "get", !keyX);
    !damagedTubeX = op:invokeMethod(!symX, "toString");
    !symY = op:invokeMethod(!elem, "get", !keyY);
    !damagedTubeY = op:invokeMethod(!symY, "getName");
    if (~op:invokeMethod(!damagedTubeY, "equals", 100)) {
      if (op:invokeMethod(!damagedTubeX, "contains", "alpha")) {
        op:++(!damagedTubesInAlpha);
      } elseif (op:invokeMethod(!damagedTubeX, "contains", "beta")) {
        op:++(!damagedTubesInBeta);
      } elseif (op:invokeMethod(!damagedTubeX, "contains", "gamma")) {
        op:++(!damagedTubesInGamma);
      }
    }
  }
  op:log("debug", "?actor: alpha: !damagedTubesInAlpha, beta: !damagedTubesInBeta, gamma: !damagedTubesInGamma");
  if (op:==(!damagedTubesInAlpha, 0) && op:==(!damagedTubesInBeta, 0) && op:==(!damagedTubesInGamma, 0)) {
    !noDamagedTubes = op:newObject("java.lang.Boolean", true);
  }
  if (op:==(!noDamagedTubes, true)) {
    act:goToNextAreaAutonomyHelper(!currentArea, "none");
  } elseif (op:ge(!damagedTubesInAlpha, !damagedTubesInBeta)) {
    if (op:ge(!damagedTubesInAlpha, !damagedTubesInGamma)) {
      act:goToNextAreaAutonomyHelper(!currentArea, "alpha");
    } else {
      act:goToNextAreaAutonomyHelper(!currentArea, "gamma");
    }
  } else {
    if (op:ge(!damagedTubesInBeta, !damagedTubesInGamma)) {
      act:goToNextAreaAutonomyHelper(!currentArea, "beta");
    } else {
      act:goToNextAreaAutonomyHelper(!currentArea, "gamma");
    }
  }
}

//
() = goToNextAreaAutonomyHelper(String ?currentArea, String ?mostDamagedArea) {
  String !actorString;
  String !predString;
  java.lang.Boolean !retVal;
  java.lang.Boolean !robotInAlpha = false;
  java.lang.Boolean !robotInBeta = false;
  java.lang.Boolean !robotInGamma = false;
  Predicate !otherRobotMoving;
  java.lang.Boolean !robotMoving = false;
  java.lang.Boolean !movingToAlpha = false;
  java.lang.Boolean !movingToBeta = false;
  java.lang.Boolean !movingToGamma = false;
  java.lang.Boolean !noOtherRobot = false;
  Predicate !alphaPred;
  Predicate !betaPred;
  Predicate !gammaPred;
  Symbol !speaker;
  Predicate !response;
  String !semanticType = "direct";
  java.util.List !beliefs;
  java.util.Map !binding;
  edu.tufts.hrilab.fol.Variable !key = X;
  Symbol !area;
  String !areaString;

  op:log("debug", "?actor: current area is: ?currentArea");
  op:log("debug", "?actor: most damaged area is: ?mostDamagedArea");
  !actorString = op:invokeMethod(?actor, "toString");
  op:log("debug", "?actor: actor is: !actorString");
  if (op:invokeMethod(!actorString, "contains", "robot1")) {
    !alphaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot2,alpha)");
    !betaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot2,beta)");
    !gammaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot2,gamma)");
    !otherRobotMoving = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot2,prep(transit,to(X)))");
  } elseif (op:invokeMethod(!actorString, "contains", "robot2")) {
    !alphaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot1,alpha)");
    !betaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot1,beta)");
    !gammaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot1,gamma)");
    !otherRobotMoving = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(robot1,prep(transit,to(X)))");
  } else {
    !alphaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "null(null,null)");
    !betaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "null(null,null)");
    !gammaPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "null(null,null)");
    !otherRobotMoving = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "null(null,null)");
  }

  !robotInAlpha = act:querySupport(!alphaPred);
  !robotInBeta = act:querySupport(!betaPred);
  !robotInGamma = act:querySupport(!gammaPred);
  !robotMoving = act:querySupport(!otherRobotMoving);
  if (op:==(!robotInAlpha, false) && op:==(!robotInBeta, false) && op:==(!robotInGamma, false) && op:==(!robotMoving, false)) {
    !noOtherRobot = op:newObject("java.lang.Boolean", true);
  }
  op:log("debug", "?actor: no other robot?: !noOtherRobot");
  if (op:==(!robotMoving, true)) {
    !beliefs = act:queryBelief(!otherRobotMoving);
    !retVal = op:invokeMethod(!beliefs, "isEmpty");
    if (op:==(!retVal, false)) {
      !binding = op:get(!beliefs, 0);
      !area = op:get(!binding, !key);
      !areaString = op:invokeMethod(!area, "toString");
      !movingToAlpha = op:invokeMethod(!areaString, "contains", "alpha");
      !movingToBeta = op:invokeMethod(!areaString, "contains", "beta");
      !movingToGamma = op:invokeMethod(!areaString, "contains", "gamma");
    }
  }
  if (op:==(!movingToAlpha, true)) {
    !robotInAlpha = op:newObject("java.lang.Boolean", true);
  } elseif (op:==(!movingToBeta, true)) {
    !robotInBeta = op:newObject("java.lang.Boolean", true);
  } elseif (op:==(!movingToGamma, true)) {
    !robotInGamma = op:newObject("java.lang.Boolean", true);
  }
  op:log("debug", "?actor: robot in alpha?: !robotInAlpha, robot in beta?: !robotInBeta, robot in gamma?: !robotInGamma, other robot moving?: !robotMoving");
  !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
  if (op:==(?mostDamagedArea, "none") && op:==(!noOtherRobot, true)) {
    if (op:==(?currentArea, "alpha")) {
      act:goToArea("beta"); 
      //was past(did(moveTo(?actor, beta)))
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(beta)");
    } elseif (op:==(?currentArea, "beta")) {
      act:goToArea("gamma");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(gamma)");
    } elseif (op:==(?currentArea, "gamma")) {
      act:goToArea("alpha");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(alpha)");
    }
  } elseif (op:==(?currentArea, "alpha")) {
    if (op:==(!robotInBeta, false)) {
      act:goToArea("beta");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(beta)");
    } else {
      act:goToArea("gamma");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(gamma)");
    }
  } elseif (op:==(?currentArea, "beta")) {
    if (op:==(!robotInGamma, false)) {
      act:goToArea("gamma");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(gamma)");
    } else {
      act:goToArea("alpha");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(alpha)");
    }
  } elseif (op:==(?currentArea, "gamma")) {
    if (op:==(!robotInAlpha, false)) {
      act:goToArea("alpha");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(alpha)");
    } else {
      act:goToArea("beta");
      !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(beta)");
    }
  }
}

//["Autonomy policy"]
() = announceMovementToArea(String ?area) {
  Symbol !speaker;
  Predicate !response;
  String !semanticType = "direct";

  !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
  !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "progressive(moveTo(?area))");
}

//["Autonomy policy announce patrolled area"]
() = announcePatrolledArea(String ?area) {
  Symbol !speaker;
  Predicate !response;
  String !semanticType = "direct";

  !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
  !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "patrolled(?area)");
  act:retractBelief(!response);
  act:assertBelief(!response);
}

//["Autonomy policy inform human tube broke"]
() = informBrokenTube(String ?area, String ?tubeName, edu.tufts.hrilab.fol.Predicate ?tube) {
  Predicate !response;
  java.util.List !bindings;
  java.lang.String !generatedText;

  !bindings = op:newArrayList("java.lang.String");

  // TODO: Make sure to retract belief when tube is fixed.
  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "informbrokentube(?area,?tubeName)");
  act:retractBelief(!response);
  act:assertBelief(!response);

 !generatedText = act:submitNLGRequest(?actor, "brad", ?tube, !bindings);
  act:sayText(!generatedText);
}

//
(java.lang.Boolean ?doIEnd) = autonomyStopConditions(){
  edu.tufts.hrilab.fol.Predicate !queryPred;
  java.util.List !beliefs;
  java.lang.Integer !beliefSize;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  edu.tufts.hrilab.fol.Variable !keyZ = Z;
  java.lang.String !status;
  java.lang.Long !sinceStart;
  java.lang.Long !sinceEnd;

  ?doIEnd = op:newObject("java.lang.Boolean", false);
  op:log("debug", "**After making the bool");

  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "handled(?actor,?actor,want(!keyY,did(?actor,goToArea(!keyX))),direct)");
  op:log("debug", "**After makign the pred");
  (!status, !sinceStart, !sinceEnd) = act:testRecency(!queryPred);
  op:log("debug", "**After testrecency");
  if (op:equals(!status, "Exists") && op:lt(!sinceStart, 2000)){
    op:log("debug", "Handled goToArea");
    ?doIEnd = op:newObject("java.lang.Boolean", true);
  }
  op:log("debug", "**Handled 1");

  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "handled(?actor,?actor,want(!keyY,did(?actor,goToTube(!keyX))),direct)");
  (!status, !sinceStart, !sinceEnd) = act:testRecency(!queryPred);
  if (op:equals(!status, "Exists") && op:lt(!sinceStart, 2000)){
    op:log("debug", "Handled goToTube");
    ?doIEnd = op:newObject("java.lang.Boolean", true);
  }
  op:log("debug", "**Handled 2");
  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "handled(?actor,?actor,want(!keyY,did(?actor,goTo(!keyZ, !keyX))),direct)");
  (!status, !sinceStart, !sinceEnd) = act:testRecency(!queryPred);
  if (op:equals(!status, "Exists") && op:lt(!sinceStart, 2000)){
    op:log("debug", "Handled goTo");
    ?doIEnd = op:newObject("java.lang.Boolean", true);
  }
  op:log("debug", "**Handled 3");
  // Note, autonomy doesnt stop when the robot is repairing. However, it will repair then leave. Seems solid.
}

//["Autonomy policy"]
() = autonomy() {
  java.lang.Boolean !infiniteLoop = true;
  java.lang.Boolean !doIEnd;
  java.lang.Long !waitTime = 2000;

  while (op:==(!infiniteLoop, true)) {
    try {
      op:log("info", "**Before autonomy stop conditions");
      try {
        !doIEnd = act:autonomyStopConditions();
      } catch(FAIL) {
        op:log("info", "The ?actor check for autonomy stop conditions failed, canceling autonomy");
        exit(CANCEL, "Autonomy Overriden");
      }

      if (op:equals(!doIEnd, True)){
        exit(CANCEL, "Autonomy Overriden");
      }
      op:log("info", "**Before new autonomy");
      act:newAutonomy();
      op:log("info", "**After new autonomy");
      act:autonomySleep(2000);
      op:log("info", "**After autonomy sleep");
    } catch(FAIL_PRECONDITIONS) {
      op:log("info", "?actor: preconditions failed");
    } catch(FAIL_OVERALLCONDITIONS) {
      op:log("info", "?actor: overall conditions failed");
      act:fixStuckRobot();
    } catch(FAIL) {
      op:log("error", "?actor: action failed");
    } catch(FAIL_NOTFOUND) {
      op:log("error", "?actor: action failed not found");
    }
  }
}

//["Saves when the robot was last idle, starts autonomy up"]
() = autonomyWatchdog(){
  // What I want to do here is to submit idle when there is nothing going on. Then calc recency on the idle statement
  // It will pull the goals from the actor
  java.lang.Boolean !infiniteLoop = true;
  java.lang.String !status;
  java.lang.Long !sinceStart;
  java.lang.Long !sinceEnd;
  java.lang.Double !difference;
  java.lang.Long !sleepTime;
  Predicate !queryPred;
  java.util.List !goals;
  java.lang.Integer !goalSize;
  Predicate !autonomyPred;
  java.lang.Long !goalID;

  while (op:==(!infiniteLoop, true)) {
    // This will make autonomy start after 10 seconds from the beliefs being inited.
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "idle(?actor)");
    (!status, !sinceStart, !sinceEnd) = act:testRecency(!queryPred);
    //op:log("info", "!status, !sinceStart, !sinceEnd");
    !goals = act:getCurrentGoals();
    //op:log("info", !goals);
    // No action found for event getCurrentGoals !goals
    !goalSize = op:invokeMethod(!goals, "size");
    //op:log("info", "Goals: !goalSize");
    // This watchdog is one of the goals that should always run, so it needs to be strictly greater than
    if (op:equals(1, !goalSize)){
      // If we are in here, we are idle.
      if (op:equals("Retracted", !status) || op:equals("None", !status)){
        op:log("info", "ASSERTING BELIEF !queryPred");
        act:assertBelief(!queryPred);
        // if it's just been asserted, sleep for 6 seconds?
      } else {
        // This should be if it has been at least 6 seconds since the belief was submitted
        if (op:gt(10000, !sinceStart)){
          op:log("info", "RESTARTING AUTONOMY ?actor");
          !autonomyPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "autonomy(?actor)");
          !goalID = act:submitGoal(!autonomyPred);
          act:retractBelief(!queryPred);
        } else {
          // This is if idle exists but has a timer, sleep for that time.
          //op:log("info", "SLEEPING FOR THE IDLE COUNTER ?actor");
          //!difference = op:-(10000, !sinceStart);
          // I can add a caveat here that if difference is below 0, set to 0.
          //!sleepTime = op:invokeMethod(!difference, "longValue");
          //op:log("info", "Sleeping for !sleepTime ms");
          //op:sleep(!sleepTime);
        }
      }
    } else {
      // If there are more goals.
      if (op:equals(!status, "Exists")){
        act:retractBelief(!queryPred);
        //op:log("info", "RETRACTING the IDLE belief since there are !goalSize goals");
        // It's been retracted, something is happening. That something should take a while, sleep for 10 seconds
        //op:log("info", "Sleeping for 10000 ms");
        //op:sleep(10000);
      } else {
        // Something is happening, still not idle, run again in 5 seconds.
        //op:log("info", "Sleeping for 7000 ms");
        //op:sleep(7000);
      }
    }
  }
}

//["A special sleep that will cancel autonomy if movebase is called"]
() = autonomySleep(java.lang.Long ?time){
  java.lang.Boolean !doIEnd;

  !doIEnd = act:assertLock(?time);
  if (op:equals(!doIEnd, False)){
    exit(FAIL, "Autonomy Overriden");
  }
}

//["fixes a bug where the robot loses track of its position"]
() = fixStuckRobot() {
  java.lang.Boolean !retVal2;
  java.lang.Boolean !gotStuck;
  Predicate !tmpPred;
  java.util.List !beliefs;
  java.util.Map !binding;
  edu.tufts.hrilab.fol.Variable !key = X;
  Symbol !area1;
  Symbol !area2;
  String !area;
  String !area1String;
  String !area2String;

  op:log("info", "?actor: fixing stuck robot");
  !area1 = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
  !area2 = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "goToLocation(X)");
  !belifs = act:queryBelief(!tmpPred);
  if (~op:isEmpty(!beliefs)) {
    !binding = op:get(!beliefs, 0);
    !area1 = op:get(!binding, !key, !area1);
  }
  !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(X)))");
  !belifs = act:queryBelief(!tmpPred);

  if (~op:invokeMethod(!beliefs, "isEmpty")) {
    !binding = op:get(!beliefs, 0);
    !area2 = op:get(!binding, !key);
  }

  !area1String = op:invokeMethod(!area1, "toString");
  !area2String = op:invokeMethod(!area2, "toString");
  op:log("info", "?actor: area1 is : !area1String, area2 is : !area2String");

  if (~op:invokeMethod(!area1String, "contains", "none") && ~op:invokeMethod(!area2String, "contains", "none")) {
    !gotStuck = op:invokeMethod(!area1, "equals", !area2);
    if (op:==(!gotStuck, true)) {
      op:log("info", "?actor: got stuck! Fixing it!");
      !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,prep(transit,to(X)))");
      act:retractBelief(!tmpPred);
      !area = act:getArea(?actor);
      if (op:compareString(!area, "transit")) {
        !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,!area2)");
        act:assertBelief(!tmpPred);
      } else {
        !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "amIn(?actor,!area)");
        act:assertBelief(!tmpPred);
      }
    }
  }
}

//["Autonomy policy"]
(java.util.List ?return) = goalObserver(Predicate ?predicate) {
  Predicate !pred;
  String !predString;
  java.lang.Boolean !hasUnitySpeak;
  java.lang.Boolean !hasGetCurrGoals;
  java.lang.Boolean !hasGetRobotGoals;
  java.lang.Boolean !hasAutonomyGoals;
  java.util.List !goalPreds;
  java.lang.Integer !goalSize;
  java.util.Map !bindings;
  String !actorStr;

  observes : num(goals, one);

  !goalPreds = act:getCurrentGoals();
  op:log("debug", "?actor: initial goals: !goalPreds");
  foreach(!pred : !goalPreds) {
    !predString = op:newObject("java.lang.String", "");
    !predString = op:invokeMethod(!pred, "toString");
    !hasUnitySpeak = op:invokeMethod(!predString, "contains", "sayText");
    !hasGetCurrGoals = op:invokeMethod(!predString, "contains", "getCurrentGoals");
    !hasGetRobotGoals = op:invokeMethod(!predString, "contains", "getRobotGoals");
    if (op:==(!hasUnitySpeak, true) || op:==(!hasGetCurrGoals, true) || op:==(!hasGetRobotGoals, true)) {
      op:log("debug", "goalObserver: ?actor: removing goal : !pred");
      op:invokeMethodNoReturn(!goalPreds, "remove", !pred);
    }
  }
  !goalSize = op:invokeMethod(!goalPreds, "size");
  op:log("debug", "goalObserver: ?actor: goals after filter: !goalPreds");
  ?return = op:newArrayList("java.util.Map");
  !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", edu.tufts.hrilab.fol.Symbol);
  if (op:==(!goalSize, 1)) {
    op:add(?return, !bindings);
  }
}

//["waits a set amount of time (used to delay each autonomous action)"]
() = holUpAMinute() {
  java.lang.Integer !timer = 0;
  !timer = op:newObject("java.lang.Integer", 0);
  while (~op:gt(!timer, 20)) {
    op:++(!timer);
    op:log("debug", "?actor: !timer");
  }
}

//
() = informTeammatesOfBrokenTubes(java.lang.String ?area, java.util.List ?damagedTubes){
  java.lang.String !tubeSubstring;
  java.lang.String !damagedSubstring;
  java.lang.Integer !indexOfSplit;
  java.lang.Integer !damagedTubesSize;
  Predicate !tube;
  java.lang.String !tubeString;
  java.lang.String !status;
  java.lang.Long !sinceStart;
  java.lang.Long !sinceEnd;
  Predicate !queryPred;
  java.lang.String !exists = "Exists";
  java.lang.String !mostDamagedTube;
  java.lang.Boolean !true = True;
  // For the tubes that are damaged, aggregate them but say the % for the highest "Tubes aplhaleft4,
  // alphaleft2, and alpha right3 are breaking. Tube aplharight3 is the most broken at X%"

  !damagedTubesSize = op:invokeMethod(?damagedTubes, "size");
  op:log("info", ?damagedTubes);
  op:log("info", "There are !damagedTubesSize damaged tubes in area ?area");
  act:sayText("There are !damagedTubesSize damaged tubes in area ?area");
  !mostDamagedTube = act:findMostDamagedTube(?damagedTubes);
  op:log("info", "The most damaged tube in ?area is !mostDamagedTube");
  !damagedSubstring = act:capitalizeTubeName(!mostDamagedTube);
  op:log("info", !damagedSubstring);

  foreach(!tube : ?damagedTubes) {
    !tubeSubstring = op:invokeMethod(!tube, "toString");
    !tubeString = op:invokeMethod(!tube, "toString");
    !indexOfSplit = op:invokeMethod(!tubeSubstring, indexOf, ",");
    !tubeSubstring = op:invokeMethod(!tubeSubstring, substring, 11, !indexOfSplit);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "informbrokentube(X,!tubeSubstring)");
    (!status, !sinceStart, !sinceEnd) = act:testRecency(!queryPred);
    op:log("info", "[nlg thing] When query on the NLG request with tube, !status, !sinceStart, !sinceEnd");

    op:log("info", !tubeString);
    !true = op:invokeMethod(!tubeString, "contains", !damagedSubstring);
    op:log("info", !true);
    if (op:equals(!true, True) && (op:equals(!status, "None") || (op:equals(!status, !exists) && op:ge(!sinceStart, 10000)))){
      act:informBrokenTube(?area, !tubeSubstring, !tube);
    } else {
      op:log("info", "[nlg thing] Wait a bit");
    }
  }
}

//["body of autonomy script"]
() = autonomyAction() {
  java.util.List !damagedTubes;
  java.util.List !goals;
  java.lang.Integer !goalSize;
  java.lang.Integer !timer = 0;
  java.lang.Boolean !retVal;
  java.lang.Boolean !isOff;
  java.lang.Boolean !didMove;
  java.lang.Boolean !atMostDamagedTube;
  java.lang.Boolean !alreadyScanned = false;
  String !currentTube;
  String !mostDamagedTube;
  String !currentArea;
  String !actorStr;
  Predicate !tmpPred;
  Symbol !tubeSym;
  Symbol !speaker;
  Predicate !response;
  String !semanticType = "direct";
  Predicate !robot1pred;
  Predicate !robot2pred;
  java.lang.Boolean !robot1Autonomy;
  java.lang.Boolean !robot2Autonomy;
  java.lang.Boolean !waited = false;
  java.lang.Boolean !robot2Wait;
  Predicate !robot1AutonomyPred;
  Symbol !x;

  conditions : {
    pre : not(amIn(?actor,prep(transit,to(!x))));
    overall obs : num(goals,one);
  }

  // This block does nothing, it was probably to test holUpAMinute
  !speaker = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "brad");
  !alreadyScanned = op:newObject("java.lang.Boolean", false);
  !actorStr = op:invokeMethod(?actor, "toString");
  if (~op:==(!waited, true)) {
    if (op:invokeMethod(!actorStr, "contains", "robot2")) {
      op:log("warn", "?actor: waiting");
      act:holUpAMinute();
      !waited = op:newObject("java.lang.Boolean", true);
    }
  }

  !goals = act:getCurrentGoals();
  !goalSize = op:invokeMethod(!goals, "size");
  op:log("debug", "?actor: starting autonomy");

  if (op:gt(!goalSize, 1)) {
    op:log("info", "?actor: New goal. Do it then come back. Goals: !goals");
  } else {
    op:log("info", "?actor: 1: autonomy");
    act:holUpAMinute();
    if (op:==(!alreadyScanned, false)) {
      op:log("info", "?actor: started scanning area autonomously");
      !damagedTubes = act:checkDamagedTubesInAreaAutonomy();
      op:log("info", "Going to start gwen's script.");
      act:ActOnTubesNearMe();
      op:log("info", "Gwen's script executed.");
      !alreadyScanned = op:newObject("java.lang.Boolean", true);
      act:holUpAMinute();
    }

    !retVal = op:invokeMethod(!damagedTubes, "isEmpty");
    if (op:==(!retVal, false)) {
      !mostDamagedTube = act:moveToMostDamagedTubeAutonomy(!damagedTubes);
      act:holUpAMinute();
      op:clear(!damagedTubes);
      op:log("info", "?actor: 2: autonomy");
      if (op:==(!mostDamagedTube, none)) {
        op:log("info", "?actor: 3: autonomy");
        act:holUpAMinute();
        act:holUpAMinute();
        act:goToNextAreaAutonomy();
        !alreadyScanned = op:newObject("java.lang.Boolean", false);
      } else {
        !currentTube = act:getCurrentTube();
        !atMostDamagedTube = op:invokeMethod(!currentTube, "contains", !mostDamagedTube);
        op:log("info", "?actor: current tube is: !currentTube");
        op:log("info", "?actor: most damaged tube is: !mostDamagedTube");
        op:log("info", "?actor: at most damaged tube? !atMostDamagedTube");
        try {
          if (op:==(!atMostDamagedTube, false)) {
            act:goToTube(!mostDamagedTube);
            act:holUpAMinute();
          }
        } catch (FAIL_PRECONDITIONS) {
          op:log("info", "?actor: preconditions failed - you were at most damaged tube");
        }
        !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "hasVisited(?actor,!mostDamagedTube)");
        !tmpPred = act:assertBelief();
        op:log("info", "?actor: Asserting hasVisited(!mostDamagedTube) to Belief!");
        act:holUpAMinute();
        !currentTube = act:getCurrentTube();
        !currentArea = act:getCurrentArea();
        !currentTube = act:capitalizeTubeName(!currentTube);
        !currentTube = op:invokeMethod(!currentArea, "concat", !currentTube);
        act:getWhichTubesAreOff();
        !tmpPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "propertyOf(!currentTube,off)");
        !isOff = act:querySupport(!tmpPred);
        !tubeSym = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !currentTube);
        if (op:==(!isOff, true)) {
          !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "perfect(did(?actor,repair(!currentTube)))");
          act:generateResponse(!speaker, !response, !semanticType);
          act:repairTube();
          act:updateDamagedTubesInArea();
          act:updateOffTubesInArea();
          act:holUpAMinute();
        } else {
          !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(did(?actor,repairTube(!currentTube)),because(not(is(!currentTube,turnedOff))))");
          act:generateResponse(!speaker, !response, !semanticType);
        }
      }
    } else {
      op:log("info", "?actor: no damaged tubes in area. Move to next area");
      act:holUpAMinute();
      act:holUpAMinute();
      act:goToNextAreaAutonomy();
      !alreadyScanned = op:newObject("java.lang.Boolean", false);
    }
  }
}

