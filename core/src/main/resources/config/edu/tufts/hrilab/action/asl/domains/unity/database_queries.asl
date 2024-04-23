import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;
import java.lang.Long;
import java.lang.String;

//["Queries the predicate and returns relevant information from the database"]
(java.lang.String ?status, Long ?sinceStart, Long ?sinceEnd) = testRecency (Predicate ?searchTerm){
  // This only works with exact queries, not variables
  // Term can be a predicate
  //Term !query = "amIn(robot1, alpha)";
  edu.tufts.hrilab.fol.Term !query;

  // Status will be "Retracted", "Exists", or "None" in order of input
  java.util.Map !returnedMap;
  java.util.Set !keySet;
  java.util.List !shell;
  edu.tufts.hrilab.fol.Term !key;
  org.apache.commons.lang3.tuple.Pair !cracked;

  !returnedMap = act:queryRecency(?searchTerm);
  
  if (op:isNull(!returnedMap)){
    ?status = op:newObject("java.lang.String", "None");
    ?sinceStart = op:newObject("java.lang.Long", 0);
    ?sinceEnd = op:newObject("java.lang.Long", 0);
    op:log("debug", "Returned map is null");
  } elseif (op:invokeMethod(!returnedMap, "isEmpty")) {
    ?status = op:newObject("java.lang.String", "None");
    ?sinceStart = op:newObject("java.lang.Long", 0);
    ?sinceEnd = op:newObject("java.lang.Long", 0);
    op:log("debug", "Returned map is empty");
  } else {
    !keySet = op:invokeMethod(!returnedMap, "keySet");
    foreach(!key: !keySet){
      !shell = op:invokeMethod(!returnedMap, "get", !key);
      op:log("debug", "SHELL !shell");
      if (op:invokeMethod(!shell, "isEmpty")) {
        ?status = op:newObject("java.lang.String", "None");
        ?sinceStart = op:newObject("java.lang.Long", 0);
        ?sinceEnd = op:newObject("java.lang.Long", 0);
      } else {
        !cracked = op:invokeMethod(!shell, "get", 0);
        op:log("debug", "CRACKED !cracked");
        ?sinceStart = op:invokeMethod(!cracked, "getLeft");
        ?sinceEnd = op:invokeMethod(!cracked, "getRight");
        op:log("debug", "?sinceStart, ?sinceEnd");
        if (op:le(?sinceEnd, .001)){
          ?status = op:newObject("java.lang.String", "Exists");
        } else{
          ?status = op:newObject("java.lang.String", "Retracted");
        }
      }
    }
  }
}

() = changeDistraction(java.lang.Integer ?newDistraction){
  Predicate !disPred;

  !disPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "distraction(X)");
  act:retractBelief(!disPred);
  !disPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "distraction(?newDistraction)");
  act:assertBelief(!disPred);
}

() = changeAndTest(){
  Predicate !disPred;
  Predicate !specificDis;

  !disPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "distraction(X)");
  !specificDis = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "distraction(1)");
  act:testRecency(!disPred);

  act:assertBelief(!specificDis);
  act:testRecency(!disPred);

  act:wait(1);

  //act:retractBelief(!disPred);
  //act:assertBelief(!specificDis);
  act:changeDistraction(4);
  act:testRecency(!disPred);

  act:wait(1);

  act:retractBelief(!disPred);
  act:testRecency(!disPred);
}
