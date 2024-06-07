import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

// ?actor starts a visual search for ?objectRef in current FOV
(java.lang.Long ?typeId) = startVisualSearch(Symbol ?objectRef) {
    Predicate !failCond;

    ?typeId = act:getTypeId(?objectRef);
    if (op:==(?typeId, -1)) {
      op:log("error", "Could not instantiate visual search.");

      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "unknown(?objectRef)");
      exit(FAIL, !failCond);
    }
}

() = stopVisualSearch(Symbol ?objectRef) {
    java.lang.Long !typeId;
    Predicate !failCond;

    !typeId = act:getTypeId(?objectRef);
    if (op:==(!typeId, -1)) {
      op:log("error", "Could not instantiate visual search.");

      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "unknown(?objectRef)");
      exit(FAIL, !failCond);
    }

    act:stopType(!typeId);
}

(java.util.List ?return) = observeScene(Predicate ?predicate) {

  observes : see(?actor,!x);

  java.util.List !tokenIds;
  java.util.Map !bindings;
  Predicate !anySearch;
  java.util.List !groups;
  java.util.Map !visionRefsMap;
  java.util.Set !refsSet;
  ?return = op:newArrayList("java.util.Map");

  if (op:isVariable(!x)) {
    // look for "any" objects
    !anySearch = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "any(X)");
    act:startVisualSearch(!anySearch);

    // get resulting refs from search(es)
    !groups = op:newArrayList("java.lang.String");
    op:add(!groups, "physobj");
    !visionRefsMap = tsc:getActivatedEntities(!groups);

    // use refs to create binding results (e.g., see(self, physobj_2), see(self, physobj_3))
    !refsSet = op:invokeMethod(!visionRefsMap, "keySet");
    foreach(!ref : !refsSet) {
      !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
      op:put(!bindings, !x, !ref);
      op:add(?return, !bindings);
    }
  } else {
    // look for only specified object
    !tokenIds = act:lookForObject(!x);
    if (~op:isEmpty(!tokenIds)) {
      !bindings = op:newHashMap("edu.tufts.hrilab.fol.Variable", "edu.tufts.hrilab.fol.Symbol");
      op:add(?return, !bindings);
    }
  }
}

// ?actor looks for ?objectRef in current FOV and returns list of ?tokenIds
(java.util.List ?tokenIds) = lookForObject(Symbol ?objectRef) {
    java.lang.Long !tokenId;
    Predicate !resultPred;

    op:log("debug", "[lookForObject] script entered.");
    act:startVisualSearch(?objectRef);
    ?tokenIds = act:getTokenIds(?objectRef);

    // for now, explicitly update belief with resultPred
    !resultPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "see(?actor,?objectRef)");
    if (op:isEmpty(?tokenIds)) {
      op:log("debug", "No object(s) found.");
      act:retractBelief(!resultPred);
    } else {
      op:log("debug", "Found object(s).");
      act:assertBelief(!resultPred);
    }
}

// ?actor looks for object ref ?objectRef in current FOV. This is similar
// to lookForObject except with success/failure based on if object is found.
(java.util.List ?tokenIds, java.lang.Long ?tokenId) = findObject(Symbol ?objectRef) {
    Predicate !failCond;

    effects : {
      failure infer : not(see(?actor,?objectRef));
      success infer : see(?actor,?objectRef);
    }

    ?tokenIds = act:lookForObject(?objectRef);
    if (op:isEmpty(?tokenIds)) {
      op:log("warn", "No objects found. Exiting.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(see(?actor, ?objectRef))");
      exit(FAIL, !failCond);
    } else {
      ?tokenId = op:get(?tokenIds, 0);
    }
}

// ?actor looks for on(grasp_points, ?descriptors) in current FOV.
// This is exactly like findObject, except the ?descriptors are wrapped
// in an on(grasp_points, ?descriptors) predicate so vision will look for
// grasp points on the object.
(java.util.List ?tokenIds, java.lang.Long ?tokenId) = findGraspableObject(Symbol ?objectRef) {
    Predicate !desc;
    Predicate !property;
    java.util.List !properties;


    effects : {
      failure infer : not(see(?actor,?objectRef));
      success infer : see(?actor,?objectRef);
    }

    // first add "on(grasp_point,?objectRef)" property to objectRef
    (!property) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "on(grasp_point(ACTION_VAR0),?objectRef)");
    (!properties) = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    op:add(!properties, !property);
    act:assertProperties(?objectRef, !properties);
    op:log("debug", "Done asserting !properties properties.");

    // then call normal findObject
    (?tokenIds, ?tokenId) = act:findObject(?objectRef);
}
