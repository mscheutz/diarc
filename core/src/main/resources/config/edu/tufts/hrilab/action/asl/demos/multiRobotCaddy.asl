import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import java.util.List;

() = setupscene[""]() {

    Symbol !painkillerTypeToken = "painkiller:property";
    Symbol !bandageboxTypeToken = "bandagebox:property";
    Symbol !antisepticTypeToken = "antiseptic:property";
    Symbol !caddyTypeToken = "medicalcaddy:property";
    Symbol !table0Area = "tableA:area";
    Symbol !table1Area = "tableB:area";
    Symbol !table2Area = "tableC:area";
    Symbol !table3Area = "tableD:area";
    Symbol !table4Area = "tableE:area";
    Symbol !table5Area = "tableF:area";
    Symbol !table6Area = "tableG:area";
    Symbol !containerArea= "physobj_0Area:area"; //todo: do this correctly
    Symbol !tmp;

    effects : {
      success infer : constant(!painkillerTypeToken, property);
      success infer : constant(!bandageboxTypeToken, property);
      success infer : constant(!antisepticTypeToken, property);
      success infer : constant(!caddyTypeToken, property);
    }

    List !areas;
    !areas = op:newArrayList("edu.tufts.hrilab.fol.Symbol");
    op:add(!areas, !table0Area);
    op:add(!areas, !table1Area);
    op:add(!areas, !table2Area);
    op:add(!areas, !table3Area);
    op:add(!areas, !table4Area);
    op:add(!areas, !table5Area);
    op:add(!areas, !table6Area);
    op:add(!areas, !containerArea);


    foreach (!areaName : !areas) {
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(amount(!areaName, !caddyTypeToken), 0)");
        act:assertBelief(!tmp);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(amount(!areaName, !bandageboxTypeToken), 0)");
        act:assertBelief(!tmp);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(amount(!areaName, !antisepticTypeToken), 0)");
        act:assertBelief(!tmp);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "fluent_equals(amount(!areaName, !painkillerTypeToken), 0)");
        act:assertBelief(!tmp);
    }

    //setup temi locations
    Symbol !temi = "temi:temi";
    try {
      //todo: the order of these matter for testing in mock to maintain parity with the real temi component.
      //  we have some hardcoded realtional predicates on location references in the prolog file still, so we need
      //  these to be added in this order (and to maintain these names), since the real temi component has references ordered
      //  alphabetically. `alpha` needs to be added first, then `completedAssemblyLocation` (ref 0 and 1 in mock due to order,
      //  and in real due to alph. ordering.
      !temi.act:saveLocation("alpha");
      !temi.act:saveLocation("completedAssemblyLocation");
      !temi.act:saveLocation("gamma");
    } catch () {
      op:log("warn", "!temi saveLocation action is not available.");
    }

    // add spot locations
    Symbol !spot = "spot:spot";
    Symbol !spotInitLocation = "spotlocation_0:spotlocation";
    try {
      !spot.act:initLocation(!spotInitLocation);
      //!spot.act:setMap();
    } catch () {
      op:log("warn", "!spot initLocation action is not available.");
    }
    op:log(info, "done setting up the scene");
}

//area bound to an object (container) => it's a container type.
() = pickupitem["picks up a (packable) ?item at a location and manages fluent property for locations"](Symbol ?actor:fetch, Symbol ?item:physobj, Symbol ?objectType:property, Symbol ?area:area, Symbol ?location:location) {
    conditions : {
        pre : free(?actor);
        pre : actorAt(?actor, ?location);
        pre : areaBoundToLocation(?area, ?location);
        pre infer : objectAt(?item, ?area);
        pre : property_of(?item, ?objectType);
    }
    effects : {
        success : not(free(?actor));
        success infer : not(objectAt(?item, ?area));
        success : carrying(?actor, ?item);
        success infer : fluent_decrease(amount(?area, ?objectType), 1);
    }
    op:log(info, "[pickupitem] ?actor picking up ?item from ?area at ?location");
    act:pickUp(?item);
    act:goToPose(carry);
}

() = putdown["puts down a (packable) ?item at a location and manages fluent property for locations"](Symbol ?actor:fetch, Symbol ?item:physobj, Symbol ?objectType:property, Symbol ?area:area, Symbol ?location:location) {
    conditions : {
        pre : carrying(?actor, ?item);
        pre : areaBoundToLocation(?area, ?location);
        pre : actorAt(?actor, ?location);
        pre : property_of(?item, ?objectType);
        pre : not(containerArea(?area));
    }
    effects : {
        success : free(?actor);
        success infer : objectAt(?item, ?area);
        success infer : fluent_increase(amount(?area, ?objectType), 1);
        success : not(carrying(?actor, ?item));
    }
    op:log(info, "[putdown] ?actor putting down ?item from ?area at ?location");

    act:placeOn(?item,?area);
}


() = putin["puts down a (packable) ?item in a packable object"](Symbol ?actor:fetch, Symbol ?item:physobj, Symbol ?objectType:property, Symbol ?container:physobj, Symbol ?containerArea:area, Symbol ?location:location) {
    conditions : {
        pre : carrying(?actor, ?item);
        pre : areaBoundToLocation(?containerArea, ?location);
        pre : areaBoundToContainer(?containerArea, ?container);
        pre : actorAt(?actor, ?location);
        pre : property_of(?item, ?objectType);
    }
    effects : {
        success : free(?actor);
        success infer : objectAt(?item, ?containerArea);
        success infer : fluent_increase(amount(?containerArea, ?objectType), 1);
        success : not(carrying(?actor, ?item));
    }
    op:log(info, "[putin] ?actor putting ?item in ?containerArea, ?container");

    act:placeIn(?item,?container);
}

//todo: these two goto actions can be consolidated if we either rename the primitive on the temi component, or allow a conditional
//  inside the script to check on the type of the agent. That may not be desirable for perf. assess. reasons. Currently have them separated
//  based on agent type, just to get the primitive call in the body of the action right for the given robot.
() = gotomovebase["the robot moves from ?origin to ?destination"](Symbol ?actor:movebase, Symbol ?origin:location, Symbol ?destination:location, Symbol ?originroom:room, Symbol ?destinationroom:room) {

    conditions : {
        pre : actorAt(?actor, ?origin);
        pre : accessibleBy(?actor, ?destination);
        pre : actorInRoom(?actor, ?originroom);
        pre : locationInRoom(?origin, ?originroom);
        pre : locationInRoom(?destination, ?destinationroom);
        pre : connected(?originroom, ?destinationroom);
        pre : not(sealed(?originroom, ?destinationroom));
    }
    effects : {
        success : not(actorAt(?actor, ?origin));
        success : actorAt(?actor, ?destination);
        success : not(actorInRoom(?actor, ?originroom));
        success : actorInRoom(?actor, ?destinationroom);
    }

    op:log(info, "[goto] ?actor going from ?origin to ?destination");

    //required just because of a mismatch in the primitive names on the temi vs the other robots.
    //todo: should we be using approachLocation, etc in some/all cases for the fetch?
    act:approach(?destination);
}

() = gotospot["the robot moves from ?origin to ?destination"](Symbol ?actor:spot, Symbol ?origin:location, Symbol ?destination:location, Symbol ?originroom:room, Symbol ?destinationroom:room) {

    conditions : {
        pre : actorAt(?actor, ?origin);
        pre : accessibleBy(?actor, ?destination);
        pre : actorInRoom(?actor, ?originroom);
        pre : locationInRoom(?origin, ?originroom);
        pre : locationInRoom(?destination, ?destinationroom);
        pre : connected(?originroom, ?destinationroom);
        pre : not(sealed(?originroom, ?destinationroom));
    }
    effects : {
        success : not(actorAt(?actor, ?origin));
        success : actorAt(?actor, ?destination);
        success : not(actorInRoom(?actor, ?originroom));
        success : actorInRoom(?actor, ?destinationroom);
    }

    op:log(info, "[gotospot] ?actor going from ?origin to ?destination");

    ?actor.act:goToLocation(?destination);
}

() = gototemi["the robot moves from ?origin to ?destination"](Symbol ?actor:temi, Symbol ?origin:location, Symbol ?destination:location, Symbol ?originroom:room, Symbol ?destinationroom:room) {
    java.lang.Boolean !escort = false;

    conditions : {
        pre : actorAt(?actor, ?origin);
        pre : accessibleBy(?actor, ?destination);
        pre : actorInRoom(?actor, ?originroom);
        pre : locationInRoom(?origin, ?originroom);
        pre : locationInRoom(?destination, ?destinationroom);
        pre : connected(?originroom, ?destinationroom);
        pre : not(sealed(?originroom, ?destinationroom));
    }
    effects : {
        success : not(actorAt(?actor, ?origin));
        success : actorAt(?actor, ?destination);
        success : not(actorInRoom(?actor, ?originroom));
        success : actorInRoom(?actor, ?destinationroom);
    }

    op:log(info, "[goto] ?actor going from ?origin to ?destination");

    ?actor.act:goToLoc(?destination, !escort);
}

() = handover["?actor hands over an object to ?receiver from adjacent location ?source to ?destination"](Symbol ?actor:fetch, Symbol ?receiver:temi, Symbol ?source:location, Symbol ?destination:location, Symbol ?item:physobj) {

    java.lang.String !descriptorName;
    edu.tufts.hrilab.fol.Symbol !receiverRef;

    Predicate !property;
    List !propsList;

    conditions : {
        pre : adjacent(?source, ?destination); //todo: should really be allowed to be adjacent(?destination, ?source) as well?
        pre : carrying(?actor, ?item);
        pre : not(equals(?actor, ?receiver));
        pre : actorAt(?actor, ?source);
        pre : actorAt(?receiver, ?destination);
        pre : isContainer(?item);
    }
    effects : {
        success : not(carrying(?actor, ?item));
        success : carrying(?receiver, ?item); //todo: but the temi needs the other agent to grab something off of it. is it really "carrying"?
        success : free(?actor);
        success : not(free(?receiver));
    }
    op:log(info, "[handOver] handing over ?item from ?actor (at ?source) to ?receiver (at ?destination)");

    !descriptorName= op:getName(?receiver);
    !propsList= op:newArrayList("edu.tufts.hrilab.fol.Term");
    !property = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "!descriptorName(X:physobj)");
    op:add(!propsList,!property);
    op:log(debug,"[findTemi] props list: !propsList");

    !receiverRef = act:positReference(!propsList);
    op:log(debug,"[findTemi] posited new reference: !receiverRef");

    act:startVisualSearch(!receiverRef);
    act:getTokenIds(!receiverRef);

    act:moveObjectAbove(?item, !receiverRef, "arm");
    act:moveObjectInDirection(?item, down);
    act:releaseObject(?item);

}

() = receiveitem["the ?actor receives an item ?item handed over by a human while at a location ?location bound to an interactable area ?area containing that item. no perception is performed."](Symbol ?actor:spot, Symbol ?item:physobj, Symbol ?propertyType:property, Symbol ?area:area, Symbol ?location:location) {
    conditions : {
        pre : free(?actor);
        pre : actorAt(?actor, ?location);
        pre : areaBoundToLocation(?area, ?location);
        pre : observableAt(?propertyType, ?area); //becomes observableAt(...,...) & item of correct type
        pre : property_of(?item, ?propertyType);
    }
    effects : {
        success : not(free(?actor));
        success infer : not(objectAt(?item, ?area));
        success : carrying(?actor, ?item);
    }

    Symbol ?self = "self:agent";
//    edu.tufts.hrilab.fol.Term !ackSemantics= "waitForAck(X)";

    op:log(info, "[receiveitem] ?actor picking up ?item from ?area at ?location");
    //how do we handle instantiating the objects that we will want at these locations? do we have the same objectAt preconditions? isn't that an issue since we don't do direct perception with the spot?
    act:moveArmToCarryPosition();
    act:openGripper();
    ?self.act:askQuestionFromString(evan, "Can I please have ?item", val(X));
    act:closeGripper();
    op:sleep(1000);
    act:moveArmOverBody();
}

() = putdownspot["a spot-specific action without perception where the ?spot will place an item it is carrying on an interactable area bound to a specific arm pose."] (Symbol ?actor:spot, Symbol ?item:physobj, Symbol ?objectType:property, Symbol ?area:area, Symbol ?location:location) {
        Symbol !finalLocation = "spotlocation_7:spotlocation";

    conditions : {
        pre : carrying(?actor, ?item);
        pre : areaBoundToLocation(?area, ?location);
        pre : actorAt(?actor, ?location);
        pre : property_of(?item, ?objectType);
    }
    effects : {
        success : free(?actor);
        success infer : objectAt(?item, ?area);
        success infer : fluent_increase(amount(?area, ?objectType), 1);
        success : not(carrying(?actor, ?item));
        success : not(actorAt(?actor, ?location)); //todo: remove. should be domain info for planner. this is to get out of the way of the handover
        success : actorAt(?actor, !finalLocation); //todo: same as above
    }
    op:log(info, "[putdownspot] ?actor putting down ?item from ?area at ?location");
    act:moveArmOverTable();
    act:openGripper();
    act:moveArmOverBody();
    act:closeGripper();
    act:goToLocation(!finalLocation); //todo: see comments in effects. this should be done by the planner with addl domain info.
    act:stowArm();
}

() = deliverkit["?actor delivers an ?item to ?destination"](Symbol ?actor:agent, Symbol ?item:physobj, Symbol ?destination:location) {
    conditions : {
        pre : carrying(?actor, ?item);
        pre : actorAt(?actor, ?destination);
    }
    effects : {
        success : delivered(?item, ?destination);
        success : not(carrying(?actor, ?item)); //todo: this implies that a person has removed the item from the agent who delivered. probably want a separate action for that to be done explicitly?
    }
    op:log(info, "[deliver] ?actor delivering ?item from to ?destination");

    //todo depending on the actor type? or is this only a temi action for now?
}

() = opendoorandmovetolocation["?actor opens the door between ?originRoom and ?destinationRoom and then moves out of the way to a location in ?destinationRoom"](Symbol ?actor:spot, Symbol ?doorLocation:location, Symbol ?destinationLocation:location, Symbol ?originRoom:room, Symbol ?destinationRoom:room) {

    Symbol !insidedoorlocation = "spotlocation_6:spotlocation";

    conditions : {
        pre : actorAt(?actor, ?doorLocation);
        pre : doorBetweenRooms(?doorLocation, ?originRoom, ?destinationRoom);
        pre : not(equals(?originRoom, ?destinationRoom));
        pre : locationInRoom(?doorLocation, ?originRoom);
        pre : locationInRoom(?destinationLocation, ?destinationRoom);
        pre : connected(?originRoom, ?destinationRoom);
    }
    effects : {
        success : not(actorInRoom(?actor, ?originRoom));
        success : actorInRoom(?actor, ?destinationRoom);
        success : not(actorAt(?actor, ?doorLocation));
        success : actorAt(?actor, ?destinationLocation);
        success infer : not(sealed(?originRoom, ?destinationRoom));
        success infer : not(sealed(?destinationRoom, ?originRoom));
    }
    op:log(info, "[opendoor] ?actor opening the door between ?originRoom and ?destinationRoom at ?doorLocation");
    act:detectAndOpenDoor();
    //todo: this is a hack to get the robot localized to the intermediate state inside the door which isn't part of the domain repr right now
    act:initLocation(!insidedoorlocation);
    act:goToLocation(?destinationLocation);
    op:log(info, "[opendoor] ?actor moving out of the doorway to ?destinationLocation in ?destinationRoom");
}




() = perceiveobjectatlocation[""](Symbol ?actor:fetch, Symbol ?objectRef:physobj, Symbol ?location:location, Symbol ?area:area, Symbol ?propertyType:property){


    conditions: {
        pre : property_of(?objectRef,?propertyType);
        pre : observableAt(?propertyType,?area); //TODO:brad: is this in Belief or the consultant?
        pre : actorAt(?actor,?location);
        pre : areaBoundToLocation(?area, ?location);
        pre infer : not(objectAt(?objectRef, ?area));
        pre : not(isContainer(?objectRef));
    }

    effects: {
        success infer : objectAt(?objectRef, ?area);
        success infer : fluent_increase(amount(?area,?propertyType), 1);
    }

    act:perceptionhelper(?objectRef, ?location, ?area, ?propertyType);

}


() = perceivecontaineratlocation[""](Symbol ?actor:fetch, Symbol ?objectRef:physobj, Symbol ?location:location, Symbol ?area:area, Symbol ?propertyType:property, Symbol ?containerArea:area){

    conditions: {
        pre : property_of(?objectRef,?propertyType);
        pre : observableAt(?propertyType,?area); //TODO:brad: is this in Belief or the consultant?
        pre : actorAt(?actor,?location);
        pre : areaBoundToLocation(?area, ?location);
        pre infer : not(objectAt(?objectRef, ?area));
        pre : isContainer(?objectRef);
        pre : areaBoundToContainer(?containerArea, ?objectRef);
    }

    effects: {
        success infer : objectAt(?objectRef, ?area);
        success infer : fluent_increase(amount(?area,?propertyType), 1);
        success : areaBoundToLocation(?containerArea, ?location);
    }

    act:perceptionhelper(?objectRef, ?location, ?area, ?propertyType);

}

() = perceptionhelper["helper action for perception actions"](Symbol ?objectRef, Symbol ?location, Symbol ?area, Symbol ?propertyType) {

    java.lang.Long !typeId =-1;
    List !tokens;
    edu.tufts.hrilab.vision.stm.MemoryObject !memoryObject;
    java.lang.Long !token;
    Predicate !failCond;
    Predicate !observabilityCondition;
    Predicate !thisStep;
    edu.tufts.hrilab.action.justification.Justification !failureJustification;

    !observabilityCondition = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "observableAt", ?propertyType, ?area);

    op:log(info, "perceiveObjectAtLocation");

    !typeId = act:getTypeId(?objectRef);
    if (op:==(!typeId, -1)) {
      op:log("error", "Could not instantiate visual search.");

      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "unknown(?propertyType)");
      exit(FAIL, !failCond);
    }

    !thisStep =  op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "perceiveobjectatlocation(?actor, ?objectRef, ?location, ?area, ?propertyType)");
    !tokens = act:getTokenIds(?objectRef);
    //exit when there are no token ids bound to the reference --- indicating no memory objects bound (and thus no objects found in search matching the reference descriptors)
    if(op:isEmpty(!tokens)){
        //retract observability of this type at this location
        act:retractBelief(!observabilityCondition);
        op:log(info, "[percieveobjectatlocation] retracting !observabilityCondition");

        op:log(error, "[perceiveobjectatlocation] No token ids found for ?objectRef");
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "objectAt(?objectRef,?area)");
        !failureJustification = op:newObject("edu.tufts.hrilab.action.justification.ConditionJustification", false, !failCond);
        op:invokeMethod(!failureJustification, "setStep", !thisStep);
        exit(FAIL_POSTCONDITIONS, !failureJustification);
    }

    op:log(info, "[percieveobjectatlocation] tokenIds for ?objectRef !tokens");
}

(edu.tufts.hrilab.fol.Symbol ?refID) = findTemi() {
    Predicate !property;
    List !propsList;

    !propsList= op:newArrayList("edu.tufts.hrilab.fol.Term");
    !property = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "temi(X:physobj)");
    op:add(!propsList,!property);
    op:log(debug,"[findTemi] props list: !propsList");

    ?refID = act:positReference(!propsList);
    op:log(debug,"[findTemi] posited new reference: ?refID");

    act:startVisualSearch(?refID);
    act:getTokenIds(?refID);
}

() = executeGraspFailurePolicy(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {

  recovery: {
    failureReasons: {grasping(X,Y,Z)}
    actionStatuses: {FAIL_PRECONDITIONS, FAIL_POSTCONDITIONS}
  }

  op:log("info", "executing recovery policy. FailedAction: ?failedActionPredicate. FailureReasons: ?failureReasons. FailedGoal: ?goal");

  // get the failed action's actor
  Symbol !otherActor;
  !otherActor = op:invokeMethod(?failedActionPredicate, "get", 0);
  op:log("info", "got actor from  failed action. Actor: !otherActor");

  // TODO: EAK: change how the AI builds out children contexts during execution.
  //       currently: builds out all children --> executes all children
  //       should: build out next child --> execute next child --> repeat for all children
  // if grasping post-cond failed --> stop visual searches, move arm out of the way, and then re-plan for top-level goal
//  !otherActor.act:stopAllSearches();
//  !otherActor.act:goToPose(carry);
  act:policy(!otherActor);
}

() = policy(Symbol ?otherActor) {
  ?otherActor.act:stopAllSearches();
  ?otherActor.act:goToPose(carry);
}

//
//
////todo: Conceptuals
////    - how are we going to bind properties across consultants. e.g. need to have an object bound to a location, and we
////         will need to determine who (vision consultant?) is responsible for this?
////    - how do we semantically represent locations where multiple robots can interact with objects? (e.g. pose vs. place in previous work)?
////    - how to handle compartments within the caddy?
