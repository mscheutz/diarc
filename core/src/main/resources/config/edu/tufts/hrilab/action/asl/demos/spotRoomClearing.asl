import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;

// Ideally separate config / not hardcoded relative to location refs
() = initRooms() {
    Predicate !tmp;
    Symbol !room1 = main_lab_space;
    Symbol !room2 = experiment_room;
    Symbol !loc1 = location_1:location;
    Symbol !loc2 = location_2:location;
    Symbol !loc3 = location_3:location;

    op:log("debug","initRooms");

    //!room1 = op:createSymbol("main lab space");
    //!room2 = op:createSymbol("experiment room");

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inRoom(!loc1, !room1)");
    act:assertBelief(!tmp);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inRoom(!loc2, !room1)");
    act:assertBelief(!tmp);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "inRoom(!loc3, !room2)");
    act:assertBelief(!tmp);
}

(Symbol ?result) = getBinding(Predicate ?queryPred) {
    Variable !queryKey = X;
    java.util.List !bindings;
    java.util.Map !binding;

    ?result = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");

    !bindings = act:queryBelief(?queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("debug", "[getBinding] query ?queryPred returned no results");
    } else {
        !binding = op:get(!bindings, 0);
        ?result= op:get(!binding, !queryKey);
        op:log("debug", "[getBinding] query ?queryPred result: ?result");
    }
}

(List ?locations) = getLocationsInRoom(Symbol ?room) {
    List !bindings;
    java.util.HashMap !binding;
    Symbol !location;
    Predicate !query;
    Variable !x = X;
    Integer !ind = 0;


    ?room = op:invokeMethod(?room,"toUntyped");

    ?locations = op:newObject("java.util.ArrayList");
    !query = op:createPredicate("inRoom(!x,?room)");
    !bindings = act:queryBelief(!query);
    if (op:isEmpty(!bindings)) {
        op:log(warn, "[getLocationsInRoom] no locations found for room: ?room");
    } else {
        foreach (!binding : !bindings) {
            !location = op:get(!binding, !x);
            //op:add(?locations,!location);
            op:invokeMethod(?locations,"add",!ind,!location);
        }
    }
}

// Specific returnToEntrance / leaveBuilding action with different report string?

(String ?locationName) = getNameForSpotLocationReference(Symbol ?location:location) {
    List !properties;
    Symbol !property;

    !properties = tsc:getProperties(?location);
    !property = op:get(!properties,0);
    ?locationName = op:getName(!property);
}

() = goToLocationWrapper(Symbol ?location:location) {
    onInterrupt :{
        cancel: act:stop();
        suspend: act:stop();
    }
    String !locName = act:getNameForSpotLocationReference(?location);
    //TODO: include room in saytext?
    if (op:equals(!locName,"building entrance")) {
        act:generateResponseFromString(operator,"Heading back to the !locName");
    } else {
        act:generateResponseFromString(operator,"Going to the !locName");
    }
    tsc:goToLocation(?location);
    //For me to test
    op:sleep(10000);

    op:log("debug", "out goToLocationWrapper");
}

// Not sure if still being targeted at all
// (not ideal, remove if have time) Dummy script for LLM to target in mission definition
() = goToRoom(Symbol ?room) {
    //String !roomName;
    op:log("info", "going to ?room");

    //!roomName = op:getName(?room);
    //op:sleep(10000);
}

() = searchRoomForPeople(Symbol ?room) {
    List !locationsInRoom;
    Symbol !currentLoc;
    Predicate !queryPred;
    Symbol !loc;
    String !locName;
    Boolean !found = false;
    Boolean !false = false;

    ?room = op:invokeMethod(?room,"toUntyped");

    op:log("info", "clearing room ?room for civilians");

    //Query belief for waypoints in room
    !locationsInRoom = act:getLocationsInRoom(?room);
    op:log("info", "locations in room: !locationsInRoom");
    //Get current location
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");
    !currentLoc = act:getBinding(!queryPred);
    op:log("info", "currently at !currentLoc");

    if (op:invokeMethod(!locationsInRoom,"contains",!currentLoc)) {
        act:generateResponseFromString(operator,"beginning to clear the ?room");
    } else {
        act:generateResponseFromString(operator,"heading over to clear the ?room");
    }

    foreach (!loc : !locationsInRoom) {
        act:goToLocationWrapper(!loc);
        !locName = act:getNameForSpotLocationReference(!loc);
        act:generateResponseFromString(operator, "scanning !locName of the ?room");
        !found = act:scanForPeople(!false);
        ////(Implied conditional, doesn't need to actually exist)
        ////  if (!found) -> bringToSafety
    }

    if (op:equals(!found,true)) {
        act:generateResponseFromString(operator,"Civilians found in the ?room");
    } else {
        act:generateResponseFromString(operator,"No one found in the ?room");
    }
}

(Boolean ?foundPerson) = scanForPeople(Boolean ?report=true) {
    op:log("info", "scanning for people");

    if (op:equals(?report,true)) {
      act:generateResponseFromString(operator,"Scanning for people in front of me");
    }
    // search for doors
    Predicate !props = op:createPredicate("and(person(X:physobj))");
    Symbol !doorRef = act:positReference(!props);
    ?foundPerson = act:lookAround(!doorRef);

    if (op:equals(?report,true)) {
        if (op:equals(?foundPerson,true)) {
          act:generateResponseFromString(operator,"I see someone in front of me"); // (I see someone in ?currentLocation or ?currentRoom)
        } else {
          act:generateResponseFromString(operator, "I do not see anyone");
        }
    }
}

//Unclear if using something like this OR just using scanForDoor from position when interrupted
() = clearRoomForDoor(Symbol ?room) {
    List !locationsInRoom;
    Symbol !currentLoc;
    Predicate !queryPred;
    Symbol !loc;
    String !locName;
    Boolean !found = false;
    Boolean !false = false;

    ?room = op:invokeMethod(?room,"toUntyped");

    op:log("info", "clearing room ?room for door");
    act:generateResponseFromString(operator,"Let me check that for you");

    //Query belief for waypoints in room
    !locationsInRoom = act:getLocationsInRoom(?room);
    op:log("info", "locations in room: !locationsInRoom");
    //Get current location
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor, X)");
    !currentLoc = act:getBinding(!queryPred);
    op:log("info", "currently at !currentLoc");

    act:generateResponseFromString(operator,"beginning to clear the ?room");
    //if (op:invokeMethod(!locationsInRoom,"contains",!currentLoc)) {
    //    act:generateResponseFromString(operator,"beginning to clear the ?room");
    //} else {
    //    act:generateResponseFromString(operator,"heading over to clear the ?room");
    //}

    foreach (!loc : !locationsInRoom) {
        act:goToLocationWrapper(!loc);
        !locName = act:getNameForSpotLocationReference(!loc);
        act:generateResponseFromString(operator, "scanning !locName of the ?room");
        !found = act:scanForDoor(!false);
        if (op:equals(!found,true)) {
            act:generateResponseFromString(operator,"I see a door in the !locName of the ?room");
            return;
        }
    }

    act:generateResponseFromString(operator,"I did not see any doors in ?room");
}

(Boolean ?foundDoor) = scanForDoor(Boolean ?report=true) {
    op:log("info", "scanning for door");

    // search for doors
    Predicate !props = op:createPredicate("and(door(X:physobj))");
    Symbol !doorRef = act:positReference(!props);
    ?foundDoor = act:lookAround(!doorRef);

    if (op:equals(?report,true)) {
        if (op:equals(?foundDoor,true)) {
          act:generateResponseFromString(operator,"I see a door in front of me");
        } else {
          act:generateResponseFromString(operator, "I do not see any doors");
        }
    }
}

//() = breakOpenDoor(Symbol ?door:physobj) {
() = breakOpenDoor() {
    op:log("info", "breaking open door");

    // search for doors
    Predicate !props = op:createPredicate("and(door(X:physobj))");
    Symbol !doorRef = act:positReference(!props);

    op:log("info", "Approaching door...");
    act:approachLocation(!doorRef);
    act:stopVisualSearch(!doorRef);
    op:log("info", "Opening door...");
    op:sleep(2000);
    act:open(!doorRef);

    act:generateResponseFromString(operator, "Done opening the door");

    op:log("info", "Done breaking open door");
}

() = escortCivilianToSafety(Symbol ?waypoint) {
    String !locName;
    edu.tufts.hrilab.slug.common.Utterance !utterance;
    edu.tufts.hrilab.slug.common.UtteranceType !type;
    Symbol !operator = operator;
    //Just to differentiate easily for local sayText
    Symbol !spot = spot;
    Symbol !temi = temi;

    !locName = act:getNameForSpotLocationReference(?waypoint);

    op:log("info", "escorting civilian to safety at !locName");

    //Optional: extended dialogue with the civilian (are you okay, can you walk, I have cleared the building, ...)

    //async?
    !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", !spot, !temi);
    !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setWords", "Hello, it is safe to exit the building now, please follow me back to safety");
    act:communicateUtterance(!utterance);

    //act:sayText("Hello, it is safe to exit the building now, please follow me back to safety");
    //generateResponseFromString(operator,...);?
    act:goToLocationWrapper(?waypoint);

    act:generateResponseFromString(operator,"Done escorting civilians to the !locName");
}

////ITK

(String ?output) = generateOutputStringFromSemantics(Predicate ?semantics) {
    Symbol !addressee;
    !addressee = this:getOriginalSubmitter();
    java.lang.Boolean !false = false;

    edu.tufts.hrilab.slug.common.Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !addressee);
    edu.tufts.hrilab.slug.common.UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setSemantics", ?semantics);
    !utterance = tsc:convertSemanticsToTextLLMService(!utterance, !false);

    ?output = op:invokeMethod(!utterance,"getWordsAsString");
}

() = getCurrentLocationString() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateCurrentLocationString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

(Symbol ?room) = getCurrentRoom() {
    Symbol !currentLocation;
    !currentLocation = act:getCurrentLocation();

    //query belief

    ?room = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "temp_room_name");
}

() = getAllKnownRooms() {
    String !output;
    Symbol !user = this:getOriginalSubmitter();
    Predicate !response;
    !output = act:generateAllKnownRoomsString();
    String !contextId = this:getCommand();
    java.lang.Long !goalID;
    !response = tsc:addToolResponse(!user,?actor,!output,!contextId);
    if (~op:isNull(!response)) {
        !goalID = act:submitGoal(!response);
        act:joinOnGoal(!goalID);
        !output = op:getName(!response);
        if (op:equals(!output,"generateResponseFromString")) {
          act:listenPostCommunicateUtterance(?actor);
        }
    }
}

(String ?output) = generateCurrentLocationString() {
    Symbol !currentLocation;
    Symbol !currentRoom;

    !currentLocation = act:getCurrentLocation();
    !currentRoom = act:getCurrentRoom();
    ?output = act:generateOutputStringFromSemantics(at(?actor, area(!currentLocation), room(!currentRoom)));
}

(String ?output) = generateAllKnownRoomsString() {
    List !roomNames;
    java.lang.String !roomName;
    java.lang.String !roomNameStr = "";
    java.lang.StringBuilder !sb = op:newObject("java.lang.StringBuilder");
    java.lang.String !sep = "";

    !roomNames = tsc:getRoomNames();
    if (op:isEmpty(!roomNames)) {
        ?output = op:newObject("java.lang.String","There are no known rooms");
    } else {
        op:invokeMethod(!sb,"append","You know the following rooms: ");
        foreach(!roomName : !roomNames) {
            op:invokeMethod(!sb,"append",!sep);
            // Explicitly convert to String to avoid numeric-looking names being coerced to integers
            !roomNameStr = op:invokeMethod(!roomName, "toString");
            op:invokeMethod(!sb,"append",!roomNameStr);
            !sep = op:newObject("java.lang.String",", ");
        }
        //op:invokeMethod(!sb,"append",". Please list no more than 5 rooms at a time in any response, unless explicitly asked for all/more/different rooms. You may still provide the total number of rooms you have saved and list 5 in your initial response though.");
        ?output = op:invokeMethod(!sb,"toString");
    }
}

(boolean ?objectFound) = lookAround(Symbol ?objectRef) {
  java.util.List !tokenIds;

  // four-stop horizontal hand-camera sweep ~140 deg, centered straight ahead. spot is stationary,
  // so the hand position stays fixed and only yaw about vertical varies (quats are pure yaw about z).
  // goToSpecificPose args: x,y,z,qx,qy,qz,qw in the body frame.

  act:goToSpecificPose(0.6, 0.0, 0.5, 0.0, 0.0, -0.5736, 0.8192); // -70 deg, right
  op:sleep(1000); // let the arm settle and the image catch up
  !tokenIds = act:lookForObject(?objectRef);
  act:stopVisualSearch(?objectRef);
  if (~op:isEmpty(!tokenIds)) {
    ?objectFound = op:newObject("java.lang.Boolean", true);
    return;
  }

  act:goToSpecificPose(0.6, 0.0, 0.5, 0.0, 0.0, -0.2022, 0.9793); // -23 deg
  op:sleep(1000);
  !tokenIds = act:lookForObject(?objectRef);
  act:stopVisualSearch(?objectRef);
  if (~op:isEmpty(!tokenIds)) {
    ?objectFound = op:newObject("java.lang.Boolean", true);
    return;
  }

  act:goToSpecificPose(0.6, 0.0, 0.5, 0.0, 0.0, 0.2022, 0.9793); // +23 deg
  op:sleep(1000);
  !tokenIds = act:lookForObject(?objectRef);
  act:stopVisualSearch(?objectRef);
  if (~op:isEmpty(!tokenIds)) {
    ?objectFound = op:newObject("java.lang.Boolean", true);
    return;
  }

  act:goToSpecificPose(0.6, 0.0, 0.5, 0.0, 0.0, 0.5736, 0.8192); // +70 deg, left
  op:sleep(1000);
  !tokenIds = act:lookForObject(?objectRef);
  act:stopVisualSearch(?objectRef);
  if (~op:isEmpty(!tokenIds)) {
    ?objectFound = op:newObject("java.lang.Boolean", true);
    return;
  }

  ?objectFound = op:newObject("java.lang.Boolean", false);
}