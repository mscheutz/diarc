() = cleanRoom["?actor goes to ?room, with the assumption that a fiducial is visible, docks with the associated dock, and then executes a use define trajectory associated with that dock."](edu.tufts.hrilab.fol.Symbol ?room) {
    effects : {
        success : at(?actor,?room);
        nonperf : cleanRoom(?room);
    }
   edu.tufts.hrilab.fol.Symbol !office1 = "office1";
   edu.tufts.hrilab.fol.Symbol !office2 = "office2";
   edu.tufts.hrilab.fol.Symbol !alpha = "alpha";
   edu.tufts.hrilab.fol.Symbol !beta = "beta";
   edu.tufts.hrilab.fol.Symbol !hallway = "hallway";

    //TODO:brad: use return value justification for failure explanation
    act:goTo(!office2,true);

    act:connectToDock();

    //TODO:brad it seems like planning doesn't work on Android, maybe because of proguard rules?
    //this can be refactored to work as a primitive, though
    //    state:sanitized(?room);
    //    act:testSanitization();

    act:goTo(!beta,true);
    act:triggerRelay(true);
    op:sleep(5000);

    act:triggerRelay(false);
    act:turnDockOff();
    op:sleep(5000);

    act:exitRoom();
}

() = exitRoom["?actor goes to ?room, with the assumption that a fiducial is visible, docks with the associated dock, and then executes a use define trajectory associated with that dock."]() {

   edu.tufts.hrilab.fol.Symbol !office2 = "office2";
   edu.tufts.hrilab.fol.Symbol !alpha = "alpha";
   edu.tufts.hrilab.fol.Symbol !beta = "beta";
   edu.tufts.hrilab.fol.Symbol !hallway = "hallway";

   act:goTo(!office2,true);
   act:disconnectFromDock();
   act:goTo(!office2,true);
   act:goTo(!hallway,true);
}



() = connectToDock["docks with the nearest visible dock and turns power to dock on"](){
    act:triggerRelay(false);
    act:dockVisible();
    //extra padding to make sure the servo worked
    op:sleep(1000);
    //send signal to smart switch from android tablet
    act:turnDockOn();
    op:sleep(2000);
    //back up
}

() = disconnectFromDock["docks with the nearest visible dock and turns power to dock on"](){
    act:triggerRelay(false);
    act:turnDockOff();
    act:undock();
    //extra padding to make sure the servo worked
    op:sleep(1000);
    //back up
    act:testMove();
}


//TODO:brad: implement teaching framework similar to model assembly
//ideally this would be taught in a similar fashion to how the model scripts are taught
() = testSanitization["place holder sanitization script"](){

   effects : {
       success : sanitized(test);
   }

   edu.tufts.hrilab.fol.Symbol !gamma = "gamma";
   edu.tufts.hrilab.fol.Symbol !delta = "delta";
   act:goTo(!gamma);
   //todo:brad: turn on payload?
   act:triggerRelay(true);
   op:sleep(1000);
   act:goTo(!delta);
   op:sleep(1000);
   act:triggerRelay(false);
}



//Copied from temi.asl from branch temi/dolphin language
//() = goToLocation["?actor goes to ?destination and updates state info"](edu.tufts.hrilab.fol.Symbol ?destination) {
//    edu.tufts.hrilab.fol.Predicate !queryPred;// = at(?actor,X);
//    edu.tufts.hrilab.fol.Predicate !failPred;
//    effects : {
//        success : at(?actor,?destination);
//        nonperf : at(?actor,unknown);
//    }
//
//    (!queryPred) = op:createPredicate("at(?actor,X)");
//    if (act:knowsLocation(?destination)) {
//        act:retractBelief(!queryPred);
//       if(act:goToLoc(?destination)){
//         op:log(info, "made it to ?destination");
//       }else{
//        op:createPredicate(not(go(?actor,?destination)), !failPred);
//        exit(FAIL, !failPred);
//       }
//    }
//    else {
//        op:createPredicate(not(knowWhere(?actor,?destination)), !failPred);
//        exit(FAIL, !failPred);
//    }
//}
//
//() = goToPosition["?actor goes to ?destination and updates state info"](java.util.List ?goalCoords) {
//    edu.tufts.hrilab.fol.Predicate !queryPred = at(zno,X);
//    edu.tufts.hrilab.fol.Predicate !failPred;
//    effects : {
//        success : at(?actor,unknown);
//        nonperf : at(?actor,unknown);
//    }
//    act:retractBelief(!queryPred);
//   if(act:goToPos(?goalCoords)){
//     op:log(info, "made it to coords ?x,?y");
//   }else{
//    op:createPredicate(not(go(zno,originallocation)), !failPred);
//    exit(FAIL, !failPred);
//   }
//}
//
//() = interruptFollowBlocking[""]() {
//   act:acknowledge();
//   act:stopMoving();
//}
//
//() = interruptGoToLocation[""]() {
//   act:endRepositionLoop();
//   act:acknowledge();
//   act:stopMoving();
//}
//
//() = goToThenSay["?actor goes to ?destination then says ?message"](edu.tufts.hrilab.fol.Symbol ?destination, edu.tufts.hrilab.fol.Symbol ?message) {
//   java.lang.String !toSay ="";
//
//   op:getName(?message,!toSay);
//   act:goToLocation(?destination);
//   act:sayTextDialogueHistory(!toSay);
//}
//
//() = goToThenDisplay["?actor goes to ?destination then displays ?message"](edu.tufts.hrilab.fol.Symbol ?destination, edu.tufts.hrilab.fol.Symbol ?message) {
//   java.lang.String !toDisplay ="";
//
//   op:getName(?message,!toDisplay);
//   act:goToLocation(?destination);
//   act:display(!toDisplay);
//}
//
//() = goToThenPlay["?actor goes to ?destination then plays video ?label"](edu.tufts.hrilab.fol.Symbol ?destination, edu.tufts.hrilab.fol.Symbol ?label) {
//   java.lang.String !videoLabel ="";
//
//   op:getName(?label,!videoLabel);
//   act:goToLocation(?destination);
//   act:playVideo(!videoLabel);
//}
//
//() = setGreetDest["sets waitingArea to ?location"](edu.tufts.hrilab.fol.Symbol ?location) {
//    edu.tufts.hrilab.fol.Predicate !retractPred = waitingArea(X);
//    edu.tufts.hrilab.fol.Predicate !assertPred;
//    act:retractBelief(!retractPred);
//    op:createPredicate(waitingArea(?location), !assertPred);
//    act:assertBelief(!assertPred);
//}
//
//() = setStorageArea["sets storageArea to ?location"](edu.tufts.hrilab.fol.Symbol ?location) {
//    edu.tufts.hrilab.fol.Predicate !retractPred = storageArea(X);
//    edu.tufts.hrilab.fol.Predicate !assertPred;
//    act:retractBelief(!retractPred);
//    op:createPredicate(storageArea(?location), !assertPred);
//    act:assertBelief(!assertPred);
//}
//
//() = greet["?actor greets a patient in ?patientLoc and brings them to the WAITING AREA"](edu.tufts.hrilab.fol.Symbol ?patientLoc) {
//    edu.tufts.hrilab.fol.Predicate !queryPred = waitingArea(X);
//    edu.tufts.hrilab.fol.Variable !key = X;
//    java.util.List !bindings;
//    java.util.Map !binding;
//    edu.tufts.hrilab.fol.Symbol !destination;
//    edu.tufts.hrilab.fol.Predicate !noBindings;// = not(know(?actor,waitingLocation));
//
//    (!noBindings) = op:createPredicate("not(know(?actor,waitingLocation))");
//
//    act:queryAllBelief(!queryPred, !bindings);
//    if (op:isEmpty(!bindings)) {
//        exit(FAIL, !noBindings);
//    }
//    op:get(!bindings, 0, !binding);
//    op:get(!binding, !key, !destination);
//    act:goToLocation(?patientLoc);
//    //ob:acknowledged(!ackMessage);
//    act:waitForDetection();
//    act:sayTextDialogueHistory("Hello");
//    act:waitForAck("Please follow me to the waiting area.");
//    act:sayTextDialogueHistory("Hello, please follow me to the waiting area.");
//    act:goToLocation(!destination);
//    act:sayTextDialogueHistory("someone will be with you shortly");
//}
//
//() = escort["?actor takes ?name from ?fromLocation to ?destination"](edu.tufts.hrilab.fol.Symbol ?destination, java.lang.String ?name, java.lang.String ?fromLocation = "current") {
//    java.lang.Float !waitDuration;
//    java.lang.Integer !maxWaitAttempts;
//    edu.tufts.hrilab.fol.Symbol !unknownLoc = unknown;
//    java.lang.Integer !i = 0;
//    java.lang.Boolean !acknowledged = false;
//
//    (!waitDuration) = act:getEscortWaitDuration();
//    (!maxWaitAttempts) = act:getEscortWaitAttempts();
//
//    if(op:notEquals(?fromLocation,"current")){
//       op:log(info, "[escort] Going to start location");
//       act:goToLocation(?fromLocation);
//    }
//
//    op:log(info, "[escort] entering patient request loop with !maxWaitAttempts attempts lasting !waitDuration seconds");
//    //EW TODO: Avoid extra sayText if acknowledged between loops
//    //Don't want to reset acknowledgement on subsequent loops
//    (!acknowledged) = act:waitForAck("Hello ?name. Please come with me. I will show you to ?destination.", !waitDuration, !maxWaitAttempts);
//
//    act:goToLocation(?destination);
//    act:sayTextDialogueHistory("Arrived at ?destination");
//    //If not acknowledged and ran out of tries -> clear screen, return to original location, and
//    //  say that we could not find the requested patient
//    if(op:notEquals(!acknowledged,true)) {
//
//        op:log(info, "[escort] going to destination to communicate failure to find patient");
//
//        act:sayTextDialogueHistory("I could not find the requested patient ?name");
//    }
//}
//
//() = fetch["?actor goes to !storageLocation, asks for ?item, and brings ?item to !startLocation "](edu.tufts.hrilab.fol.Symbol ?item) {
//    edu.tufts.hrilab.fol.Predicate !locQuery;// = at(?actor,X);
//    edu.tufts.hrilab.fol.Symbol !startLoc;
//    edu.tufts.hrilab.fol.Predicate !noBindingsLoc; //= not(knowWhere(?actor));
//    edu.tufts.hrilab.fol.Predicate !storageQuery = storageArea(X);
//    edu.tufts.hrilab.fol.Predicate !noBindingsStorage;// = not(know(?actor,storageLocation));
//    edu.tufts.hrilab.fol.Symbol !storageLoc;
//    edu.tufts.hrilab.fol.Variable !key = X;
//    java.util.List !bindings;
//    java.util.Map !binding;
//    java.util.List !startCoords;
//    edu.tufts.hrilab.fol.Symbol !unknownLoc = unknown;
//
//   (!locQuery) = op:createPredicate("at(?actor,X)");
//   (!noBindingsLoc) =op:createPredicate("not(knowWhere(?actor))");
//   (!noBindingsStorage) = op:createPredicate("not(know(?actor,storageLocation))");
//
//    act:queryAllBelief(!locQuery, !bindings);
//    if (op:isEmpty(!bindings)) {
//        //Is this a different issue entirely? When we don't know where we are we still return 'unknown'
////        exit(FAIL, !noBindingsLoc);
//        op:log(info, "locQuery returned no results");
////        (!startLoc) = op:newObject(edu.tufts.hrilab.fol.Symbol, "unknown");
//    } else {
//        op:get(!bindings, 0, !binding);
//        op:get(!binding, !key, !startLoc);
//    }
//
//    //TODO: same as escort todo
//    if (op:isEmpty(!bindings)) {
//        (!startCoords) = act:getCurrentPosition();
//        op:log(info, "[fetch] don't know current position, got coordinates !startCoords");
//    } else {
//        if (op:equals(!startLoc, !unknownLoc)) {
//            op:log(info, "[fetch] don't know current position, got coordinates !startCoords");
//            (!startCoords) = act:getCurrentPosition();
//        } else {
//            op:log(info, "[fetch] know current position, got start loc !startLoc");
//        }
//    }
//
//    act:queryAllBelief(!storageQuery, !bindings);
//    if (op:isEmpty(!bindings)) {
//        exit(FAIL, !noBindingsStorage);
//    }
//    op:get(!bindings, 0, !binding);
//    op:get(!binding, !key, !storageLoc);
//    act:goToLocation(!storageLoc);
//    act:sayTextDialogueHistory("Could I please have ?item");
//    act:waitForAck("Please place ?item on my tray.");
//    if (op:isEmpty(!bindings)) {
//        act:goToPosition(!startCoords);
//    } else {
//        if (op:equals(!startLoc, !unknownLoc)) {
//            act:goToPosition(!startCoords);
//        } else {
//            act:goToLocation(!startLoc);
//        }
//    }
//    act:sayTextDialogueHistory("I have brought ?item");
//}
//
//() = fetch["?actor goes to ?location, asks for ?item, and brings ?item to !startLocation  "](edu.tufts.hrilab.fol.Symbol ?item, edu.tufts.hrilab.fol.Symbol ?location) {
//    edu.tufts.hrilab.fol.Predicate !locQuery;// = at(?actor,X);
//    edu.tufts.hrilab.fol.Symbol !startLoc;
//    edu.tufts.hrilab.fol.Predicate !noBindingsLoc;// = not(knowWhere(?actor));
//     edu.tufts.hrilab.fol.Variable !key = X;
//    java.util.List !bindings;
//    java.util.Map !binding;
//    java.util.List !startCoords;
//    edu.tufts.hrilab.fol.Symbol !unknownLoc = unknown;
//
//   (!locQuery) = op:createPredicate("at(?actor,X)");
//   (!noBindingsLoc) =op:createPredicate("not(knowWhere(?actor))");
//
//    //get start location
//    act:queryAllBelief(!locQuery, !bindings);
//    if (op:isEmpty(!bindings)) {
//        //Is this a different issue entirely? When we don't know where we are we still return 'unknown'
////        exit(FAIL, !noBindingsLoc);
//        op:log(info, "locQuery returned no results");
////        (!startLoc) = op:newObject(edu.tufts.hrilab.fol.Symbol, "unknown");
//    } else {
//        op:get(!bindings, 0, !binding);
//        op:get(!binding, !key, !startLoc);
//    }
//
//    //TODO: same as escort todo
//    if (op:isEmpty(!bindings)) {
//        (!startCoords) = act:getCurrentPosition();
//        op:log(info, "[fetch] don't know current position, got coordinates !startCoords");
//    } else {
//        if (op:equals(!startLoc, !unknownLoc)) {
//            op:log(info, "[fetch] don't know current position, got coordinates !startCoords");
//            (!startCoords) = act:getCurrentPosition();
//        } else {
//            op:log(info, "[fetch] know current position, got start loc !startLoc");
//        }
//    }
//
//    act:goToLocation(?location);
//    act:sayTextDialogueHistory("Could I please have ?item");
//    act:waitForAck("Please place ?item on my tray.");
//    if (op:isEmpty(!bindings)) {
//        act:goToPosition(!startCoords);
//    } else {
//        if (op:equals(!startLoc, !unknownLoc)) {
//            act:goToPosition(!startCoords);
//        } else {
//            act:goToLocation(!startLoc);
//        }
//    }
//    act:sayTextDialogueHistory("I have brought ?item");
//}
//
////Clean and edit methods with unnecessary return values
//() = freeze[""]() {
//    edu.tufts.hrilab.fol.Predicate !goalPred;
//
//    java.util.List !predArgs;
//    java.lang.String !pausedTaskName;
//
//    (!goalPred) = act:suspendSystemGoal(?actor);
//
//    //don't allow freezing of nothing to throw an exception
//    if (~op:isNull(!goalPred)) {
//        (!predArgs) = op:getArgs(!goalPred);
//        (!pausedTaskName) = op:get(!predArgs, 1);
//        //Don't allow freezing of freeze
//        if (~op:equals(!pausedTaskName,"freeze()")) {
//            if (~act:waitForAckFreeze()) {
//                //waitForAck will return true if the cancel was called during freeze, in which case
//                //  we want freeze to terminate without resuming the paused goal
//                act:resumeSystemGoal(?actor);
//            }
////            (!taskCancelled) = act:waitForAckFreeze();
////            act:resumeSystemGoal(?actor);
//        }
//    }
//}