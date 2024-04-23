//TODO: Use this a s a reference for the body of the new action that will match goto in ref pddl, add from location
() = goToLocation["?actor goes to ?destination and updates state info"](edu.tufts.hrilab.fol.Symbol ?destination, java.lang.Boolean ?clearScreen) {
    edu.tufts.hrilab.fol.Predicate !queryPred;// = at(?actor,X);
    edu.tufts.hrilab.fol.Predicate !failPred;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","at(?actor,X)");

    //For action timing
    edu.tufts.hrilab.fol.Variable !key = X;
    java.util.List !bindings;
    java.util.Map !binding;
    edu.tufts.hrilab.fol.Symbol !startLoc = "none";
    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("warn", "[goToLocation] locQuery returned no results");
    } else {
        !binding = op:get(!bindings, 0);
        !startLoc = op:get(!binding, !key);
    }

    if (act:knowsLocation(?destination)) {
//        act:retractBelief(!queryPred);
       op:log("info", "[goToLocation] submitting goTo from !startLoc to ?destination");
       if(act:goToLoc(?destination, ?clearScreen)){
         op:log("info", "[goToLocation] out goTo from !startLoc to ?destination");
         op:log("info", "[goToLocation] made it to ?destination");
       } else {
         if (act:getRepositionFailure()) {
           op:log("info", "[goToLocation] goTo from !startLoc to ?destination failed due to reposition failure");
           !failPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(reposition(?actor))");
         } else {
           op:log("info", "[goToLocation] goTo from !startLoc to ?destination failed during navigation");
           !failPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(go(?actor,?destination))");
         }
         exit(FAIL, !failPred);
       }
    } else {
        !failPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(knowWhere(?actor,?destination))");
        exit(FAIL, !failPred);
    }
}

//TODO implement carry which wraps goToLocation, but adds semantic info

() = goToPosition["?actor goes to ?destination and updates state info"](java.util.List ?goalCoords, edu.tufts.hrilab.fol.Symbol ?destination, java.lang.Boolean ?clearScreen) {
   edu.tufts.hrilab.fol.Predicate !queryPred;
   edu.tufts.hrilab.fol.Predicate !failPred;
   java.lang.Boolean !goToSuccess;

   (!queryPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","at(?actor,X)");
   (!goToSuccess) = act:goToPos(?goalCoords, ?destination, ?clearScreen);
   if (op:notEquals(!goToSuccess,true)) {
       if (act:getRepositionFailure()) {
           !failPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(reposition(?actor))");
       } else {
           !failPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(go(?actor,originalLocation))");
       }
       exit(FAIL, !failPred);
   }
}

() = goToThenSay["?actor goes to ?destination then says ?message"](edu.tufts.hrilab.fol.Symbol ?destination, java.lang.String ?message) {
   java.lang.Boolean !clearScreen = true;

   act:goToLocation(?destination, !clearScreen);
   act:generateResponseFromString(?message);
}

() = goToThenDisplay["?actor goes to ?destination then displays ?message"](edu.tufts.hrilab.fol.Symbol ?destination, java.lang.String ?message) {
   java.lang.Boolean !clearScreen = false;

   act:goToLocation(?destination, !clearScreen);
   act:display(?message);
}

() = goToThenPlay["?actor goes to ?destination then plays video ?label"](edu.tufts.hrilab.fol.Symbol ?destination, edu.tufts.hrilab.fol.Symbol ?label) {
   java.lang.Boolean !clearScreen = false;

   act:goToLocation(?destination, !clearScreen);
   act:playVideo(?label);
}

() = setGreetDest["sets waitingArea to ?location"](edu.tufts.hrilab.fol.Symbol ?location) {
    edu.tufts.hrilab.fol.Predicate !retractPred = waitingArea(X);
    edu.tufts.hrilab.fol.Predicate !assertPred;
    act:retractBelief(!retractPred);
    !assertPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","waitingArea(?location)");
    act:assertBelief(!assertPred);
}

() = setStorageArea["sets storageArea to ?location"](edu.tufts.hrilab.fol.Symbol ?location) {
    edu.tufts.hrilab.fol.Predicate !retractPred = storageArea(X);
    edu.tufts.hrilab.fol.Predicate !assertPred;
    act:retractBelief(!retractPred);
    !assertPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","storageArea(?location)");
    act:assertBelief(!assertPred);
}

//TODO: test on real temi - believe we need to add onsuspend/cancel/resume methods for updating UI if interrupted during askQuestion
() = greet["?actor greets a patient in ?patientLoc and brings them to the WAITING AREA"](edu.tufts.hrilab.fol.Symbol ?patientLoc) {
    edu.tufts.hrilab.fol.Predicate !queryPred = waitingArea(X);
    edu.tufts.hrilab.fol.Variable !key = X;
    java.util.List !bindings;
    java.util.Map !binding;
    edu.tufts.hrilab.fol.Symbol !destination;
    edu.tufts.hrilab.fol.Predicate !noBindings;// = not(know(?actor,waitingLocation));
    java.lang.Boolean !toGreet = false;
    java.lang.Boolean !fromGreet = true;
    edu.tufts.hrilab.fol.Symbol !addressee = "patient";
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "waitForAckTemi(X)";

    !noBindings = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(know(?actor,waitingLocation))");

    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        exit(FAIL, !noBindings);
    }
    !binding = op:get(!bindings, 0);
    !destination = op:get(!binding, !key);
    act:goToLocation(?patientLoc, !toGreet);
    //ob:acknowledged(!ackMessage);
    act:waitForDetection();
    act:startAcknowledgeDisplay("Hello, please follow me to the waiting area.");
    act:askQuestionFromString(!addressee, "Hello, please follow me to the waiting area.", !waitForAckForm);
    act:endAcknowledgeDisplay();
    act:goToLocation(!destination, !fromGreet);
    act:generateResponseFromString("Someone will be with you shortly");
}

() = escort["?actor takes ?name from ?fromLocation to ?destination"](edu.tufts.hrilab.fol.Symbol ?destination, edu.tufts.hrilab.fol.Symbol ?name, edu.tufts.hrilab.fol.Symbol ?fromLocation = "current") {
    //TODO: test on real temi - See if resuming from goToLocation has UI conflicts or not. Otherwise will need additional logic within the component to track state or not
    onInterrupt :{
        cancel: tsc:endAcknowledgeDisplay();
        suspend: tsc:endAcknowledgeDisplay();
        resume: tsc:startAcknowledgeDisplay("Hello ?name. Please come with me. I will show you to !destinationName.");
    }
    java.lang.Float !waitDuration;
    java.lang.Integer !maxWaitAttempts;
    edu.tufts.hrilab.fol.Symbol !unknownLoc = unknown;
    java.lang.Integer !i = 0;
    java.lang.Boolean !escortingTo = false;
    java.lang.Boolean !escortingFrom = true;
    java.lang.String !destinationName;
    java.lang.String !fromString;
    edu.tufts.hrilab.fol.Symbol !addressee = "patient";
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "waitForAckTemi(X)";
    java.util.Map !ackAnswerBindings;

    (!waitDuration) = act:getEscortWaitDuration();
    (!maxWaitAttempts) = act:getEscortWaitAttempts();

    (!fromString) = op:getName(?fromLocation);
    //TODO: can't have location with name 'current' as is
    if(op:notEquals(!fromString,"current")){
       op:log("info", "[escort] Going to start location");
       act:goToLocation(?fromLocation, !escortingTo);
    }

    op:log("info", "[escort] entering patient request loop with !maxWaitAttempts attempts lasting !waitDuration seconds");
    //EW TODO: Avoid extra sayText if acknowledged between loops
    //Don't want to reset acknowledgement on subsequent loops
    (!destinationName) = act:getLocationNameFromReference(?destination);
    op:log("info","[escort] entering waitForAck");
    act:startAcknowledgeDisplay("Hello ?name. Please come with me. I will show you to !destinationName.");
    !ackAnswerBindings = act:askQuestionFromString(!addressee, "Hello ?name. Please come with me. I will show you to !destinationName.", !waitForAckForm, !maxWaitAttempts, !waitDuration);
    act:endAcknowledgeDisplay();
    op:log("info","[escort] out waitForAck");

    act:goToLocation(?destination, !escortingFrom);

    //If not acknowledged and ran out of tries -> clear screen, return to original location, and
    //  say that we could not find the requested patient
    if(op:isNull(!ackAnswerBindings)) {
//        op:log("info", "[escort] going to destination to communicate failure to find patient");
        act:generateResponseFromString("I could not find the requested patient ?name");
    } else {
        act:generateResponseFromString("Arrived at !destinationName");
    }
}

() = fetch["?actor goes to !storageLocation, asks for ?item, and brings ?item to !startLocation "](edu.tufts.hrilab.fol.Symbol ?item) {
    //TODO: test on real temi - See if resuming from goToLocation has UI conflicts or not. Otherwise will need additional logic within the component to track state or not
    onInterrupt :{
      suspend: tsc:endAcknowledgeDisplay();
      cancel: tsc:endAcknowledgeDisplay();
      resume: tsc:startAcknowledgeDisplay("Hello ?name. Please come with me. I will show you to !destinationName.");
    }

    edu.tufts.hrilab.fol.Predicate !locQuery;// = at(?actor,X);
    edu.tufts.hrilab.fol.Symbol !startLoc = "unknown";
    edu.tufts.hrilab.fol.Symbol !origLocString;
    edu.tufts.hrilab.fol.Predicate !noBindingsLoc; //= not(knowWhere(?actor));
    edu.tufts.hrilab.fol.Predicate !storageQuery = storageArea(X);
    edu.tufts.hrilab.fol.Predicate !noBindingsStorage;// = not(know(?actor,storageLocation));
    edu.tufts.hrilab.fol.Symbol !storageLoc;
    edu.tufts.hrilab.fol.Variable !key = X;
    java.util.List !startLocBindings;
    java.util.Map !startLocBinding;
    java.util.List !storageBindings;
    java.util.Map !storageBinding;
    java.util.List !startCoords;
    edu.tufts.hrilab.fol.Symbol !unknownLoc = unknown;
    java.lang.Boolean !toFetch = false;
    java.lang.Boolean !fromFetch = true;
    edu.tufts.hrilab.fol.Symbol !addressee = "patient";
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "waitForAckTemi(X)";

   (!locQuery) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","at(?actor,X)");
   (!noBindingsLoc) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(knowWhereIAm(?actor))");
   (!noBindingsStorage) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(know(?actor,storageLocation))");
   (!origLocString) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol","Original Location");


    !startLocBindings = act:queryBelief(!locQuery);
    if (op:isEmpty(!startLocBindings)) {
        op:log("info", "[fetch (?item)] locQuery returned no results");
    } else {
        !startLocBinding = op:get(!startLocBindings, 0);
        !startLoc = op:get(!startLocBinding, !key);
    }

    //TODO: same as escort todo
    if (op:isEmpty(!startLocBindings)) {
        (!startCoords) = act:getCurrentPosition();
        op:log("warn", "[fetch] (empty bindings) don't know current position, got coordinates !startCoords");
        if (op:isNull(!startCoords)) {
                exit(FAIL, !noBindingsLoc);
        }
    } else {
        if (op:equals(!startLoc, !unknownLoc)) {
            op:log("warn", "[fetch] (unknown Loc) don't know current position, got coordinates !startCoords");
            (!startCoords) = act:getCurrentPosition();
            if (op:isNull(!startCoords)) {
                    exit(FAIL, !noBindingsLoc);
            }
        } else {
            op:log("info", "[fetch] know current position, got start loc !startLoc");
        }
    }

    !storageBindings = act:queryBelief(!storageQuery);
    if (op:isEmpty(!storageBindings)) {
        exit(FAIL, !noBindingsStorage);
    }
    !storageBinding = op:get(!storageBindings, 0);
    !storageLoc = op:get(!storageBinding, !key);
    act:goToLocation(!storageLoc, !toFetch);

    //Call nonblocking version of sayText without overlay before waitForAck so that timing of display isn't awkward
    act:setSayTextBlocks(false);
    act:generateResponseFromString("Could I please have ?item");
    act:setSayTextBlocks(true);

    op:log("info","[fetch] entering waitForAck");
    act:startAcknowledgeDisplay("Please place ?item on my tray.");
    act:waitForResponse(!waitForAckForm);
    act:endAcknowledgeDisplay();
    op:log("info","[fetch] out waitForAck");

    if (op:isEmpty(!startLocBindings)) {
        act:goToPosition(!startCoords, !origLocString, true);
    } else {
        if (op:equals(!startLoc, !unknownLoc)) {
            act:goToPosition(!startCoords, !origLocString, true);
        } else {
            act:goToLocation(!startLoc, !fromFetch);
        }
    }
    act:generateResponseFromString("I have brought ?item");
}

() = fetch["?actor goes to ?location, asks for ?item, and brings ?item to !startLocation  "](edu.tufts.hrilab.fol.Symbol ?item, edu.tufts.hrilab.fol.Symbol ?location) {
    //TODO: test on real temi - See if resuming from goToLocation has UI conflicts or not. Otherwise will need additional logic within the component to track state or not
    onInterrupt :{
      suspend: tsc:endAcknowledgeDisplay();
      cancel: tsc:endAcknowledgeDisplay();
      resume: tsc:startAcknowledgeDisplay("Hello ?name. Please come with me. I will show you to !destinationName.");
    }

    edu.tufts.hrilab.fol.Predicate !locQuery;// = at(?actor,X);
    edu.tufts.hrilab.fol.Symbol !startLoc = "unknown";
    edu.tufts.hrilab.fol.Symbol !origLocString;
    edu.tufts.hrilab.fol.Predicate !noBindingsLoc;// = not(knowWhere(?actor));
    edu.tufts.hrilab.fol.Variable !key = X;
    java.util.List !bindings;
    java.util.Map !binding;
    java.util.List !startCoords;
    edu.tufts.hrilab.fol.Symbol !unknownLoc = unknown;
    java.lang.Boolean !toFetch = false;
    java.lang.Boolean !fromFetch = true;
    edu.tufts.hrilab.fol.Symbol !addressee = "patient";
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "waitForAckTemi(X)";

   (!locQuery) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","at(?actor,X)");
   (!noBindingsLoc) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","not(knowWhereIAm(?actor))");
   (!origLocString) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol","Original Location");

    //get start location
    !bindings = act:queryBelief(!locQuery);
    if (op:isEmpty(!bindings)) {
        op:log("info", "[fetch (?item,?location)] locQuery returned no results");
    } else {
        !binding = op:get(!bindings, 0);
        !startLoc = op:get(!binding, !key);
    }

    //TODO: same as escort todo
    if (op:isEmpty(!bindings)) {
        (!startCoords) = act:getCurrentPosition();
        op:log("info", "[fetch] don't know current position, got coordinates !startCoords");
        if (op:isNull(!startCoords)) {
                exit(FAIL, !noBindingsLoc);
        }
    } else {
        if (op:equals(!startLoc, !unknownLoc)) {
            (!startCoords) = act:getCurrentPosition();
            op:log("info", "[fetch] don't know current position, got coordinates !startCoords");
            if (op:isNull(!startCoords)) {
                    exit(FAIL, !noBindingsLoc);
            }
        } else {
            op:log("info", "[fetch] know current position, got start loc !startLoc");
        }
    }

    act:goToLocation(?location, !toFetch);

    //Call nonblocking version of sayText without overlay before waitForAck so that timing of display isn't awkward
    act:setSayTextBlocks(false);
    act:generateResponseFromString("Could I please have ?item");
    act:setSayTextBlocks(true);

    op:log("info","[fetch] entering waitForAck");
    act:startAcknowledgeDisplay("Please place ?item on my tray.");
    act:waitForResponse(!waitForAckForm);
    act:endAcknowledgeDisplay();
    op:log("info","[fetch] out waitForAck");

    if (op:isEmpty(!bindings)) {
        act:goToPosition(!startCoords, !origLocString, true);
    } else {
        if (op:equals(!startLoc, !unknownLoc)) {
            act:goToPosition(!startCoords, !origLocString, true);
        } else {
            act:goToLocation(!startLoc, !fromFetch);
        }
    }
    act:generateResponseFromString("I have brought ?item");
}

() = displayQRCode["displays qr code linking to ?url while displaying ?header label"](java.lang.String ?url, java.lang.String ?header) {
    onInterrupt :{
      suspend: tsc:endQRCodeDisplay();
      cancel: tsc:endQRCodeDisplay();
      resume: tsc:startQRCodeDisplay(?url, ?header);
    }
    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "waitForAckTemi(X)";

    act:startQRCodeDisplay(?url, ?header);
    act:waitForResponse(!waitForAckForm);
    act:endQRCodeDisplay();
}

//() = interruptGoToLocation["Performs all behavior necessary to unblock and clear effects of goToLocation primitive"]() {
//    edu.tufts.hrilab.fol.Predicate !waitForAckForm = "waitForAckTemi(X)";
//    //speaker and listener not used at time of impl
//    edu.tufts.hrilab.fol.Symbol !listener = "listener";
//
//    act:endRepositionLoop();
//    act:answerQuestion(?actor, !listener, !waitForAckForm);
//    act:stopMoving();
//}