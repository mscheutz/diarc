import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.slug.common.Utterance;
import java.lang.Integer;
import java.lang.String;

// Main method to transfer to a "focused context" in a conversation between a user and the system. Currently implemented
//  to practically overwrite the System prompt and target Tool set for interactions between a user/AgentTeam pair until
//  an INSTRUCT Tool is called (initially designed towards such a one-off "template" population interaction)
// To get arguments to pass into this method, use:
//  this:getOriginalSubmitter();
//  ?actor
//  this:getCommand();
//  this:getId();
() = runContextSpecificChat(Symbol ?user, Symbol ?agentTeam, java.lang.String ?contextId, java.lang.Long ?cid) {
      onInterrupt : {
        cancel: act:cancelContextSpecificChat(!tracker,?user,?agentTeam,?cid);
        suspend: act:suspendContextSpecificChat(!tracker,?user,?agentTeam,?cid);
        resume: act:resumeContextSpecificChat(!tracker,?user,?agentTeam,?cid);
      }

    Integer !tracker = 0;
    Utterance !focusedChatInitialResponse;
    Symbol !focusedChatInitialResponseSemantics;
    String !semanticsName;
    Predicate !submissionPredicate;
    String !fillerResponse;

    op:log("info","[runContextSpecificChat] in method");

    //GenerateResponse or sayText? Do we want this in the history?
    //Filler response on context transfer
    !fillerResponse = tsc:getContextTransferFillerPhrase(?contextId,?cid,?user,?agentTeam);
    async {
        act:generateResponseFromString(?user,!fillerResponse);
    }
    op:sleep(50);

    // Have convoluted wrapper script to submit the initial side effect of instructing the "focus context" action as a
    //  separate goal in order for all possible interactions to work as intended: e.g. interruption methods, initial text
    //  response vs immediate tool call, OFFContextSubmission special case, result goal blocking in the queue, ...
    noInterrupt {
        !focusedChatInitialResponse = tsc:focusDialogueChat(?user, ?agentTeam, ?contextId, ?cid);
        !submissionPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "focusedContextResponseSubmissionWrapper(?actor,?user,?agentTeam,?cid)");
        ?gid = act:submitGoal(!submissionPredicate);
        !tracker = op:++(!tracker);
    }

    // If an issue occurred during focusDialogueChat
    if (op:isNull(!focusedChatInitialResponse)) {
        exit(FAIL,occurred(internalError));
    }
    !focusedChatInitialResponseSemantics = op:invokeMethod(!focusedChatInitialResponse, "getSemantics");
    !semanticsName = op:getName(!focusedChatInitialResponseSemantics);

    edu.tufts.hrilab.slug.common.UtteranceType !utteranceType = op:invokeMethod(!focusedChatInitialResponse,"getType");
    if (op:equalsValue(!utteranceType,"UNKNOWN")) {
        !utteranceType = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "INSTRUCT");
        op:invokeMethod(!focusedChatInitialResponse, "setType", !utteranceType);
        act:handleParsedUtterance(!focusedChatInitialResponse);
        tsc:endChatContext(?user,?agentTeam);
        exit(FAIL); //TODO: should just fail out with relevant semantics here
    }

    //  wait on lock in either case - should be released when moving on from specific context
    //  (goal canceled or validation complete)
    act:waitOnFocusedChat(?user,?agentTeam,?cid);

    op:log("info", "[runContextSpecificChat] out");
}
() = focusedContextResponseSubmissionWrapper(Symbol ?user, Symbol ?agentTeam, java.lang.Long ?cid) {
    Utterance !responseUtterance;

    op:log("debug","[focusedContextSubmissionWrapper] in method");

    !responseUtterance = tsc:getContextTransferResponseUtterance(?cid);

    if (op:isNull(!responseUtterance)) {
        op:log("warn","[focusedContextSubmissionWrapper] have null transfer response utterance, canceling waitOnFocusedChat");
        tsc:interruptWaitOnFocusedChat(?user,?agentTeam,?cid);
    } else {
        act:handleParsedUtterance(!responseUtterance);
        //if (tsc:usingWWBasedASR(!agentTeam)) {
        //    act:triggerWW(); //TODO: revisit
        //}
    }
}

// Focused Context Chat Interruption Methods
() = suspendContextSpecificChat(Integer ?tracker, Symbol ?user, Symbol ?agentTeam, long ?cid) {
    op:log("info","[suspendContextSpecificChat] in method ?tracker");
    if (op:gt(?tracker,0)) {
        tsc:suspendLLMChatContext(?user,?agentTeam,?cid);
    }
}

() = resumeContextSpecificChat(Integer ?tracker, Symbol ?user, Symbol ?agentTeam, long ?cid) {
    op:log("info","[resumeContextSpecificChat] in method ?tracker");
    String !response;
    if (op:gt(?tracker,0)) {
        tsc:resumeLLMChatContext(?user,?agentTeam,?cid);
        !response = tsc:callChatCompletionForTextResponse(?user,?agentTeam,?cid);
        if (op:invokeMethod(!response,"isEmpty")) {
            op:log("warn","[resumeContextSpecificChat] no response received on resume");
        } else {
            act:generateResponseFromString(?user,!response);
            act:listenPostCommunicateUtterance(?agentTeam);
        }
    }
}

() = cancelContextSpecificChat(Integer ?tracker, Symbol ?user, Symbol ?agentTeam, java.lang.Long ?cid) {
    op:log("info","[cancelContextSpecificChat] in method ?tracker ");
    if (op:gt(?tracker,0)) {
        tsc:cancelLLMChatContext(?user,?agentTeam,?cid);
    }
}

// General Focused Context Methods
() = scheduleTask() {
    Symbol !submitter;
    java.lang.Long !cid;
    String !contextId;

    !submitter = this:getOriginalSubmitter();
    !cid = this:getId();
    !contextId = this:getCommand();

    act:runContextSpecificChat(!submitter, ?actor, !contextId, !cid);

    op:log("info", "[scheduleTask] out");
}

() = learnTaskSequence() {
    Symbol !submitter;
    java.lang.Long !cid;
    String !contextId;

    !submitter = this:getOriginalSubmitter();
    !cid = this:getId();
    !contextId = this:getCommand();

    act:runContextSpecificChat(!submitter, ?actor, !contextId, !cid);

    op:log("info", "[learnTour] out");
}

() = learnMission() {
    Symbol !submitter;
    java.lang.Long !cid;
    String !contextId;

    !submitter = this:getOriginalSubmitter();
    !cid = this:getId();
    !contextId = this:getCommand();

    act:runContextSpecificChat(!submitter, ?actor, !contextId, !cid);

    op:log("info", "[learnTour] out");
}

// Empty script to be held as target to abandon base context interaction
() = endConversation() {
    op:log(info,"[endConversation]");

    //TODO: prompt the system to say something in this case? goodbye?
}

() = stopTalking() {
    op:log("info","[stopTalking] in method");
}

// Discard current chat state - not fully intended to be exposed to the user / not guaranteed safe during focused contexts or suspended states
() = resetConversation() {
    Symbol !submitter;

    op:log(info,"[endConversation]");

    !submitter = this:getOriginalSubmitter();
    tsc:endChatContext(!submitter,?actor);
    act:generateResponseFromString(!submitter,"Conversation has been reset");
}

//TODO: Ensure this goal occurs:
//  Immediately on submission - Submit with urgent priority?
//  Without other goals being submitted and added to learning state - Not currently possible?
//      Lock EM temporarily
//      Make independent learning state and submission API
// Old Signatures - remove when merge is completed and one representation is decided on (or split into separate methods)
//() = submitTaskSequenceDefinition(Symbol ?actionName, Symbol ?startLocation, Predicate ?startAction, Predicate ?locationSequence, Predicate ?actionSequence) {
//() = submitTaskSequenceDefinition(Symbol ?actionName, Predicate ?locationSequence, Predicate ?actionSequence) {
() = submitTaskSequenceDefinition(Symbol ?actionName, Predicate ?locationActionPairs) {
    Integer !index;
    java.util.List !pairs;
    Integer !numPairs;
    Predicate !pair;
    java.util.List !pairArgs;
    Predicate !actionPred;
    Predicate !actionSignature;
    Symbol !location;
    Symbol !lastLocation;
    Symbol !action;

    op:log(info,"[submitTaskSequenceDefinition] ?actionName: ?locationActionPairs");

    !actionSignature = op:createPredicate(?actionName);
    async {
        act:learnAction(!actionSignature);
    }
    act:waitForActionLearningStart(!actionSignature);

    !pairs = op:invokeMethod(?locationActionPairs, "getArgs");
    !numPairs = op:invokeMethod(!pairs,"size");
    for (!index=0; !index lt !numPairs; !index ++) {
        !pair = op:get(!pairs,!index);
        !pairArgs = op:invokeMethod(!pair, "getArgs");
        !location = op:get(!pairArgs,0);
        // ASL bug(?) with || not short circuiting requires duplicate blocks
        if (op:isNull(!lastLocation)) {
            !lastLocation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!location");
            !actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","goToLocation",?actor,!location);
            act:submitGoal(!actionPred);
        } elseif (~op:equalsValue(!location,!lastLocation)) {
            !lastLocation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!location");
            !actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","goToLocation",?actor,!location);
            act:submitGoal(!actionPred);
        }
        !action = op:get(!pairArgs,1);
        act:submitGoal(!action);
    }

    act:endActionLearning(!actionSignature);

    // TODO: if this instruction doesn't require validation, then we want a longer-form response listing learned steps? (Otherwise we should have already listed each step in the validation question)
    !actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "learnedHowTo(?actor,performTask(?actionName))");
    act:generateResponse(!actionPred);
}

() = startMission(Predicate ?actionList) {
    Integer !index;
    java.util.List !pairs;
    Integer !numPairs;
    Predicate !pair;
    java.util.List !pairArgs;
    Predicate !actionPred;
    Predicate !actionSignature;
    Symbol !location;
    Symbol !lastLocation;
    Symbol !action;
    Symbol !actionName = "definedMission";

    op:log(info,"[submitTourDefinition] ?actionList");

    //act:generateResponseFromString("Compiling mission steps");

    !actionSignature = op:createPredicate(!actionName);
    async {
        act:learnAction(!actionSignature);
    }
    act:waitForActionLearningStart(!actionSignature);

    !pairs = op:invokeMethod(?actionList, "getArgs");
    !numPairs = op:invokeMethod(!pairs,"size");
    for (!index=0; !index lt !numPairs; !index ++) {
        !action = op:get(!pairs,!index);
        //!pair = op:get(!pairs,!index);
        //!pairArgs = op:invokeMethod(!pair, "getArgs");
        //!location = op:get(!pairArgs,0);
        //// ASL bug(?) with || not short circuiting requires duplicate blocks
        //if (op:isNull(!lastLocation)) {
        //    !lastLocation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!location");
        //    !actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","goToLocation",?actor,!location);
        //    act:submitGoal(!actionPred);
        //} elseif (~op:equalsValue(!location,!lastLocation)) {
        //    !lastLocation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "!location");
        //    !actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","goToLocation",?actor,!location);
        //    act:submitGoal(!actionPred);
        //}
        //!action = op:get(!pairArgs,1);
        act:submitGoal(!action);
    }

    act:endActionLearning(!actionSignature);

    //// TODO: if this instruction doesn't require validation, then we want the long-form response. (Otherwise we have already
    ////  listed each step in the validation question)
    //act:generateResponseFromString("I have learned the new mission definition");
    ////!actionPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "learnedHowTo(?actor,performTask(?actionName))");
    ////act:generateResponse(!actionPred);

    act:generateResponseFromString("Embarking on my mission");
    act:definedMission();
}