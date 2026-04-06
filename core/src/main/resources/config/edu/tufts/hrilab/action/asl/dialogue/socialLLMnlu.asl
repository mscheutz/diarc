import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.slug.common.Utterance;
import java.lang.Integer;
import java.lang.String;

/**
  * Wait for incoming utterances (requires ListenerComponent) and spins
  * off a new thread to handle each incoming utterance.
  * Starting a "listen(?actor)" goal is required for using the NL system.
  */
() = listen() {
    Utterance !utterance;

    while (true) {
      // wait for input utterance
      op:log("debug", "Waiting for utterance.");
      !utterance = act:waitForUtterance();
      op:log("debug", "Received utterance: !utterance");

      // to exit while loop during shutdown
      if (op:isNull(!utterance)) {
        op:log("warn", "Received null utterance. Exiting listen action.");
        return;
      }

      // asynchronously pass utterance to NLU and immediately go back to listening for new utterances
      async {
        op:log("debug", "Calling handleSpeechInput...");
        act:handleSpeechInput(!utterance);
        op:log("debug", "... done calling handleSpeechInput.");
      }
    }
}

/**
  * Handle an incoming utterance. This is the main control flow for the NLU "pipeline".
  */
() = handleSpeechInput(Utterance ?utterance) {
    op:log("debug", "handleSpeechInput for utterance: ?utterance");

    // parse utterance
    op:log("debug", "calling parser ...");
    ?utterance = act:parseUtterance(?utterance);
    op:log("debug", "parser results: ?utterance");

    act:handleParsedUtterance(?utterance);
}

() = handleParsedUtterance(Utterance ?utterance, java.lang.Boolean ?updateHistory=true) {
    java.lang.Boolean !continueValidation;
    java.lang.Boolean !validated;
    Symbol !speaker;
    Symbol !addressee;

    // parser error handling
    java.lang.String !semanticsName;
    Symbol !semantics = op:invokeMethod(?utterance, "getSemantics");
    java.lang.String !failureString;
    Utterance !failureUtterance;
    if (op:isNull(!semantics)) {
      act:updateDialogueHistory(?utterance);
      !speaker = op:invokeMethod(?utterance, "getSpeaker");
      !addressee = op:invokeMethod(?utterance, "getAddressee");
      !semantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(understand(!addressee,that))");
      !addressee.act:handleError(!speaker, !addressee, !semantics, direct);
      return;
    } else {
        //TODO: update this case to play nice with new LLM contexts
        //Originally Ava specific change to allow custom error messages from hybridParser - universally desired?
        !semanticsName = op:getName(!semantics);
        if (op:equals(!semanticsName,"error")) {
            act:updateDialogueHistory(?utterance);
            !speaker = op:invokeMethod(?utterance, "getSpeaker");
            !addressee = op:invokeMethod(?utterance, "getAddressee");
            act:handleError(!speaker, !addressee, !semantics, direct);
            //!failureString = op:invokeMethod(?utterance,"getWordsAsString");
            //if (op:invokeMethod(!failureString,"isEmpty")) {
            //    act:handleError(!speaker, !addressee, !semantics, direct);
            //} else {
            //    !addressee.act:generateResponseFromString(!speaker,!failureString);
            //}
            return;
        }
    }

    // perform reference resolution
    op:log("debug", "calling rr ...");
    ?utterance = act:resolveReferences(?utterance);
    op:log("debug", "rr results: ?utterance");

    // for LLM parses -- get human validation
    if (op:invokeMethod(?utterance, "needsValidation")) {

        // (Re)validate attempted parses until the user validates it or fully gives up
        //   (On negative validation, extended dialogue recovery and further parsing attempts are performed internally)
        !continueValidation = act:validateParse(?utterance,?updateHistory);
        while (op:equals(!continueValidation,true)) {
            !continueValidation = act:validateParse(?utterance,false);
        }

        // Exiting the validation loop without the utterance being validated signals failure, otherwise progress
        //   through rest of NLU pipeline
        !validated = op:invokeMethod(?utterance,"isValidated");
        if (op:equals(!validated,false)) {
            op:log("debug", "not validated: ?utterance");
            return;
        } else {
            op:log("debug", "validated: ?utterance");
        }
    }

    // perform pragmatic inference
    op:log("debug", "calling prag ...");
    ?utterance = act:applyPragmaticMeaning(?utterance);
    op:log("debug", "prag results: ?utterance");

    // submit dialogue "goal"
    // TODO: it would be nice to just submit a handleUtterance dialogue goal directly
    //       but that complicates dialogue bookkeeping and question asking
    op:log("debug", "submitting dialogue goal");
    !addressee = op:invokeMethod(?utterance, "getAddressee");
    !addressee.act:handleDialogueInput(?utterance);
}

/**
  * Interact with a user to validate an LLM parse of a Utterance.
  * ?unvalidatedUtterance - Parsed Utterance to validate
  * ?updateHistory - whether to update the dialogue history with the Utterance (should only be done on first validation pass)
  * ret ?revalidate - true if the parse was not validated, but still want to engage in parse recovery ("Social LLM" only)
  */
(java.lang.Boolean ?revalidate) = validateParse(Utterance ?unvalidatedUtterance, java.lang.Boolean ?updateHistory) {
    Symbol !speaker;
    Symbol !addressee;
    Utterance !queryUtterance;
    Predicate !querySemantics;
    java.util.List !querySuppSemantics;
    java.util.List !queryBindings;
    java.util.Map !queryTierAssignments;
    edu.tufts.hrilab.slug.common.UtteranceType !type;
    java.lang.String !language;
    java.util.List !boundSemantics;
    java.lang.String !utteranceString;
    Symbol !utteranceSymbol;
    Predicate !response;
    java.util.Map !bindings;
    java.util.List !suspendedQids;
    java.lang.Long !qid;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Utterance !timeoutUtterance;
    java.lang.String !validationUtteranceString;
    java.util.List !responseFormList;
    Predicate !responseForm;
    java.lang.Boolean !validated;
    java.lang.Integer !historyLength;

    ?revalidate = op:newObject("java.lang.Boolean", false);
    !addressee = op:invokeMethod(?unvalidatedUtterance, "getAddressee");
    !speaker = op:invokeMethod(?unvalidatedUtterance, "getSpeaker");

    op:log("debug", "Validating utterance: ?unvalidatedUtterance");

    if (op:equals(?updateHistory,true)) {
        // Update dialogue history explicitly here so that it shows up in any UI element before the validation question is asked or answered
        act:updateDialogueHistory(?unvalidatedUtterance);
    }
    // (Social LLM) Measure current conversation history length in order to know whether validated inputs can be parsed
    !historyLength = tsc:getLLMConversationLength(!speaker,!addressee);

    //// Ask Validation Question ////

    // Create fully populated output utterance to pass to askQuestionFromUtterance so that any given NLG component can
    //  use all available information. <- Looks like SimpleNLG was never set up to handle these semantics
    !queryUtterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", !addressee, !speaker);
    !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
    op:invokeMethod(!queryUtterance, "setType", !type);
    Symbol !semantics = op:invokeMethod(?unvalidatedUtterance, "getSemantics");
    !querySemantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "mean(!speaker, !semantics)");
    op:invokeMethod(!queryUtterance,"setSemantics",!querySemantics);
    !querySuppSemantics = op:invokeMethod(?unvalidatedUtterance,"getSupplementalSemantics");
    op:invokeMethod(!queryUtterance,"setSupplementalSemantics",!querySuppSemantics);
    !queryBindings = op:invokeMethod(?unvalidatedUtterance,"getBindings");
    op:invokeMethod(!queryUtterance,"setBindings",!queryBindings);
    !queryTierAssignments = op:invokeMethod(?unvalidatedUtterance,"getTierAssignments");
    op:invokeMethod(!queryUtterance,"setTierAssignments",!queryTierAssignments);
    !language = op:invokeMethod(?unvalidatedUtterance,"getLanguage");
    op:invokeMethod(!queryUtterance,"setLanguage",!language);

    // Ask the validation question and wait for the response
    !responseFormList = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    !responseForm = op:createPredicate("val(X)");
    op:add(!responseFormList,!responseForm);
    !responseForm = op:createPredicate("validated(X)");
    op:add(!responseFormList,!responseForm);
    !suspendedQids = act:suspendWaitForResponse(!responseFormList,!addressee,false,!speaker);
    //TODO: Empirically tune what this timeout duration should be
    !bindings = !addressee.act:askQuestionFromUtterance(!queryUtterance,!responseFormList,1,120);

    //// Handle Validation question's response (or lack thereof) ////

    // Experimental: Validation Questions time out
    //  Give an indication to remote users looking at conversation history that the question is no longer valid
    //  (assuming local users have left or forgotten the trail of conversation by now)
    if (op:isNull(!bindings)) {
        op:log("warn","[handleSpeechInput] validation question timed out, discarding");
        !timeoutUtterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !addressee);
        !validationUtteranceString = op:invokeMethod(!queryUtterance,"getWordsAsString");
        op:invokeMethod(!timeoutUtterance, "setWords", "(previous disambiguation question \"!validationUtteranceString\" has timed out without being answered)");
        act:updateDialogueHistory(!timeoutUtterance);
        //// (done through QA listener now)
        // Discard current conversation context
        //tsc:endChatContext(!speaker,!addressee);
        return;
    }

    // Gather response and resume any conflicting questions which were originally suspended
    //TODO: don't believe this case should ever be hit anymore - can remove the suspend/resume wrapper if so
    !response = op:get(!bindings, !x);
    if (~op:isEmpty(!suspendedQids)) {
      foreach(!qid : !suspendedQids) {
          act:resumeWaitForResponse(!qid,!addressee);
      }
    }

    if (op:isNull(!response)) {
        // Aborting validation - notify user of state refresh (chat discarded in validation response in LLMServiceProvider)
        op:log("debug", "Utterance validation aborted, not processing utterance");
        act:generateResponse(!speaker, and(ack(okay),forgettingAbout(?actor,attemptedInstruction),letMeKnowHowICanFurtherAssistYou()));
        return;
    } elseif (op:equalsValue(!response, yes())) {
        //  Utterance validated - progress through NLU pipeline or focused context script
        //    - LLM Chat state should have been updated properly in handleValidationResponse
        op:log("debug", "Utterance validated.");
        op:invokeMethod(?unvalidatedUtterance,"setIsValidated", true);
        if (op:lt(!historyLength,3)) {
            // LLM results should only be cached if not the product of an extended/contextual interaction. 3 allotted
            //  messages are 1. instruction, 2. validation Q, 3. validation affirmation.
            act:cacheLLMParse(?unvalidatedUtterance);
        }
    } elseif (op:equalsValue(!response, no())) {
        //TODO: This is an instance of overwriting an utterance's parse and running it back through the pipeline after it has already been processed once - is this okay?
        // Utterance not validated - want to engage in an extended dialogue to attempt to rectify misunderstanding
        //  Call Chat completion again including the users negative validation response
        //      Tool call response (direct parse using additional information) -> set ?revalidate to true and continue outer loop calling this function
        //      Text response (extended recovery dialogue interaction to figure out what is wrong) -> set ?revalidate false, isValidated true, and generateResponse semantics
        ?revalidate = tsc:reattemptParse(?unvalidatedUtterance);
        ?unvalidatedUtterance = act:resolveReferences(?unvalidatedUtterance);
        return;
    } else {
        op:log("error", "Unexpected question response: !response");
        return;
    }
}

/**
  * Once an utterance has been passed through the various "reasoning" bits of
  * the NLU system, this method handle the semantics of the utterance and update dialogue.
  */
() = handleDialogueInput(Utterance ?utterance) {
    Symbol !speaker;
    Symbol !addressee;
    Predicate !semantics;
    Predicate !indirectSemantics;
    Predicate !suppSemantics;
    java.lang.Integer !indirectSemanticsSize;

    op:log("debug", "[handleDialogueInput] entering action script for actor: ?actor");

    act:updateDialogueHistory(?utterance);

    // extract fields from utterance to use existing dialogue scripts
    !speaker = op:invokeMethod(?utterance, "getSpeaker");
    !addressee = op:invokeMethod(?utterance, "getAddressee");
    !semantics = op:invokeMethod(?utterance, "getBoundSemanticsPredicate");
    !indirectSemantics = op:invokeMethod(?utterance, "getBoundIndirectSemanticsPredicate");
    !suppSemantics = op:invokeMethod(?utterance, "getBoundSupplementalSemanticsPredicate");

    // handle direct semantics
    // has the form "semantics(boundOption1, boundOption2, ...)"
    act:handleSemantics(!speaker, !addressee, !semantics, !suppSemantics, direct);

    // handle indirect semantics
    // has the form "indirectSemantics(semantics(boundOption1, boundOption2, ...), semantics(...),...)"
    op:log("debug", "handling indirect semantics: !indirectSemantics");
    !indirectSemanticsSize = op:invokeMethod(!indirectSemantics, "size");

    Predicate !currIndirectSemantics;
    for (!i=0; !i lt !indirectSemanticsSize; !i ++) {
      !currIndirectSemantics = op:invokeMethod(!indirectSemantics, "get", !i);
      act:handleSemantics(!speaker, !addressee, !currIndirectSemantics, !suppSemantics, indirect);
    }
}

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
        }
    }
}

() = cancelContextSpecificChat(Integer ?tracker, Symbol ?user, Symbol ?agentTeam, java.lang.Long ?cid) {
    op:log("info","[cancelContextSpecificChat] in method ?tracker ");
    if (op:gt(?tracker,0)) {
        tsc:cancelLLMChatContext(?user,?agentTeam,?cid);
    }
}

// "Social LLM"
// Main method to transfer to a "focused context" in a conversation between a user and the system. Currently implemented
//  to practically overwrite the System prompt and target Tool set for interactions between a user/AgentTeam pair until
//  an INSTRUCT Tool is called (initially designed towards such a one-off "template" population interaction)
// To get arguments to pass into this method, use:
//  this:getOriginalSubmitter();
//  ?actor
//  this:getCommand();
//  this:getId();
(Predicate ?resultPred) = runContextSpecificChat(Symbol ?user, Symbol ?agentTeam, java.lang.String ?contextId, java.lang.Long ?cid) {
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

    op:log("info","[runContextSpecificChat] in method");

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

    //if (op:equals(!semanticsName,"error")) {
    //    act:handleError(?user, ?agentTeam, !focusedChatInitialResponseSemantics, direct);
    //    tsc:endChatContext(?user,?agentTeam);
    //    exit(FAIL);
    //}
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
    ?resultPred = act:waitOnFocusedChat(?user,?agentTeam,?cid);

    //// If an issue occurred during focusDialogueChat
    //if (op:isNull(?resultPred)) {
    //    exit(FAIL,occurred(internalError));
    //}

    op:log("info", "[runContextSpecificChat] out with result: ?resultPred");
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
    }
}

// Validation of this goal itself will stand in as question inquiring about whether the user intended a goal
//  submission outside of the context of the current focused dialogue interaction.
// If this does not perform well, we can not validate this goal and just explicitly form the question here
/**
  * For "focused" dialogue contexts; action to allow the user to temporarily back out of the specific context into
  * the base, standard "nlu" context to give one off instructions. Providing internal, standard functionality to
  * support this without forcing the user to understand how to explicitly suspend and resume these types of goals in
  * order to give other instructions.
  *
  * TODO: not sure how well the LLM actually handles this target. Needs more testing and development
  *       (May get rid of this)
  */
() = offContextSubmission() {
    Symbol !submitter;
    java.lang.Long !cid;
    String !contextId;
    Predicate !output;
    java.lang.Long !gid;
    String !cmd;
    java.lang.Long !timeout = 5000;

    !submitter = this:getOriginalSubmitter();
    !cid = this:getId();
    !contextId = this:getCommand();

    op:log("info","[offContextSubmission] in method");

    //TODO: shouldn't be able to be canceled/suspended like other focused chats. How to do this?
    //      Using noInterrupt wrapper functionally doesn't allow it to be canceled/suspended, but doesn't report any error on trying to do so.
    //      Though suspend/resume/cancel directly shouldn't be an acceptable dialogue target in this state anyway
    noInterrupt {
        !output = act:runContextSpecificChat(!submitter, ?actor, !contextId, !cid);

        if (op:isNull(!output)) {
            op:log("warn","[offContextSubmission] exception during call for runContextSpecificChat ***TODO HANDLE****");
            return;
        }

        // Wait for submission to complete in any case (new submission has highest priority and/or status reporting of submission and/or dialogue of submission placement)
        op:log("debug","[offContextSubmission] waiting on goal submission for !output");
        !gid = tsc:waitForGoalSubmission(!output, !timeout);
        op:log("debug","[offContextSubmission] submission result: !gid");

        if (op:==(!gid,-1)) {
            op:log("warn","[offContextSubmission] Could not find submitted goal for !output. ***TODO HANDLE****");
            return;
        }

        // If goal is suspend or cancel, need to wait for it to fully complete (should this be the case for any SKIPPENDING goal?)
        !cmd = op:getName(!output);
        if (op:equals(!cmd,"cancelGoal") || op:equals(!cmd,"suspendGoal")) {
            op:log("debug","[offContextSubmission] joining on goal !gid");
            act:joinOnGoal(!gid);
            op:log("debug","[offContextSubmission] joined on goal: !gid");
        }
    }

    // Not sure what this was meant to achieve on review, test with it removed
    //// TODO: Has to be a better way to do this
    //edu.tufts.hrilab.fol.Predicate !suspendGoalPred;
    //edu.tufts.hrilab.fol.Predicate !cancelGoalPred;
    //// TODO: If this changes from "next" to explicitly the superseded, focused dialogue action, then that needs to
    ////      retrieved explicitly here instead as well
    //!gid = tsc:getNextGoalId(?actor);
    //if (~op:==(!gid,-1)) {
    //    op:log("info","[offContextSubmission] next gid !gid");
    //    !suspendGoalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "suspendGoal(?actor,!gid)");
    //    !cancelGoalPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cancelGoal(?actor,!gid)");
    //    !gid = tsc:getGoalId(!suspendGoalPred);
    //    op:log("info","[offContextSubmission] !gid");
    //    if (~op:==(!gid,-1)) {
    //        !gid = tsc:getGoalId(!cancelGoalPred);
    //        op:log("info","[offContextSubmission] !gid");
    //    }
    //    if (~op:==(!gid,-1)) {
    //        op:log("debug","[offContextSubmission] waiting on !gid");
    //        act:joinOnGoal(!gid);
    //        op:log("debug","[offContextSubmission] waited on !gid");
    //    }
    //}

    op:log("debug","[offContextSubmission] out");
}

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

//Empty script to be held as target to abandon base context interaction
() = endConversation() {
    op:log(info,"[endConversation]");

    //TODO: prompt the system to say something in this case? goodbye?
}

() = resetConversation() {
    Symbol !submitter;

    op:log(info,"[endConversation]");

    !submitter = this:getOriginalSubmitter();
    tsc:endChatContext(!submitter,?actor);
    act:generateResponseFromString(!submitter,"Conversation has been reset");
}