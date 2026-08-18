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
    java.lang.Long !uttID = op:invokeMethod(?utterance,"getUid");
    Symbol !addressee = op:invokeMethod(?utterance, "getAddressee");
    java.lang.Boolean !streamingTTS = tsc:usingStreamingTTS(!addressee);
    Utterance !parsedUtterance;

    if (tsc:usingLLM()) {
      !parsedUtterance = tsc:parseUtteranceForNonLLMResult(?utterance, !streamingTTS);
      if (op:isNull(!parsedUtterance)) {
        if (op:equals(!streamingTTS,true)) {
          tsc:unbufferStreamingTTSOutputForUID(!uttID);
        }
        !parsedUtterance = tsc:getLLMParseResult(?utterance, !streamingTTS);
      } else {
        if (op:equals(!streamingTTS,true)) {
          tsc:cancelStreamingTTSOutputForUID(!uttID);
        }
      }
    } else {
      !parsedUtterance = tsc:parseUtterance(?utterance);
    }
    op:log("debug", "parser results: !parsedUtterance");

    act:handleParsedUtterance(!parsedUtterance);
}

() = handleParsedUtterance(Utterance ?utterance) {
    java.lang.Boolean !continueValidation;
    java.lang.Boolean !validated;
    Symbol !speaker;
    Symbol !addressee;

    // parser error handling
    Symbol !semantics = op:invokeMethod(?utterance, "getSemantics");
    // no parse, no specific error semantics set
    if (op:isNull(!semantics)) {
      act:updateDialogueHistory(?utterance);
      !speaker = op:invokeMethod(?utterance, "getSpeaker");
      !addressee = op:invokeMethod(?utterance, "getAddressee");
      !semantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(understand(!addressee,that))");
      !addressee.act:handleError(!speaker, !addressee, !semantics, direct);
      return;
    }
    // no successful parse, specific error semantics returned from HybridParser
    java.lang.String !semanticsName = op:getName(!semantics);
    if (op:equals(!semanticsName,"error")) {
        act:updateDialogueHistory(?utterance);
        !speaker = op:invokeMethod(?utterance, "getSpeaker");
        !addressee = op:invokeMethod(?utterance, "getAddressee");
        act:handleError(!speaker, !addressee, !semantics, direct);
        return;
    }

    // perform reference resolution
    op:log("debug", "calling rr ...");
    ?utterance = act:resolveReferences(?utterance);
    op:log("debug", "rr results: ?utterance");

    // TODO: currently assuming validation is only used alongside the LLM, update if this is not the case
    // If configured to, enter validation loop
    if (tsc:usingValidation() && op:invokeMethod(?utterance, "needsValidation")) {

        // Update dialogue history explicitly here so that it shows up in any UI element before the validation question is asked or answered (otherwise existing
        //    pipeline would wait until the utterance is fully parsed and validated)
        act:updateDialogueHistory(?utterance);

        // (Re)validate attempted parses until the user validates it or fully gives up
        !continueValidation = act:validateParse(?utterance,true);
        while (op:equals(!continueValidation,true)) {
            !continueValidation = act:validateParse(?utterance,false);
        }

        // Exiting the validation loop without the utterance being validated signals failure, otherwise progress through rest of NLU pipeline
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
  * ?firstParse - whether this is the first parse/validation attempt for this specific interaction
  * ret ?revalidate - true if the parse was not validated, but still want to engage in parse recovery ("Social LLM" only)
  */
(java.lang.Boolean ?revalidate) = validateParse(Utterance ?unvalidatedUtterance, java.lang.Boolean ?firstParse) {
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
    java.lang.Integer !historyLength = 1;
    java.lang.Integer !fillerCase = 2;
    java.lang.Boolean !streaming;
    java.lang.String !fillerString;
    java.lang.Long !uttID = op:invokeMethod(?unvalidatedUtterance,"getUid");

    // Pre LLM validation question generation filler modified based on being 1st vs 2nd+ parse attempt for this particular instruction
    if (op:equals(?firstParse,false)) {
      !fillerCase = op:++(!fillerCase);
    }

    op:log("debug", "Validating utterance: ?unvalidatedUtterance");
    ?revalidate = op:newObject("java.lang.Boolean", false);
    !addressee = op:invokeMethod(?unvalidatedUtterance, "getAddressee");
    !speaker = op:invokeMethod(?unvalidatedUtterance, "getSpeaker");

    // (Social LLM) Measure current conversation history length in order to know whether validated inputs can be cached safely. (Multi-turn parses cannot
    //  be safely cached, first turn parses can be)
    if (tsc:supportExtendedParses()) {
        !historyLength = tsc:getLLMConversationLength(!speaker,!addressee);
    }

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
    // Suspend any existing validation questions so only most recent is answered
    !suspendedQids = act:suspendWaitForResponse(!responseFormList,!addressee,false,!speaker);
    //TODO: Empirically tune what this timeout duration should be
    if (tsc:usePreValidationFiller()) {
      !bindings = !addressee.act:askQuestionFromUtterance(!queryUtterance,!responseFormList,1,120,!fillerCase);
    } else {
      !bindings = !addressee.act:askQuestionFromUtterance(!queryUtterance,!responseFormList,1,120);
    }

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
        return;
    }

    // Question was answered or user explicitly declined to answer, gather response and resume any conflicting questions which were originally suspended
    !response = op:get(!bindings, !x);
    if (~op:isEmpty(!suspendedQids)) {
      foreach(!qid : !suspendedQids) {
          act:resumeWaitForResponse(!qid,!addressee);
      }
    }

    if (op:isNull(!response)) {
        // Aborting validation - notify user of conversation state refresh
        op:log("debug", "Utterance validation aborted, not processing utterance");
        // General TODO: consolidate generateResponses which are really meant to be hardcoded, but are/were left as semantics for conceptual different language/multilingual LLM implementations
        act:generateResponse(!speaker, and(ack(okay),forgettingAbout(?actor,attemptedInstruction),letMeKnowHowICanFurtherAssistYou()));
        return;
    } elseif (op:equalsValue(!response, yes())) {
        // Utterance validated - progress through NLU pipeline or focused context script
        op:log("debug", "Utterance validated.");
        op:invokeMethod(?unvalidatedUtterance,"setIsValidated", true);
        if (op:lt(!historyLength,3)) {
            // Social LLM results should only be cached if not the product of an extended/contextual interaction. 3 allotted
            //  messages are 1. instruction, 2. validation Q, 3. validation affirmation.
            act:cacheLLMParse(?unvalidatedUtterance);
        }
    } elseif (op:equalsValue(!response, no())) {
        if (tsc:supportExtendedParses()) {
            // (This block can be written this way without additional checks because currently assuming extended parses are SocialLLM specific, this would have
            //  to be rewritten if other components can handle this)

            // Filler response in between negative validation answer and followup disambiguation question
            if (tsc:useReattemptParseFiller()) {
                !fillerString = tsc:sampleReattemptParseFillerPhrase();
                async {
                    act:generateResponseFromString(!speaker,!fillerString);
                }
            }

            //TODO: This is an instance of overwriting an utterance's parse and running it back through the pipeline after it has already been processed once - is this okay?
            // Utterance not validated - want to engage in an extended dialogue to attempt to rectify misunderstanding
            //  Call Chat completion again including the users negative validation response. Cases based on return value:
            //      Tool call response (new direct parse made using additional information from validation response) -> set ?revalidate to true and continue outer loop calling this function
            //      Text response (enter extended recovery dialogue interaction to figure out what is wrong) -> set ?revalidate false, Utterance.isValidated true, and generateResponse semantics
            !streaming = tsc:usingStreamingTTS(!addressee);
            ?revalidate = tsc:reattemptParse(?unvalidatedUtterance, !streaming);
            //TODO: double check this, believe we need to immediately unbuffer if going with intended back-end impl (or should change API to not need to do this)
            if (op:equals(!streaming,true)) {
              tsc:unbufferStreamingTTSOutputForUID(!uttID);
            }
            ?unvalidatedUtterance = act:resolveReferences(?unvalidatedUtterance);
            return;
        } else {
            // No extended parse / parse recovery in the system, exit out.
            act:generateResponse(!speaker,please(rephrase));
            return;
        }
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
    java.lang.Boolean !validated = op:invokeMethod(?utterance,"isValidated");

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
    act:handleSemantics(!speaker, !addressee, !semantics, !suppSemantics, direct, !validated);

    // handle indirect semantics
    // has the form "indirectSemantics(semantics(boundOption1, boundOption2, ...), semantics(...),...)"
    op:log("debug", "handling indirect semantics: !indirectSemantics");
    !indirectSemanticsSize = op:invokeMethod(!indirectSemantics, "size");

    Predicate !currIndirectSemantics;
    for (!i=0; !i lt !indirectSemanticsSize; !i ++) {
      !currIndirectSemantics = op:invokeMethod(!indirectSemantics, "get", !i);
      act:handleSemantics(!speaker, !addressee, !currIndirectSemantics, !suppSemantics, indirect, !validated);
    }
}
