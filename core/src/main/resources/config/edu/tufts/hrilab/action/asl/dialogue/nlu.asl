import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.slug.common.Utterance;

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
    Symbol !speaker;
    Symbol !addressee;
    Predicate !ack = "ack(okay)";

    op:log("debug", "handleSpeechInput for utterance: ?utterance");

    // parse utterance
    op:log("debug", "calling parser ...");
    ?utterance = act:parseUtterance(?utterance);
    op:log("debug", "parser results: ?utterance");

    // parser error handling
    Symbol !semantics = op:invokeMethod(?utterance, "getSemantics");
    if (op:isNull(!semantics)) {
      act:updateDialogueHistory(?utterance);
      !speaker = op:invokeMethod(?utterance, "getSpeaker");
      !addressee = op:invokeMethod(?utterance, "getAddressee");
      !semantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(understand(!addressee,that))");
      !addressee.act:handleError(!speaker, !addressee, !semantics, direct);
      return;
    }

    // perform reference resolution
    op:log("debug", "calling rr ...");
    ?utterance = act:resolveReferences(?utterance);
    op:log("debug", "rr results: ?utterance");

    // for LLM parses -- get human validation
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
    edu.tufts.hrilab.fol.Variable !x = "X";
    if (op:invokeMethod(?utterance, "needsValidation")) {
      op:log("debug", "Validating utterance: ?utterance");
      !addressee = op:invokeMethod(?utterance, "getAddressee");
      !speaker = op:invokeMethod(?utterance, "getSpeaker");
      act:updateDialogueHistory(?utterance);
      // Create fully populated output utterance to pass to askQuestionFromUtterance so that any given NLG component can
      // use all available information. <- Looks like SimpleNLG was never set up to handle these semantics
      !queryUtterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", !addressee, !speaker);
      !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
      op:invokeMethod(!queryUtterance, "setType", !type);
      !querySemantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "mean(!speaker, !semantics)");
      op:invokeMethod(!queryUtterance,"setSemantics",!querySemantics);
      !querySuppSemantics = op:invokeMethod(?utterance,"getSupplementalSemantics");
      op:invokeMethod(!queryUtterance,"setSupplementalSemantics",!querySuppSemantics);
      !queryBindings = op:invokeMethod(?utterance,"getBindings");
      op:invokeMethod(!queryUtterance,"setBindings",!queryBindings);
      !queryTierAssignments = op:invokeMethod(?utterance,"getTierAssignments");
      op:invokeMethod(!queryUtterance,"setTierAssignments",!queryTierAssignments);
      !language = op:invokeMethod(?utterance,"getLanguage");
      op:invokeMethod(!queryUtterance,"setLanguage",!language);

      //Wrapping question in suspend/resume for val(X) in case of validation of a yes/no answer. Only potential awkwardness
      //  arises if the goal prompting the original question is canceled, then answering the validation question will cause
      //  the validated response to hit nothing.
      act:suspendWaitForResponse(val(X),!addressee,false);
      !bindings = !addressee.act:askQuestionFromUtterance(!queryUtterance, val(X),1,-1);
      !response = op:get(!bindings, !x);
      act:resumeWaitForResponse(val(X),!addressee);

      if (op:isNull(!response)) {
        op:log("debug", "Utterance validation aborted, not processing utterance");
        act:sayText("Okay, I am ignoring the previous command"); //TODO: not sure about this
        return;
      } elseif (op:equalsValue(!response, yes())) {
        op:log("debug", "Utterance validated.");
        act:cacheParse(?utterance);
      } elseif (op:equalsValue(!response, no())) {
        // ask for human to rephrase their request
        op:log("debug", "Utterance not validated. Asking to rephrase.");
        act:sayText("please rephrase");
        return;
      } else {
        op:log("error", "Unexpected question response: !response");
        return;
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

    if (op:invokeMethod(?utterance, "needsValidation")) {
      act:updateDialogueHistoryUtterance(?utterance);
    } else {
      act:updateDialogueHistory(?utterance);
    }

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
