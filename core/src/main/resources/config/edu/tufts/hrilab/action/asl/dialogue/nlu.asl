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
      !semantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(understand(?actor,that))");
      act:handleError(!speaker, !addressee, !semantics, direct);
      return;
    }

    // for LLM parses -- get human validation
    Predicate !querySemantics;
    Predicate !response;
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    if (op:invokeMethod(?utterance, "needsValidation")) {
      op:log("debug", "Validating utterance: ?utterance");
      !addressee = op:invokeMethod(?utterance, "getAddressee");
      !speaker = op:invokeMethod(?utterance, "getSpeaker");
      !querySemantics = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "mean(!speaker, !semantics)");
      !bindings = !addressee.act:askQuestion(!speaker, !querySemantics, val(X));
      !response = op:get(!bindings, !x);

      if (op:equalsValue(!response, yes())) {
        op:log("debug", "Utterance validated.");
        act:cacheParse(?utterance);
      } elseif (op:equalsValue(!response, no())) {
        // ask for human to rephrase their request
        op:log("debug", "Utterance not validated. Asking to rephrase.");
        act:sayText("can you please rephrase");
        return;
      } else {
        op:log("error", "Unexpected question response: !response");
      }
    }

    // perform reference resolution
    op:log("debug", "calling rr ...");
    ?utterance = act:resolveReferences(?utterance);
    op:log("debug", "rr results: ?utterance");

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
