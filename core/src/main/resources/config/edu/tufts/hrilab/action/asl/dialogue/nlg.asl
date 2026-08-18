import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import java.lang.Integer;
import java.lang.Long;
import java.lang.Float;
import java.lang.String;
import java.util.List;
import java.util.Map;

/**
  * Given a populated Utterance instance, communicate it using the NLG system.
  * All NLG requests eventually go through this method. There are various convenience
  * methods offered to make various parts of this process easier (e.g., generateResponse, askQuestion).
  */
  //TODO: can likely change filler phrase API to make it more clear, revisit
() = communicateUtterance(Utterance ?utterance, Integer ?fillerPhraseCase = 0) {
    String !text;
    Symbol !addressee;
    java.lang.Long !uttID;
    java.lang.Boolean !streamingTTS;
    java.lang.Boolean !isLocal;
    java.lang.Boolean !performSayText = true;
    Predicate !semantics;
    String !fillerString;
    Utterance !outputUtterance;

    // If addressee is not local, do not need to locally sayText. Just update the dialogue history / chat displays.
    // Similarly, if streaming TTS occurs disjointly, we will not manually retrigger a sayText upon completion of the generation (done below).
    !addressee = op:invokeMethod(?utterance, "getAddressee");
    !isLocal = tsc:isLocalAddressee(!addressee);
    !performSayText = op:newObject("java.lang.Boolean", !isLocal);

    // if utterance does not have words, call NLG actions to turn semantics into words
    if (~op:invokeMethod(?utterance, "hasWords")) {
      // Perform prag
      ?utterance = act:applyPragmaticMeaning(?utterance);

      // Convert semantics to text
      if (tsc:usingLLM()) {
        // Using LLM, disjointly check for valid results from other NLG components and the LLM NLG component. Based on outcome and rest of system configuration,
        //  conditionally generate filler phrases and allow for streaming TTS of LLM results.
        !uttID = op:invokeMethod(?utterance,"getUid");
        !streamingTTS = tsc:usingStreamingTTS(!addressee);

        // Trigger HybridNLGComponent subcomponents and grab non-llm result first
        !outputUtterance = tsc:convertSemanticsToTextForNonLLMResult(?utterance, !streamingTTS);

        // Check non-llm component success
        if (op:isNull(!outputUtterance)) {
          // Failure, using LLM result. Generate filler utterance and unblock streamingTTS

          // TODO: just sample filler phrase in any case and pass into this method directly to keep code cleaner? (though tradeoff would be generating filler even if not reaching the LLM case)
          // Filler Utterance generation
          if (~op:==(?fillerPhraseCase,0)) {
            if (op:==(?fillerPhraseCase,1)) {
              // TentativeAccept filler
              !semantics = op:invokeMethod(?utterance, "getSemantics");
              !fillerString = tsc:sampleTentativeAcceptPhrase(!semantics);
            } elseif (op:==(?fillerPhraseCase,2)) {
              // Validation Q filler
              !fillerString = tsc:samplePreValidationFillerPhrase(?utterance,false);
            } elseif (op:==(?fillerPhraseCase,3)) {
              // Validation Q filler - 2nd+ attempt at parse
              !fillerString = tsc:samplePreValidationFillerPhrase(?utterance,true);
            }
            async {
              act:generateResponseFromString(?speaker,!fillerString);
            }
          }

          // Grab LLM output, unblock streamingTTS and prevent manual sayText trigger if using
          if (op:equals(!streamingTTS,true)) {
            tsc:unbufferStreamingTTSOutputForUID(!uttID);
            !performSayText = op:newObject("java.lang.Boolean", "false");
          }
          !outputUtterance = tsc:getLLMNLGResult(?utterance, !streamingTTS);
        } else {
          // Success, cancel/clear streaming TTS for this utterance and manually sayText below
          if (op:equals(!streamingTTS,true)) {
            tsc:cancelStreamingTTSOutputForUID(!uttID);
          }
        }
      } else {
        // No LLM, no filler, no streaming TTS -> parse utterance, update dialogue history, and manually sayText
        !outputUtterance = tsc:convertSemanticsToText(?utterance);
      }
      op:log("debug", "updating dialogue history..");
      act:updateDialogueHistory(!outputUtterance);
      !text = op:invokeMethod(!outputUtterance,"getWordsAsString");
      op:log("debug", "done updating dialogue history with: !text");
      if (op:equals(!performSayText,true)) {
        act:sayText(!outputUtterance);
      }
    } else {
      op:log("debug", "updating dialogue history..");
      act:updateDialogueHistory(?utterance);
      !text = op:invokeMethod(?utterance,"getWordsAsString");
      op:log("debug", "done updating dialogue history with: !text");
      if (op:equals(!performSayText,true)) {
        act:sayText(?utterance);
      }
    }

    // TODO: in what cases do we want to trigger the WW, only after e.g. questions, LLM text responses during multi-turn parses, itk output, ... or always? (current impl means always)
    if (tsc:usingWWBasedASR(!addressee)) {
      //TODO: revisit this in line with back-end StreamingTTS / WW changes. Ideally just keep to triggerWW call here.
      if (op:equals(!streamingTTS,true)) {
        act:armPostStreamRearm();
      } else {
        act:triggerWW();
      }
      // TODO: Should listening indicator be tied to WW internally, or placed explicitly here?
      if (tsc:usingListeningIndicator(!addressee)) {
        act:hideListeningIndicator();
      }
    }
}

/**
 * Convenience method for communicating an utterance in text form (i.e., words to be spoken).
 */
() = generateResponseFromString(String ?words) {
    op:log("debug", "[generateResponse] words: ?words");

    Symbol !listener;
    !listener = this:getOriginalSubmitter();
    act:generateResponseFromString(!listener,?words);
}

() = generateResponseFromString(Symbol ?addressee, String ?words) {
    op:log("debug", "[generateResponse] addressee: ?addressee words: ?words");

    Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
    UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setWords", ?words);
    act:communicateUtterance(!utterance);
}

/**
 * Convenience method for communicating an utterance in first-order-logic form.
 */
() = generateResponse(Predicate ?semantics) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Symbol !addressee;
    !addressee = this:getOriginalSubmitter();
    act:generateResponse(!addressee,?semantics);
}

() = generateResponse(Symbol ?addressee, Predicate ?semantics) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
    UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setSemantics", ?semantics);
    act:communicateUtterance(!utterance);
}

/**
 * Given a predicate, calls NLG pipeline to get its text from and then submits a say text goal with it.
 * Called from semantics without bindings, makes empty bindings and calls same method, but with bindings
 */
() = generateResponse(Symbol ?addressee, Predicate ?semantics, Symbol ?semanticType) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Utterance !utterance;
    UtteranceType !type;
    if (op:equalsValue(?semanticType, direct)) {
      !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
      !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
      op:invokeMethod(!utterance, "setType", !type);
      op:invokeMethod(!utterance, "setSemantics", ?semantics);
      act:communicateUtterance(!utterance);
    }
}

//Temp PickAndPlace2024
() = generateResponse(Symbol ?addressee, Predicate ?semantics, Symbol ?semanticType, Predicate ?suppSemantics) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Utterance !utterance;
    UtteranceType !type;
    if (op:equalsValue(?semanticType, direct)) {
      !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
      !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
      op:invokeMethod(!utterance, "setType", !type);
      op:invokeMethod(!utterance, "setSemantics", ?semantics);
      op:log(info,"adding supp semantics ?suppSemantics");
      op:invokeMethod(!utterance, "addSupplementalSemantics", ?suppSemantics);
      act:communicateUtterance(!utterance);
    }
}

/**
 * Given a predicate, calls NLG pipeline to get its text from and then calls sayText action.
 * Called from semantics and from generateResponse with empty bindings. Submits to NLG and sayText
 */
() = generateResponse(Symbol ?addressee, Predicate ?semantics, List ?bindings, Symbol ?semanticType) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Utterance !utterance;
    UtteranceType !type;
    if (op:equalsValue(?semanticType, direct)) {
      !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
      !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
      op:invokeMethod(!utterance, "setType", !type);
      op:invokeMethod(!utterance, "setSemantics", ?semantics);
      op:invokeMethod(!utterance, "setBindings", ?bindings);
      act:communicateUtterance(!utterance);
    }
}

(Map ?answerBindings) = askQuestion(Predicate ?questionSemantics, Predicate ?responseForm, long ?attempts=1, long ?duration=-1) {
    Symbol !addressee;
    !addressee = this:getOriginalSubmitter();
    ?answerBindings = act:askQuestion(!addressee, ?questionSemantics, ?responseForm, ?attempts, ?duration);
}

(Map ?answerBindings) = askQuestion(Symbol ?addressee, Predicate ?questionSemantics, Predicate ?responseForm, long ?attempts=1, long ?duration=-1) {
    List !responseForms = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    op:add(!responseForms, ?responseForm);
    ?answerBindings = act:askQuestion(?addressee, ?questionSemantics, !responseForms, ?attempts, ?duration);
}

(Map ?answerBindings) = askQuestionFromString(String ?words, Predicate ?responseForm, long ?attempts=1, long ?duration=-1) {
    Symbol !addressee;
    !addressee = this:getOriginalSubmitter();
    ?answerBindings = act:askQuestionFromString(!addressee, ?words, ?responseForm, ?attempts, ?duration);
}

/**
 * Convenience method for communicating an utterance and then waiting for a response.
 * The question to be asked is specified in first-order-logic form.
 *
 * ?addressee - who to ask the question to
 * ?questionSemantics - semantics of the question to ask
 * ?responseForms - specifies the set of semantic forms that answers can take
 * ?attempts - number of times to re-ask the questions if there's no response within ?duration
 * ?duration - ?duration to wait for response
 */
(Map ?answerBindings) = askQuestion(Symbol ?addressee, Predicate ?questionSemantics, List ?responseForms, long ?attempts=1, long ?duration=-1) {

  // build question utterance
  Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
  UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
  op:invokeMethod(!utterance, "setType", !type);
  op:invokeMethod(!utterance, "setSemantics", ?questionSemantics);

  // ask question and wait for response
  ?answerBindings = act:askQuestionFromUtterance(!utterance, ?responseForms, ?attempts, ?duration);
}

(Map ?answerBindings) = askQuestionFromString(String ?words, List ?responseForms, long ?attempts=1, long ?duration=-1) {
    Symbol !addressee;
    !addressee = this:getOriginalSubmitter();
    ?answerBindings = act:askQuestionFromString(!addressee, ?words, ?responseForms, ?attempts, ?duration);
}

(Map ?answerBindings) = askQuestionFromString(Symbol ?addressee, String ?words, Predicate ?responseForm, long ?attempts=1, long ?duration=-1) {
    List !responseForms = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    op:add(!responseForms, ?responseForm);
    ?answerBindings = act:askQuestionFromString(?addressee, ?words, !responseForms, ?attempts, ?duration);
}

/**
 * Convenience method for communicating an utterance and then waiting for a response.
 * The question to be asked is in text form (i.e., words to be spoken).
 *
 * ?addressee - who to ask the question to
 * ?words - exact words to say in the question
 * ?responseForms - specifies the set of semantic form that answers can take
 * ?attempts - number of times to re-ask the questions if there's no response within ?duration
 * ?duration - ?duration to wait for response
 */
(Map ?answerBindings) = askQuestionFromString(Symbol ?addressee, String ?words, List ?responseForms, long ?attempts=1, long ?duration=-1) {

  // build question utterance
  Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
  UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
  op:invokeMethod(!utterance, "setType", !type);
  op:invokeMethod(!utterance, "setWords", ?words);

  // speak the question text before entering ask/wait loop (communicateUtterance with sayText=false from inside askQuestionFromUtterance won't speak with-words utterances post 5f8f6df24)
  act:sayText(!utterance);

  // ask question and wait for response
  ?answerBindings = act:askQuestionFromUtterance(!utterance, ?responseForms, ?attempts, ?duration);
}

(Map ?answerBindings) = askQuestionFromUtterance(Utterance ?utterance, Predicate ?responseForm, long ?attempts, long ?duration) {
    List !responseForms = op:newArrayList("edu.tufts.hrilab.fol.Predicate");
    op:add(!responseForms, ?responseForm);
    ?answerBindings = act:askQuestionFromUtterance(?utterance, !responseForms, ?attempts, ?duration);
}

/**
 * Ask a question and wait for the response. This take in an Utterance, so users should prefer to use one
 * of the convenience methods instead.
 *
 * ?responseForms - specifies the set of semantic forms that answers can take
 * ?attempts - number of times to re-ask the questions if there's no response within ?duration
 * ?duration - ?duration to wait for response
 **/
(Map ?answerBindings) = askQuestionFromUtterance(Utterance ?utterance, List ?responseForms, long ?attempts, long ?duration, Integer ?fillerPhraseCase=0) {
  //Communicate the utterance ?attempts times (default 1) while waiting for ?duration seconds
  ?answerBindings = op:setNull();
  Long !count = 0;
  Symbol !speaker;
  Symbol !addressee;
  Integer !tracker = 0;
  String !questionString;
  java.lang.Boolean !false = false;

  onInterrupt : {
    resume: act:askQuestionFromUtteranceOnResume(!tracker,?utterance);
  }

  !speaker = op:invokeMethod(?utterance, "getSpeaker");
  !addressee = op:invokeMethod(?utterance, "getAddressee");
  while (op:isNull(?answerBindings) && op:lt(!count, ?attempts)) {
    !count = op:++(!count);

    // communicate utterance (which also updates dialogue history)
    // call asynchronously so that the waitForResponse is called asap so we don't miss any responses
    noInterrupt {
      async {
        // TODO: Removed (previously Ava specific) WW/StreamingTTS interactions from this block. Revisit and make sure changes in communicateUtterance comply
        //  with desired behaviore here appropriately
        act:communicateUtterance(?utterance, ?fillerPhraseCase);
      }
      !tracker = op:newObject("java.lang.Integer", 1);
    }

    // wait for response - default duration is -1, which means block indefinitely
    !questionString = op:invokeMethod(?utterance, "getWordsAsString");
    ?answerBindings = act:waitForResponse(!questionString, ?responseForms, !speaker, !addressee, ?duration);
    !tracker = op:newObject("java.lang.Integer", 2);
  }

  //TODO EW: If "abort" semantics supplied as answer to question, EXIT fail from this script or let caller handle case?
  //If we timed out and did not receive a response, we need to remove the active question ourselves
  if (op:isNull(?answerBindings)) {
    act:cancelWaitForResponse(!questionString, ?responseForms, !speaker, !addressee);
  }
  //TODO:EW: else interrupt looping communicateUtterance we are potentially in if we received a response mid loop?
}

() = askQuestionFromUtteranceOnResume(Integer ?tracker, Utterance ?utterance) {
    if (op:equals(?tracker,1)) {
        act:communicateUtterance(?utterance);
    }
}

() = repeatLastQuestion() {
    java.lang.String !questionString;
    !questionString = tsc:getLastQuestion(?actor);
    if (op:invokeMethod(!questionString,"isEmpty")) {
        act:generateResponse(not(have(?actor,pendingQuestions)));
    } else {
        act:generateResponseFromString(!questionString);
    }
}

/**
  * Generate a "because" predicate based on the provided Justification.
  */
(Predicate ?becausePredicate) = createBecausePredicate(Predicate ?state, Symbol ?addressee, edu.tufts.hrilab.action.justification.Justification ?justification) {
    Predicate !tmp;
    Integer !failureConditionsSize;
    List !failureList;
    List !array;
    Predicate !failureCondition = unknown();
    Predicate !intraExplanation;
    Predicate !query;
    List !bindingsList;
    Map !bindings;
    org.apache.commons.lang3.tuple.Pair !supportExplanation;
    Map !explainMap;
    java.util.Set !terms;
    Symbol !incomplete= "incomplete";

    op:log("debug", "justification = ?justification");
    !failureList = op:invokeMethod(?justification, "getFailureReason");
    op:log("debug", "got failure list: !failureList");

    !bindingsList = op:invokeMethod(?justification, "getBindings");
    op:log("debug", "got bindings list: !bindingsList");

    //check if failure reason is already known by listener
    !array = op:newObject("java.util.ArrayList");

    //only picking first failure condition
    if(~op:isEmpty(!failureList)){
      !failureCondition = op:invokeMethod(!failureList, "get", 0);
    }
    //foreach(!failureCondition : !failureList) {

      if (~op:invokeMethod(!bindingsList, "isEmpty")) {
        !bindings = op:invokeMethod(!bindingsList, "get", 0);
        !failureCondition = op:invokeMethod(!failureCondition, "copyWithNewBindings", !bindings);
      }

      !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "bel",?addressee, !failureCondition);
      if (~act:querySupport(!query)) {

        !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "explanationType",?addressee,!incomplete);
        if (~act:querySupport(!query)) {

          !supportExplanation = act:querySupportWithExplanation(!failureCondition);
          !explainMap = op:invokeMethod(!supportExplanation, "getRight");
          !terms = op:invokeMethod(!explainMap, "keySet");
          foreach(!tmp : !terms) {
            op:log("debug", "spy terms: !tmp");
            if (~op:invokeMethod(!explainMap, "get", !tmp)) {
              !tmp = op:newObject("edu.tufts.hrilab.fol.Predicate", "not", !tmp);
            }

            //check if failure reason is already known by listener
            !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "bel",?addressee, !tmp);
            if (~act:querySupport(!query)) {
              op:invokeMethod(!array, "add", !tmp);
            }
          }
        }

        if (op:invokeMethod(!array, "isEmpty")){
          op:invokeMethod(!array, "add", !failureCondition);
        }
      }
    //}

    !failureConditionsSize = op:invokeMethod(!array, "size");
    op:log("debug", "got size !failureConditionsSize");

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "explanationType",?addressee,incomplete);
    //add action step that failed if novice listener
    if (~act:querySupport(!query)) {
      !query = op:invokeMethod(?justification, "getStep");
      op:log("debug", "justification step = !query");
      if (~op:isNull(!query)) {
        op:log("debug", "got step");
        !intraExplanation = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "pre(infinitive(?state),must(did(!query)))");
      }
    }

    if (~op:isNull(!intraExplanation)) {
      if (op:gt(!failureConditionsSize, 0)) {
        !tmp = op:newObject("edu.tufts.hrilab.fol.Predicate", "and", !array);
        !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and",!intraExplanation,!tmp);
        ?becausePredicate = op:newObject("edu.tufts.hrilab.fol.Predicate", "because", !tmp);
        if (act:isRepeatedPredicate(?becausePredicate, ?addressee, ?state, 1)) {
           !failureCondition = op:invokeMethod(!array, "get", 0);
           !tmp = op:newObject("edu.tufts.hrilab.fol.Predicate", !failureCondition);
           ?becausePredicate = op:newObject("edu.tufts.hrilab.fol.Predicate", "because", !tmp);
           op:log("debug", "predicate repeated");
        }
      } else {
        !tmp = op:newObject("edu.tufts.hrilab.fol.Predicate", !intraExplanation);
        ?becausePredicate = op:newObject("edu.tufts.hrilab.fol.Predicate", "because", !tmp);
      }
    } else {
        if (op:gt(!failureConditionsSize, 1)) {
          !tmp = op:newObject("edu.tufts.hrilab.fol.Predicate", "and", !array);
          ?becausePredicate = op:newObject("edu.tufts.hrilab.fol.Predicate", "because", !tmp);
          
          if (act:isRepeatedPredicate(?becausePredicate, ?addressee, ?state, 1)) {
            !failureCondition = op:invokeMethod(!array, "get", 0);
            !tmp = op:newObject("edu.tufts.hrilab.fol.Predicate", !failureCondition);
            ?becausePredicate = op:newObject("edu.tufts.hrilab.fol.Predicate", "because", !tmp);
            op:log("debug", "predicate repeated");
          }
        } else {
          !failureCondition = op:invokeMethod(!array, "get", 0);
          op:log("debug", "got failure condition");
          !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "!failureCondition");
          //TODO: This doesn't work on the temi/android, the type of args in
          //        createPredicate(String name, Symbol... args) is Predicate rather than Object[] like on desktop
          ?becausePredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "because", !tmp);
        }
      
      //TODO: Ravenna - if no explanation, start mental model conflict resolution?
    }
    op:log("debug", "because: ?becausePredicate");
}
