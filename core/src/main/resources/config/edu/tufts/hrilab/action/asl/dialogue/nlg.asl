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
() = communicateUtterance(Utterance ?utterance) {
    String !text;

    // if utterance does not have words, call NLG actions to turn
    // semantics into words
    if (~op:invokeMethod(?utterance, "hasWords")) {
      // utterance has semantics that needs to be converted to words
      op:log("debug", "submitting dialogue goal");
      ?utterance = act:applyPragmaticMeaning(?utterance);
      ?utterance = act:convertSemanticsToText(?utterance);
    }

    // update dialogue bookkeeping
    op:log("debug", "updating dialogue history..");
    act:updateDialogueHistory(?utterance);
    op:log("debug", "done updating dialogue history");

    // get text from utterance, and say it
    !text = op:invokeMethod(?utterance, "getWordsAsString");
    op:log("debug", "calling sayText: !text");
    act:sayText(!text);
}

/**
 * Convenience method for communicating an utterance in text form (i.e., words to be spoken).
 */
() = generateResponseFromString(String ?words) {
    op:log("debug", "[generateResponse] words: ?words");

    Symbol !listener = person; // TODO: how to set this in general?
    Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !listener);
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

    Symbol !listener = person; // TODO: how to set this in general?
    Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, !listener);
    UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
    op:invokeMethod(!utterance, "setType", !type);
    op:invokeMethod(!utterance, "setSemantics", ?semantics);
    act:communicateUtterance(!utterance);
}

/**
 * Given a predicate, calls NLG pipeline to get its text from and then submits a say text goal with it.
 * Called from semantics without bindings, makes empty bindings and calls same method, but with bindings
 */
() = generateResponse(Symbol ?listener, Predicate ?semantics, Symbol ?semanticType) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Utterance !utterance;
    UtteranceType !type;
    if (op:equalsValue(?semanticType, direct)) {
      !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?listener);
      !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
      op:invokeMethod(!utterance, "setType", !type);
      op:invokeMethod(!utterance, "setSemantics", ?semantics);
      act:communicateUtterance(!utterance);
    }
}

/**
 * Given a predicate, calls NLG pipeline to get its text from and then calls sayText action.
 * Called from semantics and from generateResponse with empty bindings. Submits to NLG and sayText
 */
() = generateResponse(Symbol ?listener, Predicate ?semantics, List ?bindings, Symbol ?semanticType) {
    op:log("debug", "[generateResponse] semantics: ?semantics");

    Utterance !utterance;
    UtteranceType !type;
    if (op:equalsValue(?semanticType, direct)) {
      !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?listener);
      !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "REPLY");
      op:invokeMethod(!utterance, "setType", !type);
      op:invokeMethod(!utterance, "setSemantics", ?semantics);
      op:invokeMethod(!utterance, "setBindings", ?bindings);
      act:communicateUtterance(!utterance);
    }
}

/**
 * Convenience method for communicating an utterance and then waiting for a response.
 * The question to be asked is specified in first-order-logic form.
 *
 * ?addressee - who to ask the question to
 * ?questionSemantics - semantics of the question to ask
 * ?responseForm - specifies the semantic form that answers need to take
 * ?attempts - number of times to re-ask the questions if there's no response within ?duration
 * ?duration - ?duration to wait for response
 */
(Map ?answerBindings) = askQuestion(Symbol ?addressee, Predicate ?questionSemantics, Predicate ?responseForm, long ?attempts=1, long ?duration=-1) {

  // build question utterance
  Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
  UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
  op:invokeMethod(!utterance, "setType", !type);
  op:invokeMethod(!utterance, "setSemantics", ?questionSemantics);

  // ask question and wait for response
  ?answerBindings = act:askQuestionFromUtterance(!utterance, ?responseForm, ?attempts, ?duration);
}

/**
 * Convenience method for communicating an utterance and then waiting for a response.
 * The question to be asked is in text form (i.e., words to be spoken).
 *
 * ?addressee - who to ask the question to
 * ?words - exact words to say in the question
 * ?responseForm - specifies the semantic form that answers need to take
 * ?attempts - number of times to re-ask the questions if there's no response within ?duration
 * ?duration - ?duration to wait for response
 */
(Map ?answerBindings) = askQuestionFromString(Symbol ?addressee, String ?words, Predicate ?responseForm, long ?attempts=1, long ?duration=-1) {

  // build question utterance
  Utterance !utterance = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.Utterance", "createOutputUtterance", ?actor, ?addressee);
  UtteranceType !type = op:invokeStaticMethod("edu.tufts.hrilab.slug.common.UtteranceType", "valueOf", "QUESTION");
  op:invokeMethod(!utterance, "setType", !type);
  op:invokeMethod(!utterance, "setWords", ?words);

  // ask question and wait for response
  ?answerBindings = act:askQuestionFromUtterance(!utterance, ?responseForm, ?attempts, ?duration);
}

/**
 * Ask a question and wait for the response. This take in an Utterance, so users should prefer to use one
 * of the convenience methods instead.
 *
 * ?responseForm - specifies the semantic form that answers need to take
 * ?attempts - number of times to re-ask the questions if there's no response within ?duration
 * ?duration - ?duration to wait for response
 **/
(Map ?answerBindings) = askQuestionFromUtterance(Utterance ?utterance, Predicate ?responseForm, long ?attempts, long ?duration) {
  //Communicate the utterance ?attempts times (default 1) while waiting for ?duration seconds
  ?answerBindings = op:setNull(); // require bc of the weird way ASL handles return args
  Long !count = 0;
  while (op:isNull(?answerBindings) && op:lt(!count, ?attempts)) {
    !count = op:++(!count);

    // communicate utterance (which also updates dialogue history)
    // call asynchronously so that the waitForResponse is called asap so we don't miss any responses
    async {
      act:communicateUtterance(!utterance);
    }

    // wait for response - default duration is -1, which means block indefinitely
    ?answerBindings = act:waitForResponse(?responseForm, ?duration);
  }

  //If we timed out and did not receive a response, we need to remove the active question ourselves
  if (op:isNull(?answerBindings)) {
    act:cancelWaitForResponse(?responseForm);
  }
  //TODO:EW: else interrupt looping communicateUtterance we are potentially in if we received a response mid loop?
}

/**
  * Generate a "because" predicate based on the provided Justification.
  */
(Predicate ?becausePredicate) = createBecausePredicate(Predicate ?state, Symbol ?listener, edu.tufts.hrilab.action.justification.Justification ?justification) {
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

      !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "bel",?listener, !failureCondition);
      if (~act:querySupport(!query)) {

        !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "explanationType",?listener,!incomplete);
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
            !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "bel",?listener, !tmp);
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

    !query = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "explanationType",?listener,incomplete);
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
        if (act:isRepeatedPredicate(?becausePredicate, ?listener, ?state, 1)) {
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
          
          if (act:isRepeatedPredicate(?becausePredicate, ?listener, ?state, 1)) {
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
