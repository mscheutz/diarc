/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.pragmatics;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.slug.common.Utterance;

import ai.thinkingrobots.trade.TRADEService;

public interface PragmaticsInterface {

  /**
   * Main entry point for applying pragmatic meaning to both input and output
   * utterances. The rules applied (input prag rules vs prag gen rules) are determined by
   * the information contained in the Utterance instance.
   *
   * INPUT: takes in an Utterance from a parser,
   * perform pragmatic inference on it, and returns the results.
   *
   * OUTPUT: takes in an Utterance from dialogue,
   * perform pragmatic inference on it, and returns the results.
   *
   * @param utterance utterance whose semantics will undergo pragmatic inference
   * @return utterance with pragmatics applies
   */
    @TRADEService
    @Action
    Utterance applyPragmaticMeaning(Utterance utterance);

}
