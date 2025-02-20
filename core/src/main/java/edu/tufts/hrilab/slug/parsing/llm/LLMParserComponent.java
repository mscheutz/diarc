/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.llm;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.slug.common.Utterance;

public class LLMParserComponent extends DiarcComponent implements NLUInterface {

  public LLMParserComponent() {
    super();
  }


  /**
   * Reaches out to the LLMServiceProvider to parse the input Utterance to obtain semantic information.
   *
   * @param input The input Utterance to parse.
   * @return The parsed Utterance with updated semantic information.
   */
  @Override
  public Utterance parseUtterance(Utterance input) {
    log.info("parseUtterance: " + input.getWordsAsString());

    Utterance output;
    try {
      output = TRADE.getAvailableService(new TRADEServiceConstraints().name("parseUtteranceLLMService").argTypes(Utterance.class)).call(Utterance.class, input);
      return output;
    } catch (TRADEException ex) {
      log.error("Error calling parseUtteranceLLMService.", ex);
      return input;
    }
  }
}
