/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.llm;

import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.slug.common.Utterance;

public class LLMParserComponent extends DiarcComponent {

  public LLMParserComponent() {
    super();
  }


  /**
   * Reaches out to the LLMServiceProvider to parse the input Utterance to obtain semantic information.
   *
   * @param input The input Utterance to parse.
   * @return The parsed Utterance with updated semantic information.
   */
  public Utterance parseUtterance(Utterance input) {
  @TRADEService
  @Action
    long t0 = System.nanoTime();
    log.debug("[lat] parser_entry text=\"{}\"", input.getWordsAsString());
    log.info("parseUtterance: " + input.getWordsAsString());

    Utterance output;
    try {
      output = TRADE.getAvailableService(new TRADEServiceConstraints().name("parseUtteranceLLMService").argTypes(Utterance.class)).call(Utterance.class, input);
      long durMs = (System.nanoTime() - t0) / 1_000_000;
      log.debug("[lat] parser_done dur={}ms", durMs);
      return output;
    } catch (TRADEException ex) {
      long durMs = (System.nanoTime() - t0) / 1_000_000;
      log.debug("[lat] parser_done dur={}ms err=trade", durMs);
      log.error("Error calling parseUtteranceLLMService.", ex);
      return input;
    }
  }
}
