/*
 * Copyright Thinking Robots, Inc., Tufts University, and others 2026.
 */

package edu.tufts.hrilab.ava.tts;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.tts.cartesia.CartesiaTTSComponent;

public class AvaCartesiaTTSComponent extends CartesiaTTSComponent {

  @Override
  public boolean sayText(Utterance utterance) {
    return sayText(utterance, true);
  }

  @Override
  public boolean sayText(Utterance utterance, boolean wait) {
    if (utterance.getAddressee().getName().equals("webapp")) {
      log.info("published in webapp chat: {}", utterance.getWordsAsString());
      return false;
    }
    boolean result = sayText(utterance.getWordsAsString(), wait);
    try {
      String lastResponseString = TRADE.getAvailableService(
              new TRADEServiceConstraints().name("getLastResponseString").argTypes(Symbol.class, Symbol.class))
              .call(String.class, utterance.getAddressee(), utterance.getSpeaker());
      if (utterance.getWordsAsString().equals(lastResponseString) && utterance.getType() == UtteranceType.REPLY) {
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("triggerWW")).call(void.class);
        } catch (TRADEException e) {
          log.error("[sayText] unable to call triggerWW on ava component");
        }
      }
    } catch (TRADEException e) {
      log.debug("[sayText] unable to get lastResponseString from SocialLLMServiceProvider - should only occur if SocialLLM is disabled");
    }
    return result;
  }

}
