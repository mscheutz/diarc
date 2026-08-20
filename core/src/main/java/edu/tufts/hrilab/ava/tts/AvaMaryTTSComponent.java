package edu.tufts.hrilab.ava.tts;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.tts.MaryTTSComponent;

import javax.sound.sampled.UnsupportedAudioFileException;
import java.io.IOException;

//TODO: Pulled Ava specific logic out of nlg.asl and created this instead for branch merging purposes.
// Look into how to do this properly (Motivation is not wanting robot to speak locally when a response is meant for remote user)
//In general, where/how do we decide what form of communication occurs based off of the addressee of an utterance and the configuration of a domain?
public class AvaMaryTTSComponent extends MaryTTSComponent {
  public AvaMaryTTSComponent() throws IOException, UnsupportedAudioFileException, InterruptedException {
    super();
  }

  @Override
  public boolean sayText(Utterance utterance) {
    return sayText(utterance,true);
  }


  @Override
  public boolean sayText(Utterance utterance, boolean wait) {
    boolean isLocal = false;
    try {
      isLocal = TRADE.getAvailableService(
                      new TRADEServiceConstraints().name("isLocalAddressee").argTypes(Symbol.class))
              .call(Boolean.class, utterance.getAddressee());
    } catch (TRADEException e) {
      log.warn("[sayText] unable to call isLocalAddressee from FrontEndConfigurationComponent");
    }
    if (!isLocal) {
      boolean result = sayText(utterance.getWordsAsString(), wait);
      // Experimental - automatically trigger WW on any SocialLLM text response call. Will still cause a few unnecessary
      //  "false positive" WW triggers in practice.
      // TODO: make this trigger a "careful" call where background noise heuristics are made more restrictive?
      try {
        // "lastResponseString" set from parseUtteranceLLMService, focusDialogueChat, callChatCompletionForTextResponse, addToolResponse
        String lastResponseString = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastResponseString").argTypes(Symbol.class, Symbol.class)).call(String.class, utterance.getAddressee(), utterance.getSpeaker());
        if (utterance.getWordsAsString().equals(lastResponseString)) {
          if (utterance.getType() == UtteranceType.REPLY) {
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("triggerWW")).call(void.class);
            } catch (TRADEException e) {
              log.error("[updateLLMConversationHistory] unable to call triggerWW on ava component");
            }
          }
        }
      } catch (TRADEException e) {
        log.debug("[sayText] unable to get lastResponseString from SocialLLMServiceProvider - should only occur if SocialLLM is disabled");
      }
      return result;
    } else {
      log.info("published in webapp chat: {}", utterance.getWordsAsString());
      return false;
    }
  }
}
