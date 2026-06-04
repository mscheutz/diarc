/*
 * Copyright Thinking Robots, Inc., Tufts University, and others 2026.
 */

package edu.tufts.hrilab.ava.tts;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.tts.local.LocalTTSComponent;

import java.util.concurrent.atomic.AtomicBoolean;

public class AvaLocalTTSComponent extends LocalTTSComponent {

  private final AtomicBoolean pendingRearm = new AtomicBoolean(false);

  @TRADEService
  @Action
  public void armPostStreamRearm() {
    log.debug("[armPostStreamRearm] set");
    pendingRearm.set(true);
  }

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
    String lastResponseString = null;
    try {
      lastResponseString = TRADE.getAvailableService(
              new TRADEServiceConstraints().name("getLastResponseString").argTypes(Symbol.class, Symbol.class))
              .call(String.class, utterance.getAddressee(), utterance.getSpeaker());
    } catch (TRADEException e) {
      log.debug("[sayText] unable to get lastResponseString from SocialLLMServiceProvider - should only occur if SocialLLM is disabled");
    }
    boolean lastResponseStreamed = false;
    try {
      lastResponseStreamed = TRADE.getAvailableService(
              new TRADEServiceConstraints().name("getLastResponseStreamed").argTypes(Symbol.class, Symbol.class))
              .call(Boolean.class, utterance.getAddressee(), utterance.getSpeaker());
    } catch (TRADEException e) {
      log.debug("[sayText] unable to get lastResponseStreamed from SocialLLMServiceProvider");
    }
    // Duplicate-voicing gate: streaming chunkHandler emits stillGatheringInformation.response, then handleToolResponse re-dispatches the same text through sayText.
    // load-bearing -- without this Ava speaks every chitchat reply twice. don't move triggerWW back into this branch either (streaming onStreamCompleted handles post-drain rearm).
    boolean result;
    if (lastResponseString != null
            && utterance.getWordsAsString().equals(lastResponseString)
            && utterance.getType() == UtteranceType.REPLY) {
      if (lastResponseStreamed == true) {
        log.info("[sayText] suppressing duplicate voicing (already streamed): text_len={}", utterance.getWordsAsString().length());
        return true;
      } else {
        result = sayText(utterance.getWordsAsString(), wait);
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("triggerWW")).call(void.class);
        } catch (TRADEException e) {
          log.debug("[sayText] triggerWW unavailable");
        }
        return result;
      }
    }
    result = sayText(utterance.getWordsAsString(), wait);
    return result;
  }

  @Override
  protected void onStreamStarted() {
    pendingRearm.set(false);
  }

  private static final long POST_DRAIN_REARM_DELAY_MS = 500;

  @Override
  protected void onStreamCompleted(boolean aborted) {
    super.onStreamCompleted(aborted);
    // On abort the user already barged in with the wake word, which opens its own listen and
    // raises the indicator -- skip the post-drain rearm. The sleep would otherwise hold the
    // shared single-thread executor and delay the interrupt ding queued right behind us.
    if (pendingRearm.compareAndSet(true, false) && !aborted) {
      try {
        Thread.sleep(POST_DRAIN_REARM_DELAY_MS);
      } catch (InterruptedException ie) {
        Thread.currentThread().interrupt();
        return;
      }
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("triggerWW")).call(void.class);
      } catch (TRADEException e) {
        log.debug("[onStreamCompleted] triggerWW unavailable");
      }
    }
  }

}
