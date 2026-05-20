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
import edu.tufts.hrilab.tts.cartesia.CartesiaTTSComponent;

import java.util.concurrent.atomic.AtomicBoolean;

public class AvaCartesiaTTSComponent extends CartesiaTTSComponent {

  private final AtomicBoolean pendingRearm = new AtomicBoolean(false);

  @TRADEService
  @Action
  public void armPostStreamRearm() {
    StackTraceElement[] st = new Throwable().getStackTrace();
    StringBuilder caller = new StringBuilder();
    for (int i = 1; i < Math.min(st.length, 6); i++) {
      if (caller.length() > 0) caller.append(" <- ");
      caller.append(st[i].getClassName()).append('.').append(st[i].getMethodName()).append(':').append(st[i].getLineNumber());
    }
    log.info("[armPostStreamRearm] set callers={}", caller.toString());
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
    // TODO: HACK -- duplicate-voicing suppression. After Eric's tool-param reorder (commit
    // 35bfd740d, "Force response to come before templates"), the streaming chunkHandler emits
    // the stillGatheringInformation.response field token-by-token to TTS. The legacy
    // handleToolResponse -> sayText path then ALSO speaks the same content -> the response is
    // voiced twice per turn. We detect the duplicate by matching utterance.words against
    // chatContext.lastResponseString (set in SocialTRLLMServiceProvider.handleTextResponse for
    // every LLM text response). String-equality is a proxy for "this content was streamed";
    // the proper fix is in SocialTRLLMServiceProvider so handleToolResponse for the textResponseTool
    // does not dispatch through sayText (or sets semantics that the dispatcher actually
    // recognises as non-voicing). Until then, do NOT remove this gate: it is the only thing
    // keeping Ava from saying every chitchat reply twice. Also do not move the triggerWW back
    // inside this branch -- the streaming path's onStreamCompleted handles post-drain rearm,
    // and a second triggerWW here would create a double rearm per turn.
    if (lastResponseString != null
            && utterance.getWordsAsString().equals(lastResponseString)
            && utterance.getType() == UtteranceType.REPLY) {
      log.info("[sayText] suppressing duplicate voicing (already streamed): text_len={}", utterance.getWordsAsString().length());
      return true;
    }
    boolean result = sayText(utterance.getWordsAsString(), wait);
    return result;
  }

  @Override
  protected void onStreamStarted() {
    pendingRearm.set(false);
  }

  private static final long POST_DRAIN_REARM_DELAY_MS = 500;

  @Override
  protected void onStreamCompleted() {
    super.onStreamCompleted();
    if (pendingRearm.compareAndSet(true, false)) {
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
