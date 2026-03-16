package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.slug.common.Utterance;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.Semaphore;

public class ChatContext {
  Chat history;
  ChatType chatType = ChatType.BASE;
  ChatState chatState = ChatState.PARSE;
  long cid = -1;
  //Maybe lock is external and is just a condition here
  Semaphore contextLock = new Semaphore(0);
  Semaphore postprocessingLock = new Semaphore(0);
  Utterance utteranceUpForValidation = null;
  String contextID = "nlu";
  long submissionTime = -1;

  private static final Logger log = LoggerFactory.getLogger(ChatContext.class);

  public ChatContext(Chat chat) {
    history = chat;
  }

  public ChatContext(Chat chat, long cid) {
    history = chat;
    this.cid = cid;
    chatType = ChatType.FOCUSED;
  }


  public ChatContext(ChatContext other) {
    chatType = other.chatType;
    chatState = other.chatState;
    cid = other.cid;
    utteranceUpForValidation = other.utteranceUpForValidation;
    contextID = other.contextID;
    submissionTime = other.submissionTime;
    history = new Chat(other.history);
  }

  //TODO: swap off semaphore and make so arbitrary # of listeners can wait and be signalled at once properly.
  public void signalCompletion() {
    if (chatType != ChatType.BASE) {
      contextLock.release();

      if (chatType == ChatType.OFFTOPIC) {
        try {
          postprocessingLock.acquire();
        } catch (InterruptedException e) {
          log.error("[ChatContext.singalCompletion] interrupted waiting on postprocessing lock", e);
        }
      }
    }
  }

  public Predicate waitForCompletion() {
    try {
      contextLock.acquire();
    } catch (InterruptedException e) {
      log.error("[ChatContext.waitForCompletion] interrupted waiting on context lock", e);
    }

    Predicate ret = null;
    if (utteranceUpForValidation == null) {
      log.debug("[ChatContext.waitForCompletion] context terminated without validated utterance");
    } else {
      Symbol semantics = utteranceUpForValidation.getSemantics();
      if (semantics == null || !semantics.isPredicate()) {
        log.debug("[ChatContext.waitForCompletion] invalid semantics in utterance");
      } else {
        ret = (Predicate) semantics;
      }
    }

    if (chatType == ChatType.OFFTOPIC) {
      postprocessingLock.release();
    }

    return ret;
  }
}