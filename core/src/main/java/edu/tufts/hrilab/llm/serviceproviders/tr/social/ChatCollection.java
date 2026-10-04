package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.slug.common.Utterance;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ChatCollection {
  /**
   * Current ChatContext representing the active dialogue interaction between an agent and a user. Set to null if no
   *  active conversation is underway.
   */
  ChatContext currentChat;
  /**
   * "Focused" Chat contexts only - collection of dialogue interactions relating to a "focused" dialogue interaction
   *  goal which has been suspended
   */
  List<ChatContext> suspendedChats;
  /**
   * Collection of ChatContext histories leading up to instruction of a "focused" dialogue goal. Temporarily stored
   *  here between instruction of the goal and actual submission of the goal. History may contain relevant information
   *  towards population of the "focused" target template.
   */
  Map<String, ChatContext> pendingFocusedChats;

  /**
   * Synchronization objects relevant to keeping context and effects in order when receiving concurrent inputs or
   *  hitting external parsers/NLG components.
   */
  Lock inputLock = new ReentrantLock(true);
  Condition inputCondition = inputLock.newCondition();
  /**
   * Current Utterance being processed under the currentChat context
   */
  Utterance utteranceBeingProcessed = null;
  /**
   * List of Utterance ids which have been signaled as processed without a corresponding start notification. Possible
   *  in the case of external parsers completing before the Utterance reaches the LLMServiceProvider.
   */
  List<Long> processedUttIds = Collections.synchronizedList(new ArrayList<>());

  private static final Logger log = LoggerFactory.getLogger(ChatCollection.class);

  ChatCollection() {
    currentChat = null;
    suspendedChats = new ArrayList<>();
    pendingFocusedChats = new HashMap<>();
  }

  ChatCollection(Chat chat) {
    currentChat = new ChatContext(chat);
    suspendedChats = new ArrayList<>();
    pendingFocusedChats = new HashMap<>();
  }

  public boolean grabInputLock(Utterance incoming) {
    return grabInputLock(incoming, false);
  }

  /**
   * Method to wait for and grab lock to safely start processing an input Utterance and apply side effects to all member
   *  ChatContexts. Note that any member variables may have been modified between the start of and returning from this
   *  call. To release the lock grabbed from this method, signalInputProcessed must be called with the same Utterance
   *  (or simply one with matching Utterance ID). With allowReentry set, a call with the Utterance already being
   *  processed returns immediately without taking another claim.
   * @param incoming Related Utterance to be processed in conjunction with the grabbed input lock
   * @param allowReentry whether a call with the Utterance already being processed may proceed instead of waiting
   * @return shouldProcess - indication of whether signalInputProcessed has already been called with the corresponding
   *  Utterance due to an external parser hit (false == external hit occurred).
   */
  public boolean grabInputLock(Utterance incoming, boolean allowReentry) {
    log.debug("[waitToProcessInput] acquiring input lock");

    boolean shouldProcess;
    inputLock.lock();
    if (processedUttIds.remove(incoming.getUid())) {
      shouldProcess = false;
    } else {
      shouldProcess = true;
      // the WW loop parses an utterance once to check whether it was a false positive, then
      // reports that same utterance. That first parse never reaches a dialogue history
      // update, so nothing releases the lock it took, and waiting on our own id here was
      // seen sitting forever. Letting the same id straight back in is a workaround --
      // arguably that first parse should not be taking this lock in the first place
      if (utteranceBeingProcessed != null && (!allowReentry || utteranceBeingProcessed.getUid() != incoming.getUid())) {
        try {
          inputCondition.await();
          log.debug("[waitToProcessInput] acquired input lock");
        } catch (InterruptedException e) {
          log.error("[ChatContext.waitToProcessInput] interrupted waiting on input lock", e);
          //TODO: shouldProcess or no in this case?
          inputLock.unlock();
          return false;
        }
      }
      utteranceBeingProcessed = incoming;
    }
    inputLock.unlock();
    return shouldProcess;
  }

  public boolean shouldProcess(Utterance utterance) {
    return !processedUttIds.contains(utterance.getUid());
  }

  /**
   * Corresponding unlock method of grabInputLock. Signal that we are done processing the supplied Utterance and applying
   *  relevant side effects to member ChatContexts.
   */
  public void signalInputProcessed(Utterance processed) {
    inputLock.lock();
    if (utteranceBeingProcessed != null && utteranceBeingProcessed.getUid() == processed.getUid()) {
      log.debug("[signalInputProcessed] releasing input lock");
      inputCondition.signal();
      utteranceBeingProcessed = null;
    } else {
      log.debug("[signalInputProcessed] received update for unexpected utterance {}. Adding to processed list", processed);
      processedUttIds.add(processed.getUid());
    }
    inputLock.unlock();
  }

  /**
   * Parameterless unlock method of grabInputLock. Signal that we are done processing the current Utterance and applying
   *  relevant side effects to member ChatContexts. Only use this version of the method if you have guarantees on
   *  lock state and do not have access to the original Utterance.
   */
  public void signalInputProcessed() {
    inputLock.lock();
    if (utteranceBeingProcessed != null) {
      log.debug("[signalInputProcessed] releasing input lock");
      processedUttIds.add(utteranceBeingProcessed.getUid());
      inputCondition.signal();
      utteranceBeingProcessed = null;
    } else {
      log.warn("[signalInputProcessed] called with no existing Utterance being processed");
    }
    inputLock.unlock();
  }

  /**
   * Release all locks for the collection and active context. Only use this method in the case of error recovery.
   */
  protected void releaseLocks() {
    //inputLock.lock();
    //inputCondition.signal();
    //inputLock.unlock();
    signalInputProcessed();
    if (currentChat != null) {
      currentChat.releaseLocks();
    }
  }
}
