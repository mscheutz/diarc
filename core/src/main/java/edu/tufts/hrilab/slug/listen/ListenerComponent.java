/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.listen;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ListenerComponent extends DiarcComponent {

  private Queue<Utterance> inputSpeechQ = new ArrayDeque();

  /**
   * Utterance queue lock.
   */
  private Lock utteranceLock = new ReentrantLock();
  /**
   * Utterance queue lock condition that is signalled when a new Utterance is received.
   */
  private Condition newUtteranceCondition = utteranceLock.newCondition();

  private List<Symbol> diarcAgents = new ArrayList<>();

  public void populateDiarcAgents() {
    TRADEServiceConstraints getDiarcAgents = new TRADEServiceConstraints().name("getDiarcAgents").argTypes();
    try {
      TRADEServiceInfo service = TRADE.getAvailableService(getDiarcAgents);
      Set<Symbol> diarcAgents = service.call(Set.class);
      this.diarcAgents.addAll(diarcAgents);
    } catch (TRADEException e) {
      log.error("Exception trying to populate listeners via getDiarcAgents.", e);
    }
  }

  @TRADEService
  public void reportRecognizedSpeech(Utterance input) {
    utteranceLock.lock();
    try {
      if (diarcAgents.isEmpty()) {
        populateDiarcAgents();
      }
      input.setListeners(diarcAgents);
      inputSpeechQ.add(input);
      newUtteranceCondition.signalAll();
    } finally {
      utteranceLock.unlock();
    }
  }

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "interruptWaitingForUtterance()")
  public Utterance waitForUtterance() {
    utteranceLock.lock();
    try {
      if (inputSpeechQ.isEmpty()) {
        try {
          newUtteranceCondition.await();
        } catch (InterruptedException e) {
          log.error("Interrupted while waiting for utterance.", e);
        }
      }
      return inputSpeechQ.poll();
    } finally {
      utteranceLock.unlock();
    }
  }

  @TRADEService
  @Action
  public void interruptWaitingForUtterance() {
    utteranceLock.lock();
    try {
      newUtteranceCondition.signalAll();
    } finally {
      utteranceLock.unlock();
    }
  }

  @Override
  protected void shutdownComponent() {
    interruptWaitingForUtterance();
  }
}
