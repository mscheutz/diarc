/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.dialogue;

import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.fol.Term;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class DiaLog {

  // *************************************************
  // **************** Fields *************************
  // *************************************************

  private final List<Utterance> dialogueHistory;
  // TODO: EAK: why is this a priority queue instead of FIFO
  private final PriorityQueue<Utterance> nlInputs;
  // TODO: EAK: why is this a priority queue instead of FIFO
  private final Queue<Utterance> nlgRequests;
  private final List<List<Term>> offRecSemanticHistory;
  private final List<List<Term>> semanticHistory;

  private Lock nlInputLock = new ReentrantLock();
  private Condition nlInputCondition = nlInputLock.newCondition();

  private static Logger log = LoggerFactory.getLogger(DiaLog.class);

  // *************************************************
  // **************** Constructors *******************
  // *************************************************

  public DiaLog() {
    dialogueHistory = new ArrayList<>();
    nlInputs = new PriorityQueue<>();
    offRecSemanticHistory = new ArrayList<>();
    semanticHistory = new ArrayList<>();
    nlgRequests = new PriorityQueue<>();
  }

  // *************************************************
  // **************** Getters ************************
  // *************************************************

  public List<Utterance> getDialogueHistory() {
    return dialogueHistory;
  }

  public boolean isRequestsEmpty() {
    return nlgRequests.isEmpty();
  }

  public List<List<Term>> getOffRecSemanticHistory() {
    return offRecSemanticHistory;
  }

  public List<List<Term>> getSemanticHistory() {
    return semanticHistory;
  }

  public boolean isInputsEmpty() {
    try {
      nlInputLock.lock();
      return nlInputs.isEmpty();
    } finally {
      nlInputLock.unlock();
    }
  }

  public Utterance pollInputs() {
    try {
      nlInputLock.lock();
      return nlInputs.poll();
    } finally {
      nlInputLock.unlock();
    }
  }

  // *************************************************
  // **************** Adders *************************
  // *************************************************

  public void addHistory(Utterance u) {
    dialogueHistory.add(u);
  }

  public void addNLInput(Utterance input) {
    try {
      nlInputLock.lock();
      nlInputs.add(input);
    } finally {
      nlInputLock.unlock();
    }
  }

  public void addNLGRequest(Utterance request) {
    synchronized (nlgRequests) {
      log.debug("Adding nlgRequest: " + request);
      nlgRequests.add(request);
    }
  }

  public void addOffRecSemanticHistory(List<Term> orsm) {
    offRecSemanticHistory.add(orsm);
  }

  public void addSemanticHistory(List<Term> t) {
    semanticHistory.add(t);
  }

}
