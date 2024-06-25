/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.dialogue;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.dialogue.gui.DialogueGui;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

@Component
public class DialogueComponent extends DiarcComponent {

  /**
   * Local bookkeeping of processed utterances and nlg requests.
   */
  private final DiaLog diaLog = new DiaLog();

  private DialogueConsultant consultant;
  private List<TRADEServiceInfo> dialogueHistorySubscribers = new ArrayList<>();
  private ConcurrentHashMap<Term, Answer> currentQuestions = new ConcurrentHashMap<>();
  private ConcurrentHashMap<Term, Answer> suspendedQuestions = new ConcurrentHashMap<>();
  private boolean showDialogueGui = false;
  private DialogueGui dialogueGui;

  @Override
  protected void init() {
    if (showDialogueGui) {
      dialogueGui = new DialogueGui(diaLog);
    }

    consultant = new DialogueConsultant(diaLog);
    try {
      Collection<String> consultantGroups= this.getMyGroups();
      consultantGroups.add(consultant.getKBName());
      TRADE.registerAllServices(consultant, consultantGroups);
    } catch (TRADEException e) {
      log.error("[init] exception registering dialogue consultant");
    }
  }

  @Override
  public void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("g")) {
      showDialogueGui = true;
    }
  }

  @Override
  public List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("g").longOpt("gui").desc("show dialogue gui").build());
    return options;
  }

  @Override
  protected void shutdownComponent() {
    try {
      TRADE.deregister(consultant);
    } catch (TRADEException e) {
      log.error("[shutdownComponent] error trying to deregister dialogue consultant", e);
    }
  }

  @TRADEService
  @Action
  public void updateDialogueHistory(Utterance utt) {
    diaLog.addHistory(utt);
    notifyDialogueHistorySubscribers(utt);
  }

  @TRADEService
  public List<Utterance> getDialogueHistory() {
    return diaLog.getDialogueHistory();
  }

  private void notifyDialogueHistorySubscribers(Utterance utt) {
    List<TRADEServiceInfo> servicesToRemove = new ArrayList<>();
    for (TRADEServiceInfo subscriber : dialogueHistorySubscribers) {
      try {
        subscriber.call(void.class, utt);
      } catch (TRADEException e) {
        log.error("Could not make callback to: " + subscriber, e);
        //TODO: what is the scope of TRADEExceptions for which we want to unregister the service?
        if (e.getMessage().startsWith("Service not available any longer")) {
          servicesToRemove.add(subscriber);
        }
      }
    }

    //If a remote container has gone down without unregistering for notifications,
    // then remove the service rather than continuously log errors.
    for (TRADEServiceInfo service: servicesToRemove) {
      log.info("[notifyDialogueHistorySubscribers] unregistering unavailable service from dialogue history notifications ");
      unregisterForDialogueHistoryNotifications(service);
    }
  }

  @TRADEService
  public void registerForDialogueHistoryNotifications(TRADEServiceInfo callback) {
    dialogueHistorySubscribers.add(callback);
  }

  @TRADEService
  public void unregisterForDialogueHistoryNotifications(TRADEServiceInfo callback) {
    dialogueHistorySubscribers.remove(callback);
  }

  @TRADEService
  @Action
  public boolean isRepeatedPredicate(Predicate word, Symbol listener, Predicate state, int recency) {
    List<Utterance> history = diaLog.getDialogueHistory();
    if (recency == -1) {
      recency = history.size();
    }
    for (int i = 0; i < recency; i++) {
      List<Symbol> listeners = history.get(i).getListeners();
      for (int j = 0; j < listeners.size(); j++) {
        if (listeners.get(j).equals(listener)) {
          log.debug("Listener Matched: " + listeners.get(j).getName());
          String semanticString = history.get(i).getSemantics().toString();
          String predicateWord = word.toString();
          String predicateState = state.toString();
          if (semanticString.contains(predicateWord) && semanticString.contains(predicateState)) {
            log.debug("Repeat Found!");
            return true;
          }
        }
      }
    }

    log.debug("Repeat Not Found!");
    return false;
  }

  @TRADEService
  @Action
  @OnInterrupt(
          onCancelServiceCall = "cancelWaitForResponse(?responseForm)",
          onSuspendServiceCall = "suspendWaitForResponse(?responseForm)",
          onResumeServiceCall = "resumeWaitForResponse(?responseForm)"
  )
  public Map<Variable, Symbol> waitForResponse(Predicate responseForm) {
    return waitForResponse(responseForm, -1);
  }

  /**
   * @param responseForm answer semantics which this method will block until receiving
   * @param waitDuration duration this method will block until giving up. If a negative number, will block indefinitely
   * @return Bindings from the answer, or null if no answer was received in time
   */
  @OnInterrupt(
          onCancelServiceCall = "cancelWaitForResponse(?responseForm)",
          onSuspendServiceCall = "suspendWaitForResponse(?responseForm)",
          onResumeServiceCall = "resumeWaitForResponse(?responseForm)"
  )
  @TRADEService
  @Action
  public Map<Variable, Symbol> waitForResponse(Predicate responseForm, long waitDuration) {

    //TODO:brad: make this safer
    if(responseForm.getName().equals("pattern")){
      responseForm=(Predicate) responseForm.get(0);
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("activatePattern").argTypes(String.class)).call(Boolean.class,responseForm.getName());
      } catch (TRADEException e) {
        log.error("[waitForResponse]",e);
      }
    }

    //add response pattern to data structure
    Answer answer;
    if (!currentQuestions.containsKey(responseForm)) {
      answer = new Answer();
      currentQuestions.put(responseForm, answer);
    } else {
      //This should only happen upon resuming after suspension, we do not currently handle concurrent duplicate questions
      answer = currentQuestions.get(responseForm);
    }

    //wait for answer
    try {
      answer.getResponseLock().lock();
      try {
        if (waitDuration > 0) {
          answer.getResponseCondition().await(waitDuration, TimeUnit.SECONDS);
        } else {
          answer.getResponseCondition().await();
        }
      } catch (InterruptedException e) {
        log.error("[askQuestion] exception waiting for response", e);
      }
    } finally {
      answer.getResponseLock().unlock();
    }

    return answer.getAnswerBindings();
  }


  /**
   * Determines if the semantics of a given utterance are an answer to any currently active questions
   *
   * @param speaker   currently not used
   * @param listener  currently not used
   * @param semantics answer semantics provided by the interlocutor
   * @return
   */
  @TRADEService
  @Action
  public boolean isQuestionResponse(Symbol speaker, Symbol listener, Term semantics) {
    List<Term> matchingQuestions = currentQuestions.keySet().stream()
            .filter(semantics::instanceOf)
            .collect(Collectors.toList());

    boolean matches = !matchingQuestions.isEmpty();
    log.debug("[isQuestionResponse] semantics: " + semantics + " matches: " + matches);
    return matches;
  }


  //TODO:brad the logic in this primitive should be moved to some sort of action selector
  // that can issue goals/plans to ask the appropriate clarification questions, and also
  // decide which question a given answer is for.

  /**
   * Search currently 'active' questions from one that matches answerSemantics.
   * Get the relevant bindings from the answer, save them and unlock the question thread.
   *
   * @param speaker         currently not used
   * @param listener        currently not used
   * @param answerSemantics answer semantics provided by the interlocutor
   */
  @TRADEService
  @Action
  public synchronized void answerQuestion(Symbol speaker, Symbol listener, Term answerSemantics) {

    //get matching question
    List<Term> matchingQuestions = currentQuestions.keySet().stream()
            .filter(answerSemantics::instanceOf)
            .toList();

    //Bind response
    Answer response;
    if (matchingQuestions.size() == 1) {
      Term question = matchingQuestions.get(0);
      response = currentQuestions.remove(question);
      response.setAnswerBindings(question.getBindings(answerSemantics));
    } else if (matchingQuestions.isEmpty()) {
      //TODO:brad clarification request
      log.warn("[answerQuestion] no matching question found for semantics " + answerSemantics + ". If this happens it means there is some weird race condition somewhere and the question was already answered by something else");
      return;
    } else {
      //TODO:brad: we should probably have it ask a follow up question if it's unclear what's going on here.
      log.warn("[answerQuestion] unclear which question is being responded to, using the first one... ");
      Term question = matchingQuestions.get(0);
      response = currentQuestions.remove(question);
      response.setAnswerBindings(question.getBindings(answerSemantics));
    }

    unlockAnswer(response);
  }

  /**
   * Remove the answer corresponding to the provided semantics from the current or suspended question collections and
   * signal the answer condition
   *
   * @param responseForm answer semantics required to match against the corresponding question
   */
  @TRADEService
  @Action
  public void cancelWaitForResponse(Term responseForm) {
    Answer answer = currentQuestions.remove(responseForm);

    if (answer == null) {
      answer = suspendedQuestions.remove(responseForm);
    }

    if (answer != null) {
      unlockAnswer(answer);
    }
  }

  /**
   * Transfer the answer corresponding to the provided semantics from the current question collection to the suspended
   * one and signal the answer condition
   *
   * @param responseForm answer semantics required to match against the corresponding question
   */
  @TRADEService
  @Action
  public void suspendWaitForResponse(Term responseForm) {
    Answer answer = currentQuestions.remove(responseForm);

    if (answer != null) {
      suspendedQuestions.put(responseForm, answer);
      unlockAnswer(answer);
    }
  }

  /**
   * Put the previously cached answer back into the current questions collection for consideration
   *
   * @param responseForm answer semantics required to match against the corresponding question
   */
  @TRADEService
  @Action
  public void resumeWaitForResponse(Term responseForm) {
    Answer answer = suspendedQuestions.remove(responseForm);

    if (answer != null) {
      currentQuestions.put(responseForm, answer);
    }
  }

  private void unlockAnswer(Answer answer) {
    answer.getResponseLock().lock();
    try {
      answer.getResponseCondition().signalAll();
    } finally {
      answer.getResponseLock().unlock();
    }
  }
}
