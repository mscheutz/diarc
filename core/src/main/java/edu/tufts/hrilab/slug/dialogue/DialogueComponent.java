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

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class DialogueComponent extends DiarcComponent {

  /**
   * Local bookkeeping of processed utterances and nlg requests.
   */
  private final DiaLog diaLog = new DiaLog();

  private DialogueConsultant consultant;
  private List<TRADEServiceInfo> dialogueHistorySubscribers = new ArrayList<>();
  //TODO: I believe this should live at the level of AgentTeams. At least everything here should be indexed
  //  based on more details of the utterance which represents the asked question.
  //  For now at least differentiating between agent. Previously questions from and answers to different agents step on each other
  private ConcurrentHashMap<Symbol,ConcurrentHashMap<Term, Answer>> currentQuestionsPerAgent = new ConcurrentHashMap<>();
  private ConcurrentHashMap<Symbol,ConcurrentHashMap<Term, Answer>> suspendedQuestionsPerAgent = new ConcurrentHashMap<>();
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

    registerForDiarcAgents();
  }

  private void registerForDiarcAgents() {
    // Register to get notified of DIARC agents. To protect against a race
    // condition, this must happen before trying to get them now.
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
              new TRADEServiceConstraints().returnType(Set.class).name("getDiarcAgents").argTypes(),
              "populateDiarcAgents"
      );
    } catch (TRADEException e) {
      log.error("Failed to request notification for DIARC agents", e);
    }
  }

  @TRADEService
  public void populateDiarcAgents(TRADEServiceInfo service) {
    TRADEServiceInfo diarcAgentsService = service;

    try {
      @SuppressWarnings("unchecked")
      Set<Symbol> agents = diarcAgentsService.call(Set.class);
      agents.forEach(agent -> {
        currentQuestionsPerAgent.put(agent.toUntyped(), new ConcurrentHashMap<>());
        suspendedQuestionsPerAgent.put(agent.toUntyped(), new ConcurrentHashMap<>());
      });
      log.debug("DIARC agents received");
    } catch (TRADEException e) {
      log.error("Error trying to get diarc agents.", e);
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

    //Believe shutdown during Question Asking was causing a thread to hang
    for (Symbol agent : currentQuestionsPerAgent.keySet()) {
      ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(agent);
      ConcurrentHashMap<Term,Answer> suspendedQuestions = suspendedQuestionsPerAgent.get(agent);
      List<Term> questionsToCancel = new ArrayList<>(currentQuestions.keySet());
      if (suspendedQuestions != null) {
        questionsToCancel.addAll(suspendedQuestions.keySet());
      }
      for (Term question : questionsToCancel) {
        cancelWaitForResponse(question,agent);
      }
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
      } catch (Exception e) {
        log.error("Could not make callback to: " + subscriber, e);
        //TODO: what is the scope of TRADEExceptions for which we want to unregister the service?
        if (e.getMessage().startsWith("Service not available any longer") || e.getMessage().equals("Cannot read field \"lock\" because \"tsi\" is null")) {
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
  public boolean isRepeatedPredicate(Predicate word, Symbol addressee, Predicate state, int recency) {
    List<Utterance> history = diaLog.getDialogueHistory();
    if (recency == -1) {
      recency = history.size();
    }
    for (int i = 0; i < recency; i++) {
      List<Symbol> listeners = history.get(i).getListeners();
      for (int j = 0; j < listeners.size(); j++) {
        if (listeners.get(j).equals(addressee)) {
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
          onCancelServiceCall = "cancelWaitForResponse(?responseForm,?speaker)",
          onSuspendServiceCall = "cancelWaitForResponse(?responseForm,?speaker)"
          //onSuspendServiceCall = "suspendWaitForResponse(?responseForm,?speaker,true)",
          //onResumeServiceCall = "resumeWaitForResponse(?responseForm,?speaker)"
  )
  public Map<Variable, Symbol> waitForResponse(Predicate responseForm, Symbol speaker) {
    return waitForResponse(responseForm, speaker,-1);
  }

  /**
   * @param responseForm answer semantics which this method will block until receiving
   * @param waitDuration duration this method will block until giving up. If a negative number, will block indefinitely
   * @return Bindings from the answer, or an empty map if no answer was received in time or the question was overwritten/canceled
   */
  @OnInterrupt(
          onCancelServiceCall = "cancelWaitForResponse(?responseForm,?speaker)",
          onSuspendServiceCall = "cancelWaitForResponse(?responseForm,?speaker)"
          //onSuspendServiceCall = "suspendWaitForResponse(?responseForm,?speaker,true)",
          //onResumeServiceCall = "resumeWaitForResponse(?responseForm,?speaker)"
  )
  @TRADEService
  @Action
  public Map<Variable, Symbol> waitForResponse(Predicate responseForm, Symbol speaker, long waitDuration) {

    //TODO:brad: make this safer
    if(responseForm.getName().equals("pattern")){
      responseForm=(Predicate) responseForm.get(0);
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("activatePattern").argTypes(String.class)).call(Boolean.class,responseForm.getName());
      } catch (TRADEException e) {
        log.error("[waitForResponse]",e);
      }
    }

    speaker = speaker.toUntyped();
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(speaker);
    //add response pattern to data structure
    Answer answer;
    //TODO: the below change requires all calls to waitForResponse to be able to handle empty bindings in response
    //Issue: if any clarification/validation questions expect the same response format as an original question
    //Solution: require the writer of whatever script is triggering this behavior to handle that explicitly? (i.e. by pausing and resuming the original question)
    //TODO: handle this case differently? options:
    //  Continue to hold separate notion of each question and ask the user which question they are responding to when receiving corresponding response
    //  Ask the user to answer the other question first
    //  Ask the user which question they want to keep
    //  Suspend the first question and resume after the new one is answered
    //  ...
    if (currentQuestions.containsKey(responseForm)) {
      log.warn("[waitForResponse] Another question expecting the same response format already exists, returning empty bindings for previous question");
      cancelWaitForResponse(responseForm,speaker);
    }
    answer = new Answer();
    currentQuestions.put(responseForm, answer);

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
        log.error("[waitForResponse] exception waiting for response", e);
      }
    } finally {
      answer.getResponseLock().unlock();
    }

    return answer.getAnswerBindings();
  }

  //TODO: after implementing this and looking at other functions, is everything in this file actually threadsafe?
  //This functionality also is affected by decisions made on how to handle duplicate questions being asked by the system, which needs to be revisited.
  /**
   * @param responseForms List of answer semantics which this method will block until receiving one of
   * @param waitDuration duration this method will block until giving up. If a negative number, will block indefinitely
   * @return Bindings from the corresponding answer, or an empty map if no answer was received in time or any of the questions were overwritten
   */
  @OnInterrupt(
          onCancelServiceCall = "cancelWaitForOneOfResponses(?responseForms,?speaker)",
          onSuspendServiceCall = "cancelWaitForOneOfResponses(?responseForms,?speaker)"
          //onSuspendServiceCall = "suspendWaitForOneOfResponses(?responseForms,?speaker,true)",
          //onResumeServiceCall = "resumeWaitForOneOfResponses(?responseForms,?speaker)"
  )
  @TRADEService
  @Action
  public Map<Variable,Symbol> waitForOneOfResponses(List<Predicate> responseForms, Symbol speaker, long waitDuration) {
    if (responseForms.isEmpty()) {
      log.warn("[waitForOneOfResponses] empty form list supplied");
      return new HashMap<>();
    }

    //joint condition specific to this function which is used to wait for one of the response forms being supplied
    Lock jointResponseLock = new ReentrantLock();
    Condition jointResponseCondition = jointResponseLock.newCondition();
    //thread for each response form
    ExecutorService executor = Executors.newFixedThreadPool(responseForms.size());
    Map<Predicate, Future<Map<Variable,Symbol>>> questionFutures = new HashMap<>();
    for (Predicate form: responseForms) {
      questionFutures.put(form,executor.submit(() -> {
        Map<Variable,Symbol> bindings = waitForResponse(form, speaker, waitDuration);
        //In each thread, you signal the condition if the response is supplied (or if the question is overwritten)
        jointResponseLock.lock();
        jointResponseCondition.signal();
        jointResponseLock.unlock();
        return bindings;
      }));
    }

    jointResponseLock.lock();
    try {
      jointResponseCondition.await();
    } catch (InterruptedException e) {
      log.error("[waitForOneOfResponses] interrupted while waiting on joint response condition", e);
    }
    jointResponseLock.unlock();

    //After the condition is signaled, you get the corresponding bindings and cancel all other questions
    Map<Variable,Symbol> returnBindings = new HashMap<>();
    for (Map.Entry<Predicate,Future<Map<Variable,Symbol>>> questionFuture : questionFutures.entrySet()) {
      if (questionFuture.getValue().isDone()) {
          try {
              returnBindings = questionFuture.getValue().get();
          } catch (InterruptedException | ExecutionException e) {
              log.error("[waitForOneOfResponses] exception getting bindings from completed question future", e);
          }
      } else {
        //This is potentially problematic in deep edge cases depending on solution to users ignoring system questions.
        //If a validation question is asked and the user proceeds to answer an original question where one of the valid
        //  response forms is identical, this would cancel the validation question.
        cancelWaitForResponse(questionFuture.getKey(),speaker);
      }
    }

    executor.shutdown();

    return returnBindings;
  }
  @TRADEService
  @Action
  public void cancelWaitForOneOfResponses(List<Term> responseForm, Symbol listener) {
    if (!responseForm.isEmpty()) {
      cancelWaitForResponse(responseForm.get(0),listener);
    }
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
    listener = listener.toUntyped();
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(listener);
    List<Term> matchingQuestions = currentQuestions.keySet().stream()
            .filter(semantics::instanceOf)
            .collect(Collectors.toList());

    boolean matches = !matchingQuestions.isEmpty();

    //Universal EXIT semantics
    if (!currentQuestions.isEmpty() && semantics.getName().equals("abort")) {
      log.debug("[isQuestionResponse] encountered universal exit semantics");
      matches = true;
    }

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
    listener = listener.toUntyped();
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(listener);
    //get matching question
    List<Term> matchingQuestions = currentQuestions.keySet().stream()
            .filter(answerSemantics::instanceOf)
            .toList();

    //Bind response
    Answer response;
    if (matchingQuestions.size() == 1) {
      //Single match
      Term question = matchingQuestions.get(0);
      response = currentQuestions.remove(question);
      response.setAnswerBindings(question.getBindings(answerSemantics));
    } else if (matchingQuestions.isEmpty()) {
      //No direct match
      //If universal exit semantics, then 'cancel' question with the most recent timestamp
      if (answerSemantics.getName().equals("abort")) {
        //TODO: allow inner semantics to either match specific question or be general "most recent" symbol?
        AtomicReference<Term> question = new AtomicReference<>();
        long maxTimestamp = -1;
        currentQuestions.forEach((key, value) -> {
            if (value.getTimestamp() > maxTimestamp) {
                question.set(key);
            }
        });
        if (question.get() != null) {
          response = currentQuestions.remove(question.get());
          if (response != null) {
            unlockAnswer(response);
          }
          return;
        }
      }
      //TODO:brad clarification request
      log.warn("[answerQuestion] no matching question found for semantics " + answerSemantics + ". If this happens it means there is some weird race condition somewhere and the question was already answered by something else");
      return;
    } else {
      //Multiple matches
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
  public void cancelWaitForResponse(Term responseForm, Symbol listener) {
    listener = listener.toUntyped();
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(listener);
    ConcurrentHashMap<Term,Answer> suspendedQuestions = suspendedQuestionsPerAgent.get(listener);

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
   * @param unlock specify whether to unlock the suspended question's answer (practically causes the corresponding waitForResponse call to unblock if true, and not otherwise)
   */
  @TRADEService
  @Action
  public void suspendWaitForResponse(Term responseForm, Symbol listener, boolean unlock) {
    listener = listener.toUntyped();
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(listener);
    ConcurrentHashMap<Term,Answer> suspendedQuestions = suspendedQuestionsPerAgent.get(listener);

    Answer answer = currentQuestions.remove(responseForm);

    if (answer != null) {
      if (suspendedQuestions.get(responseForm) != null) {
        //TODO: handle this case - may need to index by question or something else. Or not allow this
        log.warn("[suspendWaitForResponse] suspending multiple questions expecting the same response form, only one of these will properly be resumed");
      }
      suspendedQuestions.put(responseForm, answer);
      if (unlock) {
        unlockAnswer(answer);
      }
    }
  }

  /**
   * Put the previously cached answer back into the current questions collection for consideration
   *
   * @param responseForm answer semantics required to match against the corresponding question
   */
  @TRADEService
  @Action
  public void resumeWaitForResponse(Term responseForm, Symbol listener) {
    listener = listener.toUntyped();
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(listener);
    ConcurrentHashMap<Term,Answer> suspendedQuestions = suspendedQuestionsPerAgent.get(listener);

    Answer answer = suspendedQuestions.remove(responseForm);

    if (answer != null) {
      currentQuestions.put(responseForm, answer);
    }
  }

  @TRADEService
  public boolean isAgentWaitingForResponse(Symbol agent) {
    ConcurrentHashMap<Term,Answer> currentQuestions = currentQuestionsPerAgent.get(agent);
    return !(currentQuestions == null || currentQuestions.isEmpty());
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
