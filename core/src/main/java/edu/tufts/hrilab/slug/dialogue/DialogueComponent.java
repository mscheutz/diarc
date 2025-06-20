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
import edu.tufts.hrilab.fol.Factory;
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
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class DialogueComponent extends DiarcComponent {

  /**
   * Local bookkeeping of processed utterances and nlg requests.
   */
  private final DiaLog diaLog = new DiaLog();

  private DialogueConsultant consultant;
  private Set<TRADEServiceInfo> dialogueHistorySubscribers = new HashSet<>();
  private Set<TRADEServiceInfo> questionAskingSubscribers = new HashSet<>();
  //I believe this should live at the level of AgentTeams.
  private ConcurrentHashMap<Symbol,Set<Question>> questionsPerAgent = new ConcurrentHashMap<>();
  private Map<Symbol,Set<Symbol>> agentHierarchy = new HashMap<>();

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
        questionsPerAgent.put(agent.toUntyped(), new HashSet<>());
        questionsPerAgent.put(agent.toUntyped(), new HashSet<>());
      });
      log.debug("DIARC agents received");

      agentHierarchy = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllAgentTeams")).call(Map.class);
      if (agentHierarchy != null) {
        //Strip type information in order to more easily match
        // against actors in below functions
        Map<Symbol, Set<Symbol>> untypedAgentHierarchy = new HashMap<>();
        for (Map.Entry<Symbol, Set<Symbol>> agentTeam : agentHierarchy.entrySet()) {
          Symbol teamName = agentTeam.getKey();
          teamName = Factory.createSymbol(teamName.toUntypedString());
          Set<Symbol> teamMembers = agentTeam.getValue();
          Set<Symbol> untypedTeamMembers = new HashSet<>();
          for (Symbol teamMember : teamMembers) {
            untypedTeamMembers.add(Factory.createSymbol(teamMember.toUntypedString()));
          }
          untypedAgentHierarchy.put(teamName, untypedTeamMembers);
        }
        //TODO Revert this, fix this properly.
        for (Symbol agent : untypedAgentHierarchy.get(Factory.createSymbol("self"))) {
          if (!agent.getName().equals("self")) {
            Set<Symbol> members = new HashSet<>();
            members.add(agent);
            untypedAgentHierarchy.put(agent, members);
          }
        }
        agentHierarchy = untypedAgentHierarchy;
      } else {
        agentHierarchy = new HashMap<>();
      }
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
    for (Symbol agent : questionsPerAgent.keySet()) {
      Set<Question> currentQuestions = questionsPerAgent.get(agent);
      for (Question question : currentQuestions) {
        question.endWaitForResponse();
      }
    }
  }

  @TRADEService
  @Action
  public void updateDialogueHistory(Utterance utt) {
    updateDialogueHistory(utt,true);
  }

  @TRADEService
  @Action
  public void updateDialogueHistory(Utterance utt, boolean newUtterance) {
    if (newUtterance) {
      diaLog.addHistory(utt);
    } else {
      diaLog.updateHistory(utt);
    }
    notifyDialogueHistorySubscribers(utt, newUtterance);
  }

  @TRADEService
  public List<Utterance> getDialogueHistory() {
    return diaLog.getDialogueHistory();
  }

  private void notifyDialogueHistorySubscribers(Utterance utt, boolean newUtterance) {
    List<TRADEServiceInfo> servicesToRemove = new ArrayList<>();
    for (TRADEServiceInfo subscriber : dialogueHistorySubscribers) {
      try {
        subscriber.call(void.class, utt, newUtterance);
      } catch (Exception e) {
        log.error("Could not make callback to: " + subscriber, e);
        //TODO: what is the scope of TRADEExceptions for which we want to unregister the service?
        if (e.getMessage() == null || e.getMessage().startsWith("Service not available any longer") || e.getMessage().equals("Cannot read field \"lock\" because \"tsi\" is null")) {
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

  //TODO: temp: Added for kioskGoalManager functionality - can remove if reimplemented
  @TRADEService
  public void registerForDialogueHistoryNotifications(TRADEServiceInfo callback) {
    if (dialogueHistorySubscribers.stream().anyMatch(c-> c.matches(callback))) {
      log.warn("[registerForDialogueHistoryNotifications] attempt to register duplicate callback, not adding");
    } else {
      dialogueHistorySubscribers.add(callback);
    }
  }

  @TRADEService
  public void unregisterForDialogueHistoryNotifications(TRADEServiceInfo callback) {
    dialogueHistorySubscribers.remove(callback);
  }

  private void notifyQuestionAskingSubscribers(Symbol agent, boolean active) {
    List<TRADEServiceInfo> servicesToRemove = new ArrayList<>();
    for (TRADEServiceInfo subscriber : questionAskingSubscribers) {
      try {
        subscriber.call(void.class, agent, active);
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
      log.info("[notifyQuestionAskingSubscribers] unregistering unavailable service from question asking notifications ");
      unregisterForQuestionAskingNotifications(service);
    }
  }

  @TRADEService
  public void registerForQuestionAskingNotifications(TRADEServiceInfo callback) {
    if (questionAskingSubscribers.stream().anyMatch(c-> c.matches(callback))) {
      log.warn("[registerForQuestionAskingNotifications] attempt to register duplicate callback, not adding");
    } else {
      questionAskingSubscribers.add(callback);
    }
  }

  @TRADEService
  public void unregisterForQuestionAskingNotifications(TRADEServiceInfo callback) {
    questionAskingSubscribers.remove(callback);
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
          onCancelServiceCall = "cancelWaitForResponse(?questionString,?responseForms,?speaker)",
          onSuspendServiceCall = "cancelWaitForResponse(?questionString,?responseForms,?speaker)"
  )
  public Map<Variable, Symbol> waitForResponse(String questionString, List<Term> responseForms, Symbol speaker) {
    return waitForResponse(questionString, responseForms, speaker,-1);
  }

  /**
   * @param questionString String corresponding to question which was asked
   * @param responseForms List of possible semantic forms which are considered answers to the question
   * @param speaker The agent which had asked the question
   * @param waitDuration duration this method will block until giving up. If a negative number, will block indefinitely
   * @return Bindings from the answer, or an empty map if no answer was received in time or the question was overwritten/canceled
   */
  @OnInterrupt(
          onCancelServiceCall = "cancelWaitForResponse(?questionString,?responseForms,?speaker)",
          onSuspendServiceCall = "cancelWaitForResponse(?questionString,?responseForms,?speaker)"
  )
  @TRADEService
  @Action
  public Map<Variable, Symbol> waitForResponse(String questionString, List<Term> responseForms, Symbol speaker, long waitDuration) {
    if (responseForms.isEmpty()) {
      log.warn("[waitForOneOfResponses] empty form list supplied");
      return new HashMap<>();
    }

    speaker = speaker.toUntyped();
    Question question = new Question(questionString, responseForms);
    Set<Question> currentQuestions = questionsPerAgent.get(speaker);
    if (currentQuestions == null) {
      log.warn("[waitForResponse] unexpected speaker encountered {}", speaker);
      questionsPerAgent.put(speaker, Set.of(question));
      currentQuestions = questionsPerAgent.get(speaker);
    } else {
      currentQuestions.add(question);
    }

    notifyQuestionAskingSubscribers(speaker,true);
    Map<Variable,Symbol> bindings = question.waitForResponse(waitDuration);
    currentQuestions.remove(question);
    notifyQuestionAskingSubscribers(speaker,false);
    return bindings;
  }

  //TODO:brad the logic in this primitive should be moved to some sort of action selector
  // that can issue goals/plans to ask the appropriate clarification questions, and also
  // decide which question a given answer is for.
  @TRADEService
  @Action
  public synchronized Map<Symbol,List<Question>> getMatchingQuestions(Symbol speaker, Symbol listener, Term answerSemantics) {
    Map<Symbol,List<Question>> matchingQuestionPerAgent = new HashMap<>();
    if (questionsPerAgent.isEmpty()) {
      return matchingQuestionPerAgent;
    }
    listener = listener.toUntyped();
    speaker = speaker.toUntyped();
    Set<Question>currentQuestions = questionsPerAgent.get(listener);
    //get matching question(s)
    List<Question> matchingQuestions = currentQuestions.stream()
            .filter(q -> q.isActive() && q.isMatchingAnswerSemantics(answerSemantics))
            .toList();

    //Bind response
    if (matchingQuestions.isEmpty()) {
      //No direct match
      if (answerSemantics.getName().equals("abort")) {
        //If universal exit semantics, then 'cancel' question with the most recent timestamp
        //TODO: should this case just act as a match to any/all? If multiple current questions then ask to disambiguate?
        //TODO: allow inner semantics to either match specific question or be general "most recent" symbol?
        AtomicReference<Question> question = new AtomicReference<>();
        long maxTimestamp = -1;
        currentQuestions.forEach(q -> {
          if (q.getTimestamp() > maxTimestamp) {
            question.set(q);
          }
        });
        if (question.get() != null) {
          Question mostRecent = question.get();
          if (mostRecent != null) {
            matchingQuestionPerAgent.put(listener,List.of(mostRecent));
          }
        }
      } else {
        //Check children if no matching question is found
        Set<Symbol> children = agentHierarchy.get(listener);
        if (children != null) {
          for (Symbol child : children) {
            if (!(child.getName().equals(listener.getName()))) {
              matchingQuestionPerAgent.putAll(getMatchingQuestions(speaker,child,answerSemantics));
            }
          }
        }
      }
    } else {
      matchingQuestionPerAgent.put(listener,matchingQuestions);
    }
    return matchingQuestionPerAgent;
  }

  @TRADEService
  @Action
  public synchronized void answerQuestion(Question question, Term answerSemantics) {
    Term matchingForm = question.getLastFormMatch();
    if (matchingForm != null) {
      question.answer(matchingForm.getBindings(answerSemantics));
    } else {
      log.debug("[answerQuestion] last form match for supplied question is null, should indicate question was aborted");
      question.answer(new HashMap<>());
    }
  }

  /**
   * Remove the question corresponding to the provided string and response semantics from the current question
   * collection and signal the answer condition without setting response bindings
   *
   * @param questionString String corresponding to question which was asked
   * @param responseForms List of possible semantic forms required to match against the corresponding question
   * @param listener The agent which had asked the question
   */
  @TRADEService
  @Action
  public void cancelWaitForResponse(String questionString, List<Term> responseForms, Symbol listener) {
    listener = listener.toUntyped();
    Set<Question> currentQuestions = questionsPerAgent.get(listener);
    //get matching question(s)
    List<Question> matchingQuestions = currentQuestions.stream()
            .filter(q -> q.isMatchingQuestion(questionString) && q.hasMatchingResponseForms(responseForms))
            .toList();

    for (Question q : matchingQuestions) {
      q.endWaitForResponse();
    }
  }

  /**
   * Suspend the question(s) corresponding to the provided response semantics. Optionally unlock the answer to
   *  unblock waitForResponse call(s).
   * @param responseForm semantic form to match against the current questions
   * @param listener The agent which had asked the question
   * @param unlock specify whether to unlock the suspended question's answer (practically causes the corresponding waitForResponse call to unblock if true, and not otherwise)
   */
  @TRADEService
  @Action
  public void suspendWaitForResponse(Term responseForm, Symbol listener, boolean unlock) {
    listener = listener.toUntyped();
    Set<Question>currentQuestions = questionsPerAgent.get(listener);
    //get matching question(s)
    List<Question> matchingQuestions = currentQuestions.stream()
            .filter(q -> q.isActive() && q.hasMatchingResponseForm(responseForm))
            .toList();

    if (matchingQuestions.size() > 1) {
      log.warn("[suspendWaitForResponse] suspending multiple questions");
    }

    for (Question question : matchingQuestions) {
      question.suspend();
      if (unlock) {
        question.endWaitForResponse();
      }
    }
  }

  /**
   * Resume the previously suspended question(s) to allow them to match future question responses
   *
   * @param responseForm answer semantics required to match against the corresponding question
   * @param listener The agent which had asked the question
   *
   */
  @TRADEService
  @Action
  public void resumeWaitForResponse(Term responseForm, Symbol listener) {
    listener = listener.toUntyped();
    Set<Question>currentQuestions = questionsPerAgent.get(listener);
    //get matching question(s)
    List<Question> matchingQuestions = currentQuestions.stream()
            .filter(q -> !q.isActive() && q.hasMatchingResponseForm(responseForm))
            .toList();

    if (matchingQuestions.size() > 1) {
      log.warn("[resumeWaitForResponse] suspending multiple questions");
    }

    for (Question question : matchingQuestions) {
      question.resume();
    }
  }

  @TRADEService
  public boolean isAgentWaitingForResponse(Symbol agent) {
    Set<Question> currentQuestions = questionsPerAgent.get(agent);
    return !(currentQuestions == null || currentQuestions.stream().noneMatch(Question::isActive));
  }

  @TRADEService
  public String getLastQuestion(Symbol agent) {
    agent = agent.toUntyped();
    Set<Question>currentQuestions = questionsPerAgent.get(agent);
    if (currentQuestions.isEmpty()) {
      return "";
    } else {
      Question mostRecentQuestion = currentQuestions.stream().max(Comparator.comparingLong(Question::getTimestamp)).get();
      return mostRecentQuestion.getQuestionString();
    }
  }
}
