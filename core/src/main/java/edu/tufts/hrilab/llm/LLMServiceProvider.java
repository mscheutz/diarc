/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

//3/12:
// Base abstract class providing services required by the LLMParserComponent and LLMNLGComponent. Contains utilities for
//  tracking LLM Chats and other system information which may commonly be included in prompting requests.
// Two abstract methods must be implemented:
//      parseUtteranceLLMService: called directly by parseUtterance - goal is to populate the supplied input utterance's semantics, bindings, etc.
//      convertSemanticsToTextLLMService: called directly by convertSemanticsToText - goal is to return an output Utterance with populated words list based on input Utterance's semantics
// Subclass Tree Summary:
//  - BasicLLMServiceProvider: Another abstract class one level up providing a focused API for one-shot parsing and NLG. Extend this class if
//                             you will not have any extended interactions between the user and LLM; a new Chat with fresh context is created each parse or generation attempt
//      -- TRLLMServiceProvider: Even more focused abstract subclass of BasicLLMServiceProvider containing general pre/post-processing utilities
//                               specific to a certain prompt format which has already been applied to multiple different domains.
//          --- (4 separate domain/demo specific concrete subclasses of TRLLMServiceProvider currently exist)
//
 //     -- (1 BasicLLMServiceProvider, non-TRLLMServicerProvider currently exists)
//  - SocialLLMServiceProvider: (In progress) Initial implementation of LLMServiceProvider to provide a new framework
//                              for user language interactions with the architecture using an LLM. The goal is to allow
//                              the LLM to target any of the following:
//                                   1. Extended free-form dialogue interactions with the user - whether social or towards the NLU task
//                                   2. INSTRUCT and ITK semantics using LLM "tool"/"function" definitions
//                                   3. REPLY semantics using parallel prompting only when a question has been asked
//                                   4. Private back-door interactions with the architecture to supply additional information or guidance
public abstract class LLMServiceProvider extends DiarcComponent {

    //////////////// Abstract Methods Start ////////////////

    @TRADEService
    public abstract Utterance parseUtteranceLLMService(Utterance utterance);

    @TRADEService
    public abstract Utterance convertSemanticsToTextLLMService(Utterance u);

    @TRADEService
    public abstract boolean inExtendedInteraction(Symbol user, Symbol agentTeam);

    //////////////// Abstract Methods End ////////////////

    // System agentTeam hierarchy - relevant for understanding what agents are part of what conversations
    protected Map<Symbol,Set<Symbol>> agentHierarchy;
    private boolean addedAgents = false;

    //// General information which may be relevant for prompting - off by default ////
    //Independent dialogue history tracking - in case intending to supply external/out of Chat context dialogue to the prompt
    protected boolean trackDialogueHistory = false;
    protected Map<Symbol, Set<Integer>> conversationMappings = new HashMap<>();
    protected Map<Integer, List<Utterance>> currentConversations = new HashMap<>();
    protected AtomicInteger counter = new AtomicInteger();
    private boolean registeredForDialogueNotifications = false;

    // Track learned actions and generated plans - in case prompt targets need to be updated
    protected boolean trackLearnedActions = false;
    protected List<ActionDBEntry> learnedActions = new ArrayList<>();
    private boolean registeredForNewActions = false;
    private boolean retrievedLearnedActions = false;

    @Override
    protected void init() {
        super.init();
        shouldRunExecutionLoop = true;
    }

    @Override
    protected void executionLoop() {
        if (!addedAgents) {
            try {
                agentHierarchy = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllAgentTeams")).call(Map.class);
                addedAgents = true;
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
                }
            } catch (TRADEException e) {
                log.debug("[init] error making TRADE call to getAllAgentTeams", e);
            }
        }

        if (trackDialogueHistory && !registeredForDialogueNotifications) {
            try {
                TRADEServiceInfo updateHistoryService = getMyService("updateLLMConversationHistory", Utterance.class, Boolean.class);
                TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications")).call(void.class, updateHistoryService);
                registeredForDialogueNotifications = true;
            } catch (TRADEException e) {
                log.error("[LLMServiceProvider] exception registering for dialogue notifications, trying again", e);
            }
        }

        if (trackLearnedActions && !registeredForNewActions) {
            registerForNotifications();
        }

        if (trackLearnedActions && !retrievedLearnedActions) {
            initLearnedActions();
        }
    }

    ////////////////// Chat Object Handling ////////////////

    //TODO: move methods back into here when "social" version has been merged into unified class hierarchy

    protected Completion callChatEndpoint(Chat chat) {
        Completion response = null;
        try {
            response = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
        } catch (TRADEException e) {
            log.error("[llmParserService] Error performing chat completion", e);
        }
        return response;
    }

    //////////////// Dialogue History Tracking ////////////////

    @TRADEService
    public void updateLLMConversationHistory(Utterance utterance, boolean newUtterance) {
        if (!newUtterance) {
            return;
        }
        Symbol user;
        Symbol agentTeam;
        Symbol speaker = getUntypedSymbol(utterance.getSpeaker());
        Symbol addressee = getUntypedSymbol(utterance.getAddressee());
        if (agentHierarchy.containsKey(addressee)) {
            user = speaker;
            agentTeam = addressee;
        } else if (agentHierarchy.containsKey(speaker)) {
            agentTeam = speaker;
            user = addressee;
        } else {
            log.error("[updateLLMConversationHistory] AgentTeam not detected as speaker or listener for incoming utterance");
            return;
        }

        List<Utterance> history;
        int chatIndex = getConversationHistoryIndex(user, agentTeam);
        if (chatIndex == -1) {
            history = startLLMConversation(user,agentTeam);
        } else {
            history = currentConversations.get(chatIndex);
        }

        if (history != null) {
            //TODO: Should prune based on token count, not number of messages
            if (history.size() > 10) {
                history.remove(0);
            }
            history.add(utterance);
        }
    }

    private Set<Integer> getSharedConversations(Set<Symbol> members) {
        if (members == null || members.isEmpty()) {
            return new HashSet<>();
        }

        // TODO: synchronize?
        // Get all current chats which contain at least the supplied members
        Set<Integer> sharedChats = conversationMappings.getOrDefault(getUntypedSymbol(members.iterator().next()), new HashSet<>());
        for (Symbol member : members) {
            member = getUntypedSymbol(member);
            if (sharedChats.isEmpty()) {
                break;
            }
            Set<Integer> memberChats = conversationMappings.getOrDefault(member, new HashSet<>());
            sharedChats.retainAll(memberChats);
        }

        // No existing chats found
        if (sharedChats.isEmpty()) {
            log.debug("[getMatchingChat] could not find existing conversation including the following agents: {}", members);
        }

        return sharedChats;
    }

    protected List<Utterance> getConversationHistory(Symbol user, Symbol agentTeam) {
        Set<Symbol> members = getDescendants(agentTeam);
        members.add(user);
        return getConversationHistory(members);
    }

    protected List<Utterance> getConversationHistory(Set<Symbol> members) {
        int chatIndex = getConversationHistoryIndex(members);

        if (chatIndex == -1) {
            return new ArrayList<>();
        }

        if (currentConversations.get(chatIndex) == null) {
            log.error("[getMatchingChat] chatMappings and currentChats have different notions of existing conversations");
            return new ArrayList<>();
        }
        return new ArrayList<>(currentConversations.get(chatIndex));
    }

    private int getConversationHistoryIndex(Symbol user, Symbol agentTeam) {
        Set<Symbol> members = getDescendants(agentTeam);
        members.add(user);
        return getConversationHistoryIndex(members);
    }

    private int getConversationHistoryIndex(Set<Symbol> members) {
        Set<Integer> sharedChats = getSharedConversations(members);

        // Multiple chats found, not enough members were provided to distinguish between all currently ongoing conversations
        if (sharedChats.size() > 1) {
            log.warn("[getMatchingChat] supplied entities are subsets of multiple conversation groups. Returning most recently updated conversation");
            //TODO: return most recently updated chat
        } else if (sharedChats.isEmpty()) {
            return -1;
        }
        return sharedChats.iterator().next();
    }

    //For conversation history context purposes, not currently properly supported
    /**
     * Start shared dialogue history between a user and an AgentTeam
     */
    @TRADEService
    public List<Utterance> startLLMConversation(Symbol speaker, Symbol agentTeam) {
        return startLLMConversation(Set.of(speaker),Set.of(agentTeam));
    }
    /**
     * Start a shared dialogue history between a set of users and a set of AgentTeams
     */
    @TRADEService
    public List<Utterance> startLLMConversation(Set<Symbol> speakers, Set<Symbol> agentTeams) {
        Set<Symbol> members = new HashSet<>(speakers);
        for (Symbol agentTeam : agentTeams) {
            members.addAll(getDescendants(agentTeam));
        }
        ////TODO: not sure we want this, adding default listener of any actorless generateResponse to every conversation
        //members.add(Factory.createSymbol("person"));
        return startLLMConversation(members);
    }
    /**
     * Start shared dialogue history between a set of entities
     */
    @TRADEService
    public List<Utterance> startLLMConversation(Set<Symbol> members) {
        Integer conversationIndex = counter.incrementAndGet();
        for (Symbol member : members) {
            member = getUntypedSymbol(member);
            Set<Integer> memberConversations = conversationMappings.getOrDefault(member, new HashSet<>());
            memberConversations.add(conversationIndex);
            conversationMappings.put(member, memberConversations);
        }
        List<Utterance> newHistory = new ArrayList<>();
        currentConversations.put(conversationIndex, newHistory);
        return newHistory;
    }
    /**
     * End shared dialogue history between a user and an AgentTeam
     */
    @TRADEService
    public void endLLMConversation(Symbol speaker, Symbol agentTeam) {
        endLLMConversation(Set.of(speaker),Set.of(agentTeam));
    }
    /**
     * End a shared dialogue history between a set of users and a set of AgentTeams
     */
    @TRADEService
    public void endLLMConversation(Set<Symbol> speakers, Set<Symbol> agentTeams) {
        Set<Symbol> members = new HashSet<>(speakers);
        for (Symbol agentTeam : agentTeams) {
            members.addAll(getDescendants(agentTeam));
        }
        endLLMConversation(members);
    }
    /**
     * End shared dialogue history between a set of entities
     */
    @TRADEService
    public void endLLMConversation(Set<Symbol> members) {
        Set<Integer> sharedConversations = getSharedConversations(members);
        if (sharedConversations.isEmpty()) {
            log.warn("[endLLMConversation] no shared conversation found for members: {}", members);
            return;
        } else if (sharedConversations.size() > 1) {
            log.warn("[endLLMConversation] multiple conversations found including supplied members: {}", members);
        }
        for (Integer id : sharedConversations) {
            endLLMConversation(members,id);
        }
    }

    @TRADEService
    public void endLLMConversation(Set<Symbol> members, Integer id) {
        currentConversations.remove(id);
        for (Symbol member : members) {
            member = getUntypedSymbol(member);
            Set<Integer> currentConversations = conversationMappings.get(member);
            if (currentConversations != null) {
                currentConversations.remove(id);
            }
        }
    }

    @TRADEService
    @Action
    public void clearConversationHistory() {
        //currentChats = new HashMap<>();
        conversationMappings = new HashMap<>();
        currentConversations = new HashMap<>();
    }

    //////////////// Taught Action Tracking ////////////////

    @TRADEService
    public void registerForNotifications() {
        try {
            TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
            TRADEServiceInfo additionNotificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleDatabaseAddition").argTypes(ActionDBEntry.class));
            registerService.call(void.class, additionNotificationHandler, NotificationType.DATABASE_ADDITION);
            TRADEServiceInfo removalNotificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleDatabaseRemoval").argTypes(ActionDBEntry.class));
            registerService.call(void.class, removalNotificationHandler, NotificationType.DATABASE_REMOVAL);
            registeredForNewActions = true;
            log.info("[registerForNotifications] successfully registered for database additions");
        } catch (TRADEException e) {
            log.debug("[registerForNotifications] error registering for actionDB notifications", e);
        }
    }

    protected void initLearnedActions() {
        try {
            learnedActions = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLearnedActions")).call(List.class);
            retrievedLearnedActions = true;
        } catch (TRADEException e) {
            log.debug("[initLearnedActions] error getting taught actions from EM on startup");
        }
    }

    @TRADEService
    public void handleDatabaseAddition(ActionDBEntry entry) throws TRADEException {
        if (entry.isScript() && entry.getDBFile() == null && !entry.getName().equals("planned")) {
            log.info("[handleDatabaseAddition] have learned action {}", entry.getName());
            learnedActions.add(entry);
        }
    }

    @TRADEService
    public void handleDatabaseRemoval(ActionDBEntry entry) throws TRADEException {
        learnedActions.remove(entry);
        //TODO: (external) should ideally remove any cache entries relevant to this action
    }

    //////////////// Taught Action Tracking End ////////////////

    @Override
    public void shutdownComponent() {
        super.shutdownComponent();
        try {
            if (trackLearnedActions) {
                TRADEServiceInfo unregisterService = TRADE.getAvailableService(new TRADEServiceConstraints().name("unregisterGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
                TRADEServiceInfo notificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleGoalNotification").argTypes(GoalInfo.class));
                unregisterService.call(void.class, notificationHandler, NotificationType.DATABASE_ADDITION);
            }

            if (trackDialogueHistory) {
                TRADEServiceInfo updateHistoryService = getMyService("updateLLMConversationHistory", Utterance.class, Boolean.class);
                TRADE.getAvailableService(new TRADEServiceConstraints().name("unregisterForDialogueHistoryNotifications")).call(void.class, updateHistoryService);
            }
        } catch (TRADEException e) {
            log.error("[shutdownComponent]", e);
        }
    }

    protected Set<Symbol> getDescendants(Symbol agentTeam) {
        agentTeam = getUntypedSymbol(agentTeam);

        //Get self and all descendants from hierarchy
        Set<Symbol> agentList = agentHierarchy.get(agentTeam);
        //Agent is a leaf, add only itself
        if (agentList == null) {
            agentList = new HashSet<>();
            agentList.add(agentTeam);
        } else {
            agentList = new HashSet<>(agentList);
        }
        return agentList;
    }

    protected Symbol getUntypedSymbol(Symbol s) {
        if (s.hasType()) {
            s = Factory.createSymbol(s.toUntypedString());
        }
        return s;
    }
}
