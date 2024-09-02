/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.NLGInterface;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

// This class is just a template for a class which can provide relevant LLM NLU and NLG services with access to dialogue
//  history and tracking specific chat contexts.
// Including a subclass of this one in a config will provide necessary services for NLU and NLG on its own (still need
//  to have a separate LLMComponent)
// Temp: To use alongside the HybridParseComponent, pass in "-llm spotPickAndPlace"
//TODO: rework existing dialogue llm structure/LLMParserComponent/this to coexist or have single version of things
//TODO: commandLine arg for performNLG
public abstract class LLMServiceProvider extends DiarcComponent implements NLGInterface{
    boolean performNLG = true;
    SimpleNLGComponent nlg;

    //Set of specific user/agent back and forths to get to the bottom of a single NLU attempt
    Map<Pair<Symbol,Symbol>,Chat> currentChats = new HashMap<>();

    //Set of existing 'conversations' between sets of users and system agents
    //Currently don't have a way of determining whether dialogue is happening as part of the same or a different interaction
    //  so will simply be a rolling window of the dialogue history (maybe this is all we want anyway)
    //Currently going to split dialogue histories between specific speaker/AgentTeam pairs. Generally could be any set
    //  of speakers and any set of AgentTeams.
    Map<Symbol,Set<Integer>> conversationMappings = new HashMap<>();
    Map<Integer,List<Utterance>> currentConversations = new HashMap<>();
    AtomicInteger counter = new AtomicInteger();

    //TODO: be conscious of maxToken limit on each Chat
    //TODO: history length pruning for conversation history if going too long

    private Map<Symbol,Set<Symbol>> agentHierarchy;

    private boolean addedAgents = false;
    private boolean registeredForDialogueNotifications = false;

    /**
     * Generate a Chat object to call LLM's chatCompletion service with for the generation task. Practically entails
     * generating a system and user message for an NLG request of the supplied utterance.
     * @param utterance outgoing utterance to perform NLG for
     * @param conversationHistory Relevant dialogue history leading up to this outgoing utterance which may provide context.
     * @return Fully updated Chat object to pass to an LLM's chatCompletion service for NLG
     */
    protected abstract Chat generateNLGChat(Utterance utterance, List<Utterance> conversationHistory);
    /**
     * Generate a Chat object to call LLM's chatCompletion service with for the parsing task. Practically entails
     * creating a new or updating an existing Chat object to contain the proper system, user, and assistant messages to
     * come up with a proper parse of the supplied utterance.
     * @param utterance incoming utterance to be parsed
     * @param currentChat Chat object containing prior requests and responses between the LLM for this current parse attempt.
     *                    Null if this is the first interaction.
     * @param conversationHistory Relevant dialogue history leading up to this incoming utterance which may provide context.
     * @return Fully updated Chat object to pass to an LLM's chatCompletion service for parsing
     */
    protected abstract Chat generateNLUChat(Utterance utterance, Chat currentChat, List<Utterance> conversationHistory);

    /**
     * Postprocessing of LLM's chatCompletion return value for the NLU task.
     * @return The fully populated Utterance to return from parseUtterance
     */
    protected abstract Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse);
    /**
     * Postprocessing of LLM's chatCompletion return value for the NLG task.
     * @return The fully populated Utterance to return convertSemanticsToText.
     */
    protected abstract Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse);


    @Override
    protected void init() {
        super.init();
        shouldRunExecutionLoop = true;
        //if (!performNLG) {
        //    nlg = new SimpleNLGComponent();
        //}
        //Always instantiating as a fallback in case LLM gets overloaded or completion request throws any other exception
        nlg = new SimpleNLGComponent();
    }

    //@Override
    //protected List<Option> additionalUsageInfo() {
    //    List<Option> options = new ArrayList<>();
    //    options.add(Option.builder("nlg").desc("Use the LLM to perform NLG. Otherwise use SimpleNLG.").build());
    //    return options;
    //}

    //@Override
    //protected void parseArgs (CommandLine cmdLine) {
    //    if (cmdLine.hasOption("nlg")) {
    //        performNLG = true;
    //    }
    //}

    @Override
    protected void executionLoop() {
        if (!registeredForDialogueNotifications) {
            try {
                TRADEServiceInfo updateHistoryService = getMyService("updateLLMConversationHistory", Utterance.class);
                TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications")).call(void.class, updateHistoryService);
                registeredForDialogueNotifications = true;
            } catch (TRADEException e) {
                log.error("[LLMServiceProvider] exception registering for dialogue notifications, trying again", e);
            }
        }
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
                    agentHierarchy = untypedAgentHierarchy;
                }
            } catch (TRADEException e) {
                log.debug("[init] error making TRADE call to getAllAgentTeams", e);
            }
        }
    }

    @Override
    public Utterance convertSemanticsToText(Utterance u) {
        if (performNLG) {
            return llmGenerationService(u);
        } else {
            return nlg.convertSemanticsToText(u);
        }
    }

    public Utterance parseUtterance(Utterance utterance) {
        return llmChatCompletionHelper(utterance, true);
    }

    public Utterance llmGenerationService(Utterance utterance) {
        return llmChatCompletionHelper(utterance, false);
    }

    private Utterance llmChatCompletionHelper(Utterance utterance, boolean parsing) {
        // Get existing chat and dialogue history for the utterance's speaker and listener and call abstract method to
        //  create the Chat object to pass to the LLM
        Chat chat;
        List<Utterance> conversationHistory;
        Symbol user;
        Symbol agentTeam;
        if (parsing) {
            user = getUntypedSymbol(utterance.getSpeaker());
            agentTeam = getUntypedSymbol(utterance.getAddressee());
        } else {
            user = getUntypedSymbol(utterance.getAddressee());
            agentTeam = getUntypedSymbol(utterance.getSpeaker());
        }
        if (!agentHierarchy.containsKey(agentTeam)) {
            log.error("[llmChatCompletionHelper] System agent found in incoming utterance not found in agent hierarchy");
            chat = null;
            conversationHistory = new ArrayList<>();
        } else {
            chat = currentChats.get(Pair.of(user, agentTeam));
            conversationHistory = getConversationHistory(utterance.getSpeaker(), utterance.getAddressee());
        }

        if (parsing) {
            chat = generateNLUChat(utterance, chat, conversationHistory);
        } else {
            chat = generateNLGChat(utterance, conversationHistory);
        }

        // Call LLM to get Completion
        Completion response;
        try {
            response = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
        } catch (TRADEException e) {
            log.error("[llmParserService] Error performing chat completion", e);
            if (!parsing) {
                log.debug("[llmParserService] Error during chat completion during NLG, using SimpleNLG as a fallback");
                return nlg.convertSemanticsToText(utterance);
            } else {
                return utterance;
            }
        }

        //Add LLM's response to the chat //TODO: do we actually generally want this present in the chat in this way?
        chat.addRobotMessage(response.getText());

        // Call abstract method to transform Completion to Utterance (implementation dependent on prompt)
        if (parsing) {
            return parseNLUCompletionResponse(utterance, response);
        } else {
            return parseNLGCompletionResponse(utterance, response);
        }
    }

    @TRADEService
    public void updateLLMConversationHistory(Utterance utterance) {
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

    private List<Utterance> getConversationHistory(Symbol user, Symbol agentTeam) {
        Set<Symbol> members = getDescendants(agentTeam);
        members.add(user);
        return getConversationHistory(members);
    }

    private List<Utterance> getConversationHistory(Set<Symbol> members) {
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

    //TODO: not used right now, only place I see this being currently possible is from some dialogue recovery script
    @TRADEService
    @Action
    public void startLLMChatContext(Utterance u) {
        Chat chat = new Chat();
        chat.addUserMessage(u.getWordsAsString());
        Symbol speaker = getUntypedSymbol(u.getSpeaker());
        Symbol addressee = getUntypedSymbol(u.getAddressee());
        currentChats.put(Pair.of(speaker,addressee),chat);
    }

    @TRADEService
    @Action
    public void endLLMChatContext(Pair<Symbol,Symbol> members) {
        members = Pair.of(getUntypedSymbol(members.getLeft()),getUntypedSymbol(members.getRight()));
        currentChats.remove(members);
    }

    protected Set<Symbol> getDescendants(Symbol agentTeam) {
        agentTeam = getUntypedSymbol(agentTeam);

        //Get self and all descendants from hierarchy
        Set<Symbol> agentList = agentHierarchy.get(agentTeam);
        //Agent is a leaf, add only itself
        if (agentList == null) {
            agentList = new HashSet<>();
            agentList.add(agentTeam);
        }
        return agentList;
    }

    protected Symbol getUntypedSymbol(Symbol s) {
        if (s.hasType()) {
            s = Factory.createSymbol(s.toUntypedString());
        }
        return s;
    }

    @TRADEService
    @Action
    public void clearConversationHistory() {
        currentChats = new HashMap<>();
        conversationMappings = new HashMap<>();
        currentConversations = new HashMap<>();
    }

    @TRADEService
    @Action
    public void toggleLLMNLG() {
        performNLG = !performNLG;
    }
}
