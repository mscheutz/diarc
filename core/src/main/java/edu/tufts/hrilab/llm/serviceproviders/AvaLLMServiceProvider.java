/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
//import edu.tufts.hrilab.action.goal.GoalInfo;
//import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.LLMServiceProvider;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.apache.commons.cli.CommandLine;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AvaLLMServiceProvider extends LLMServiceProvider {
    Prompt nluSystemPrompt = Prompts.getPrompt("ava/nlu/system");
    Prompt nluUserPrompt = Prompts.getPrompt("ava/nlu/user");
    Prompt nlgSystemPrompt = Prompts.getPrompt("ava/nlg/system");
    Prompt nlgUserPrompt = Prompts.getPrompt("ava/nlg/user");
    Prompt validationPrompt = Prompts.getPrompt("ava/nlg/validation");
    private boolean registeredForNewActions = false;
    private String learnedActionsString = "";

    @Override
    protected void executionLoop () {
        super.executionLoop();
        if (!registeredForNewActions) {
            registerForNotifications();
        }
    }

    @Override
    public Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse) {
        try {
            Utterance.Builder output = new Utterance.Builder(incomingUtterance);
            log.info(llmResponse.getText());
            String responseString = llmResponse.getText();

            Pattern semanticsPattern = Pattern.compile(".*\"?Semantics\"?\\s*:\\s*\"?(.*)\"?,\\s*\"?Input Type\"?:.*", Pattern.DOTALL);
            Matcher m = semanticsPattern.matcher(responseString);
            if (!m.matches()) {
                log.error("[AvaLLMServiceProvider] cannot find Semantics or input type result in LLM response");
                return incomingUtterance;
            }
            Predicate semantics = Factory.createPredicate(m.group(m.groupCount()));

            Pattern typePattern = Pattern.compile(".*\"?Input Type\"?\\s*:\\s*\"?(.*?)\"?,\\s*\"?Confidence\"?:.*", Pattern.DOTALL);
            m = typePattern.matcher(responseString);
            if (!m.matches()) {
                log.error("[AvaLLMServiceProvider] cannot find Input Type result in LLM response");
                return incomingUtterance;
            }
            UtteranceType utteranceType;
            try {
                utteranceType = UtteranceType.valueOf(m.group(m.groupCount()).trim());
            } catch (IllegalArgumentException e) {
                log.error("[AvaLLMServiceProvider] generated Input Type {} invalid", m.group(m.groupCount()));
                return incomingUtterance;
            }

            Pattern confidencePattern = Pattern.compile(".*\"?Confidence\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
            m = confidencePattern.matcher(responseString);
            if (!m.matches()) {
                log.error("[AvaLLMServiceProvider] cannot find Confidence result in LLM response");
                return incomingUtterance;
            }
            String confidenceString = m.group(m.groupCount()).trim();
            int confidence = Integer.parseInt(confidenceString);
            if (confidence < 50) {
                log.info("[spotPickAndPlaceLLMServiceProvider] llm indicated parse result as low confidence ({})", confidence);
                //TODO: start conversation and recovery dialogue?
                //      Cache result so that script can be entered based on utterance/agent/something?
                return incomingUtterance;
            }

            output.setUtteranceType(utteranceType);

            Map<Symbol, Variable> varMap = new LinkedHashMap<>();
            List<Symbol> processedArguments = new ArrayList<>();
            if (utteranceType == UtteranceType.INSTRUCT) {
                String actorName = incomingUtterance.getAddressee().getName();
                processedArguments.add(Factory.createSymbol(actorName));
            }
            processedArguments = processSemantics(semantics, processedArguments, varMap, output, incomingUtterance.getAddressee().getName());
            if (processedArguments == null) {
                return incomingUtterance;
            }
            output.setSemantics(Factory.createPredicate(semantics.getName(), processedArguments));
            output.setNeedsValidation(true);

            Utterance result = output.build();
            boolean parsed = correctCommonNLUMistakes(result);
            if (parsed) {
                return result;
            } else {
                return incomingUtterance;
            }
        } catch (Exception e) {
            log.warn("[parseNLUCompletionResponse]",e);
            return incomingUtterance;
        }
    }

    private boolean correctCommonNLUMistakes(Utterance utterance) {
        ////TODO: this needs to be caught earlier in the pipeline than here, just copying here to maintain awareness of
        //      the issue without hacking code in the asr app or parser itself.
        //incoming.setWords(incoming.getWords().stream().map(word -> {
        //  word = word.toLowerCase();
        //  if (word.equals("wi-fi")) {
        //    word = "wifi";
        //  }
        //  return word;
        //}).collect(Collectors.toList()));

        String actorName = utterance.getAddressee().getName();
        try {
            String command = utterance.getSemantics().getName();
            if (command.equals("location")) {
                Symbol arg = ((Term)utterance.getSemantics()).get(0);
                if (!arg.isVariable() && !arg.getName().equals("none") && !(arg.isVariable() && !arg.getType().equals("location"))) {
                    log.warn("[correctCommonNLUMistakes] LLM returned location(?location) parse where ?location is not a reference. Not allowing");
                    return false;
                }
            } else if (command.equals("taskPredicate")) {
                //Not currently prompted to do this
                if (utterance.getSemantics().hasArgs()) {
                    Symbol innerArg = ((Term)utterance.getSemantics()).get(0);
                    List<Symbol> argsWithActor = new ArrayList<>();
                    argsWithActor.add(Factory.createSymbol("ava"));
                    if (innerArg.isTerm()) {
                        argsWithActor.addAll(((Term) ((Term) utterance.getSemantics()).get(0)).getArgs());
                    }
                    utterance.setSemantics(Factory.createPredicate("taskPredicate", Factory.createPredicate(innerArg.getName(), argsWithActor)));
                }
            }
        } catch (Exception e) {
            log.warn("[correctCommonNLUMistakes] unexpected semantics format");
            return false;
        }
        return true;
    }

    private List<Symbol> processSemantics(Predicate semantics, List<Symbol> processedArguments, Map<Symbol,Variable> varMap, Utterance.Builder output, String actor) {
        for (Symbol arg: semantics.getArgs()) {
            if (arg.isPredicate()) {
                List<Symbol> innerArgs = processSemantics((Predicate)arg,new ArrayList<>(),varMap,output,actor);
                if (innerArgs == null) {
                    return null;
                }
                processedArguments.add(Factory.createPredicate(arg.getName(), innerArgs));
            } else {
                String argString = arg.toString();
                if (argString.startsWith("?")) {
                    log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}", argString);
                    return null;
                }
                Symbol argSymbol;
                String descriptorType = getDescriptorType(actor,argString);
                //String descriptorType = "";
                ////TODO: remove this - make LLM add types itself. Right now any conflicting literal will be processed as a ref
                //if (!semantics.getName().endsWith("Name")) {
                //    getDescriptorType(actor, argString);
                //}
                if (descriptorType.isEmpty()) {
                    argSymbol = Factory.createFOL(argString);
                } else {
                    //v2 only
                    argSymbol = Factory.createSymbol(argString, descriptorType);
                }

                //TODO: replace with all types which are refs (or in general should be anything which is not an agent?)
                //Should generally be any reference, just know in this case all references will be locations
                if (argSymbol != null && (argSymbol.getType().equals("location"))) {
                    if (!varMap.containsKey(argSymbol)) {
                        Variable var = Factory.createVariable("VAR" + varMap.size() + ":" + argSymbol.getType());
                        varMap.put(argSymbol, var);

                        output.addSupplementalSemantics(Factory.createPredicate(argSymbol.toUntypedString(), var));
                        Symbol tier = Factory.createSymbol("DEFINITE");
                        output.addTierAssignment(var, tier);

                        processedArguments.add(var);
                    } else {
                        processedArguments.add(varMap.get(argSymbol));
                    }
                } else {
                    //TODO: this isn't always the case, if we want to allow variables to be generated by the LLM this needs to change and the prompt needs to be updated
                    //Domain-specific safeguard to prevent the LLM from capitalizing arguments
                    //If argument is a wildcard, don't want it to be capitalized by the LLM just because it recognizes it as a proper noun
                    if (Character.isUpperCase(argString.charAt(0)) && argString.length() > 1) {
                        try {
                            TimeUnit.valueOf(argString);
                            processedArguments.add(Factory.createSymbol(argString));
                        } catch (IllegalArgumentException ignored) {
                            String lowercaseArg = argString;
                            lowercaseArg = Character.toLowerCase(lowercaseArg.charAt(0)) + lowercaseArg.substring(1);
                            processedArguments.add(Factory.createSymbol(lowercaseArg));
                        }
                    } else {
                        processedArguments.add(Factory.createSymbol(argString));
                    }
                }
            }
        }
        return processedArguments;
    }

    @Override
    public Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse) {
        log.info(llmResponse.getText());
        String responseString = llmResponse.getText().trim();

        Pattern outputPattern = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
        Matcher m = outputPattern.matcher(responseString);
        if (!m.matches()) {
            log.error("[parseNLGCompletionResponse] cannot find Output field in LLM response");
            return null;
        }
        String output = m.group(m.groupCount());
        outgoingUtterance.setWords(List.of(output.split(" ")));

        return outgoingUtterance;
    }

    @Override
    protected Chat generateNLGChat(Utterance utterance, List<Utterance> conversationHistory) {
        Chat chat = new Chat();

        //Intercept Validation Question and use specific validation prompt
        String userMessage;
        Predicate semantics;
        List<Term> boundSemantics = utterance.getBoundSemantics();
        if (boundSemantics.isEmpty()) {
            semantics = (Predicate) utterance.getSemantics();
        } else {
            semantics = (Predicate) boundSemantics.get(0);
        }
        List<String> locations = getLocations("self", true);
        List<String> patrolPaths = getPatrolPaths();

        Map<String,String> promptVars = new LinkedHashMap<>();
        utterance = new Utterance.Builder(utterance).build();
        //Doesn't include suppSemantics
        semantics = replaceReferencesInSemantics(semantics);
        utterance.setSemantics(semantics);
        promptVars.put("%utterance", utterance.toString());
        populatePromptVarsWithProperties(promptVars, locations);
        if (semantics.getName().equals("mean")) {
            userMessage = validationPrompt.apply(promptVars);
        } else {
            userMessage = nlgUserPrompt.apply(promptVars);
        }

        promptVars = new LinkedHashMap<>();
        promptVars.put("%conversationHistory", getConversationHistoryString(conversationHistory));
        promptVars.put("%language", language);
        promptVars.put("%patrolPathList", String.join("\n", patrolPaths));
        promptVars.put("%locationList", String.join("\n", locations));
        chat.setSystem(nlgSystemPrompt.apply(promptVars));
        chat.addUserMessage(userMessage);

        log.info("generateNLGChat called for: {}", utterance);

        return chat;
    }

    Predicate replaceReferencesInSemantics(Predicate semantics) {
        List<Symbol> transformedArgs = new ArrayList<>();
        for (Symbol arg: semantics.getArgs()) {
            if (arg.isPredicate()) {
                transformedArgs.add(replaceReferencesInSemantics((Predicate) arg));
            } else {
                if (arg.getType().equals("location")) {
                    Map<Symbol, List<Term>> refs = new HashMap<>();
                    try {
                        refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
                        if (refs.get(arg).size() == 1) {
                            transformedArgs.add(Factory.createSymbol(refs.get(arg).get(0).getName(),refs.get(arg).get(0).get(0).getType()));
                        }
                    } catch (TRADEException e) {
                        log.warn("[generateREG]", e);
                    }
                } else {
                    transformedArgs.add(arg);
                }
            }
        }
        return Factory.createPredicate(semantics.getName(), transformedArgs);
    }

    @Override
    protected Chat generateNLUChat(Utterance utterance, Chat currentChat, List<Utterance> conversationHistory){
        if (currentChat == null) {
            currentChat = new Chat();
        }

        Map<String,String> promptVars = new LinkedHashMap<>();
        promptVars.put("%conversationHistory", getConversationHistoryString(conversationHistory));
        List<String> locations = getLocations("self", true);
        List<String> patrolPaths = getPatrolPaths();
        promptVars.put("%patrolPathList", String.join("\n", patrolPaths));
        promptVars.put("%locationList", String.join("\n", locations));
        promptVars.put("%worldState", getWorldStateString(utterance.getAddressee()));
        promptVars.put("%learnedActions%",learnedActionsString);
        currentChat.setSystem(nluSystemPrompt.apply(promptVars));
        promptVars.clear();
        populatePromptVarsWithProperties(promptVars, locations);
        promptVars.put("%input", utterance.getWordsAsString());
        currentChat.addUserMessage(nluUserPrompt.apply(promptVars));

        return currentChat;
    }

    @Action
    @TRADEService
    public void testLLMNLGWithString(String semantics) {
        Chat chat = new Chat();
        List<Utterance> conversationHistory = new ArrayList<>();
        Predicate semanticsPred = Factory.createPredicate(semantics);
        Utterance utterance = new Utterance.Builder().setSemantics(semanticsPred).setIsInputUtterance(false).setUtteranceType(UtteranceType.REPLY).setSpeaker(Factory.createSymbol("robotone")).setListener(Factory.createSymbol("pete")).build();
        chat = generateNLGChat(utterance, conversationHistory);

        // Call LLM to get Completion
        Completion response;
        try {
            response = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
        } catch (TRADEException e) {
            log.error("[testLLMNLGWithString] Error performing chat completion", e);
            return;
        }
        utterance = parseNLGCompletionResponse(utterance, response);
        log.info("[testLLMNLGWithString] {}",utterance.getWordsAsString());

        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(String.class)).call(Void.class,utterance.getWordsAsString());
        } catch (TRADEException e) {
            log.error("[testLLMNLGWithString] Error calling sayText", e);
        }
    }

    private String getConversationHistoryString(List<Utterance> conversationHistory) {
        StringBuilder builder = new StringBuilder();
        if (conversationHistory.isEmpty()) {
            builder.append("<there is no conversation history>");
        } else {
            for (Utterance pastUtterance : conversationHistory) {
                builder.append(pastUtterance.getSpeaker()).append(": ").append(pastUtterance.getWordsAsString()).append("\n");
            }
        }
        return builder.toString();
    }

    private Map<String,String> populatePromptVarsWithProperties(Map<String,String> promptVars, List<String> locations) {
        //int numLocations = locations.size();
        //if (numLocations > 0) {
        //    promptVars.put("location one", locations.get(0));
        //    if (numLocations > 1) {
        //        promptVars.put("location two", locations.get(1));
        //        if (numLocations > 2) {
        //            promptVars.put("location three", locations.get(2));
        //            if (numLocations > 3) {
        //                promptVars.put("location four", locations.get(3));
        //            }
        //        }
        //    }
        //}
        return promptVars;
    }

    //TODO: calls to this/getMapReferences wildly inefficient, fix
    private List<String> getLocations(String actorName, boolean english) {
        try {
            Map<Symbol, PoseReference> refs = new HashMap<>();
            //TODO: should be based on group
            TRADEServiceInfo tsi = getServiceForAgent(actorName, "getMapReferences");
            if (tsi != null) {
               refs = tsi.call(Map.class);
            }
            return refs.values().stream().map(ref -> {
                if (!ref.properties.isEmpty()) {
                    return ref.properties.get(0).getName();
                } else {
                    return "";
                }
            }).toList();
        } catch (TRADEException e) {
            log.error("[getLocations]", e);
        }

        return new ArrayList<>();
    }

    private List<String> getPatrolPaths() {
        List<String> pathNames = new ArrayList<>();
        Predicate query = Factory.createPredicate("collection","X","Y");
        List<Map<Variable, Symbol>> bindings;
        try {
            bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
            for (Map<Variable, Symbol> binding : bindings) {
                Symbol name = binding.get(new Variable("X"));
                pathNames.add(name.getName());
            }
        } catch (TRADEException e) {
            log.error("[getPatrolPaths] ", e);
        }
        return pathNames;
    }

    private String getWorldStateString(Symbol agent) {
        StringBuilder builder = new StringBuilder();
        Predicate query = Factory.createPredicate("at","X","Y");
        List<Map<Variable, Symbol>> bindings;
        String robotLocationString = "";
        String holdingString = "";
        try {
            bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
            TRADEServiceInfo tsi = getServiceForAgent("self", "getMapReferences");
            Map<Symbol, PoseReference> refs = new HashMap<>();
            if (tsi != null) {
                refs = tsi.call(Map.class);
            }
            for (Map<Variable, Symbol> binding : bindings) {
                Symbol y = binding.get(new Variable("Y"));
                //TODO: should be based on group
                PoseReference ref = refs.get(y);
                if (ref != null) {
                    if (!ref.properties.isEmpty()) {
                        robotLocationString = String.format("The robot is at %s. ", ref.properties.get(0).getName());
                    }
                }
            }
        } catch (TRADEException e) {
            log.error("[getWorldState] ", e);
            return "Nothing is known by the robot";
        }
        if (robotLocationString.isEmpty()) {
            log.warn("[getWorldState] no matching beliefs found for agent {}", agent);
            return "Nothing is known by the robot";
        }
        return robotLocationString + holdingString + builder;
    }

    private String getDescriptorType(String agentName, String descriptor) {
        List<String> propertyNames  = getLocations("self", true);
        if (propertyNames.contains(descriptor)) {
            return "location";
        }

        return "";
    }

    @TRADEService
    //public void registerForNotifications(TRADEServiceInfo notificationHandler) {
    public void registerForNotifications() {
        try {
            TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
            TRADEServiceInfo notificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleDatabaseAddition").argTypes(ActionDBEntry.class));
            registerService.call(void.class, notificationHandler, NotificationType.DATABASE_ADDITION);
            registeredForNewActions = true;
            log.info("[registerForNotifications] successfully registered for database additions");
        } catch (TRADEException e) {
            log.debug("[registerForNotifications] error registering for actionDB notifications", e);
        }
    }

    @TRADEService
    public void handleDatabaseAddition(ActionDBEntry entry) throws TRADEException {
        if (entry.isScript() && entry.getDBFile() == null && !entry.getName().equals("planned")) {
            log.info("[handleDatabaseAddition] have learned action {}", entry.getName());
            //learnedActions.add(entry);
            learnedActionsString += String.format("%s\nDescription: The user has previously manually taught the definition of this action through the use of the learnNewTask and endActionLearning actions. It will be referred to as \"%s\" when instructed by the user.\n#\n",entry.getSignature(false),entry.getName());
        }
    }

    @Override
    public void shutdownComponent() {
        super.shutdownComponent();
        try {
            TRADEServiceInfo unregisterService = TRADE.getAvailableService(new TRADEServiceConstraints().name("unregisterGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
            TRADEServiceInfo notificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleGoalNotification").argTypes(GoalInfo.class));
            unregisterService.call(void.class, notificationHandler, NotificationType.DATABASE_ADDITION);
        } catch (TRADEException e) {
            throw new RuntimeException(e);
        }
    }

    TRADEServiceInfo getServiceForAgent(String agentName, String serviceName, Class<?>... argTypes) {
        HashSet<TRADEServiceInfo> services = (HashSet<TRADEServiceInfo>) TRADE.getAvailableServices(new TRADEServiceConstraints().name(serviceName).argTypes(argTypes));
        for (TRADEServiceInfo tsi : services) {
            Collection<String> groups = tsi.getGroups();
            if (agentName.equals("self") && groups.isEmpty()) {
                return tsi;
            } else {
                for (String group : groups) {
                    if (group.contains(agentName)) {
                        return tsi;
                    }
                }
            }
        }
        log.warn("[getServiceForAgent] could not find service {} for agent {}", serviceName, agentName);
        return null;
    }

    @TRADEService
    @Action
    public boolean ignoreTentativeAccept(Predicate goalPred){
        log.info("[ignoreTentativeAccept] goalPred: "+goalPred);
        if(goalPred != null) {
            switch (goalPred.getName()) {
                case "listScheduledGoals":
                case "describeCurrentLocation":
                case "describePendingGoals":
                case "describeAllKnownLocations":
                case "describeAllKnownPatrolPaths":
                case "describeAllLocationsOnPatrolPath":
                case "repeatLastQuestion":
                case "describeCurrentGoal":
                    return true;
                default:
                    return false;
            }
        }
        return false;
    }
}
