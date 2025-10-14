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
    Prompt validationSystemPrompt = Prompts.getPrompt("ava/validation/system");
    Prompt validationUserPrompt = Prompts.getPrompt("ava/validation/user");
    private boolean registeredForNewActions = false;
    private boolean retrievedLearnedActions = false;
    private String learnedActionsString = "";
    //TODO: All of this information should potentially be spread out to different parts of the system and gathered here
    //  on startup rather than hardcoded. E.g. descriptions from ActionDBEntries (asl pre-existing and new java annotations),
    //  NL examples from various parsers, selection of exposed actions from config, ... This is a fairly large architectural
    //  decision to discuss and therefore will need to wait until after first deployment.
    private class LLMActionEntry {
        String signature;
        String description;
        LLMNLExample example;

        LLMActionEntry(String signature, String description, LLMNLExample example) {
            this.signature = signature;
            this.description = description;
            this.example = example;
        }

        LLMActionEntry(String signature, String description, String exampleInstruction, String exampleTranslation, int exampleConfidence) {
            this.signature = signature;
            this.description = description;
            this.example = new LLMNLExample(exampleInstruction, exampleTranslation, exampleConfidence);
        }

        public String toString(boolean nlu) {
            String signatureString = signature;
            if (!nlu) {
                String[] parts = signatureString.split("\\(",2);
                signatureString = parts[0] + "(ava";
                if (parts[1].equals(")")) {
                    signatureString += parts[1];
                } else {
                    signatureString += "," + parts[1];
                }
            }
            String output = signatureString + "\n" +
                  "Description: " + description + "\n";
            if (nlu) {
                output +=
                        "Example Translation: \"" +
                                example.instruction +
                                "\" -> {\"Semantics\":\"" +
                                example.semantics +
                                "\",\"Input Type\":\"INSTRUCT\",\"Confidence\":\"" +
                                example.confidence +
                                "\"}\n";
            }
            output += "#\n";
            return output;
        }
    }
    private class LLMNLExample {
        String instruction;
        String semantics;
        int confidence;

        LLMNLExample(String instruction, String semantics, int confidence) {
            this.instruction = instruction;
            this.semantics = semantics;
            this.confidence = confidence;
        }
    }
    private List<LLMActionEntry> exposedActionList = new ArrayList<>();


    @Override
    protected void init() {
        super.init();
        generateExposedActionList();
    }

    @Override
    protected void executionLoop () {
        super.executionLoop();
        if (!registeredForNewActions) {
            registerForNotifications();
        }
        if (!retrievedLearnedActions) {
            initLearnedActions();
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
            if (confidence < 5) {
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

        Symbol actor = utterance.getAddressee();
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
                    argsWithActor.add(actor);
                    if (innerArg.isTerm()) {
                        argsWithActor.addAll(((Term) ((Term) utterance.getSemantics()).get(0)).getArgs());
                    }
                    utterance.setSemantics(Factory.createPredicate(command, Factory.createPredicate(innerArg.getName(), argsWithActor)));
                }
            } else if (command.equals("forgetTaughtAction") || command.equals("reportIfCanPerform") || command.equals("describeActionSteps") || command.equals("urgentPriority")) {
                //Not currently prompted to do this
                if (utterance.getSemantics().hasArgs()) {
                    Symbol innerArg = ((Term)utterance.getSemantics()).get(1);
                    List<Symbol> argsWithActor = new ArrayList<>();
                    argsWithActor.add(actor);
                    if (innerArg.isTerm()) {
                        argsWithActor.addAll(((Term) ((Term) utterance.getSemantics()).get(1)).getArgs());
                    }
                    utterance.setSemantics(Factory.createPredicate(command, actor, Factory.createPredicate(innerArg.getName(), argsWithActor)));
                }
            }
        } catch (Exception e) {
            log.warn("[correctCommonNLUMistakes] unexpected semantics format");
            return false;
        }
        return true;
    }

    private List<Symbol> processSemantics(Predicate semantics, List<Symbol> processedArguments, Map<Symbol,Variable> varMap, Utterance.Builder output, String actor) {
        return processSemantics(semantics,processedArguments,varMap, output, actor, false);
    }
    private List<Symbol> processSemantics(Predicate semantics, List<Symbol> processedArguments, Map<Symbol,Variable> varMap, Utterance.Builder output, String actor, boolean unpopulatedArgSafe) {
        if (semantics.getName().equals("reportIfCanPerform")) {
            unpopulatedArgSafe = true;
        }
        for (Symbol arg: semantics.getArgs()) {
            if (arg.isPredicate()) {
                List<Symbol> innerArgs = processSemantics((Predicate)arg,new ArrayList<>(),varMap,output,actor,unpopulatedArgSafe);
                if (innerArgs == null) {
                    return null;
                }
                processedArguments.add(Factory.createPredicate(arg.getName(), innerArgs));
            } else {
                String argString = arg.toString();
                if (argString.startsWith("?")) {
                    if (unpopulatedArgSafe) {
                        processedArguments.add(Factory.createSymbol(argString.substring(1).split(":")[0]));
                        continue;
                    } else {
                        log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}", argString);
                        return null;
                    }
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
            userMessage = validationUserPrompt.apply(promptVars);
        } else {
            userMessage = nlgUserPrompt.apply(promptVars);
        }
        chat.addUserMessage(userMessage);

        promptVars = new LinkedHashMap<>();
        promptVars.put("%conversationHistory", getConversationHistoryString(conversationHistory));
        promptVars.put("%language", language);
        promptVars.put("%actionList", generateLLMActionListString(false));
        promptVars.put("%learnedActions%",learnedActionsString);
        promptVars.put("%patrolPathList", String.join("\n", patrolPaths));
        promptVars.put("%locationList", String.join("\n", locations));
        promptVars.put("%addressee", utterance.getAddressee().getName());
        if (semantics.getName().equals("mean")) {
            chat.setSystem(validationSystemPrompt.apply(promptVars));
        } else {
            chat.setSystem(nlgSystemPrompt.apply(promptVars));
        }

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
        List<String> videos = getVideoNames();
        List<String> images = getImageNames();
        String patrolPathList = String.join(":path\n", patrolPaths);
        if (!patrolPathList.isEmpty()) {
            patrolPathList += ":path\n";
        }
        String locationList = String.join(":location\n", locations);
        if (!locationList.isEmpty()) {
            locationList += ":location\n";
        }
        String videoList = String.join(":video\n", videos);
        if (!videoList.isEmpty()) {
            videoList += ":video\n";
        }
        String imageList = String.join(":image\n", images);
        if (!videoList.isEmpty()) {
            imageList += ":image\n";
        }
        promptVars.put("%actionList", generateLLMActionListString(true));
        promptVars.put("%patrolPathList", patrolPathList);
        promptVars.put("%locationList", locationList);
        promptVars.put("%videoList", videoList);
        promptVars.put("%exampleVideo", videos.isEmpty() ? "test" : videos.get(0));
        promptVars.put("%exampleImage", images.isEmpty() ? "test" : images.get(0));
        promptVars.put("%imageList", imageList);
        promptVars.put("%worldState", getWorldStateString(utterance.getAddressee()));
        promptVars.put("%goals", getGoalString(utterance.getAddressee()));
        promptVars.put("%questionList", getCurrentQuestionsString(utterance.getAddressee()));
        promptVars.put("%learnedActions%",learnedActionsString);
        promptVars.put("%exampleLocation", locations.isEmpty() ? "test location" : locations.get(0));
        promptVars.put("%examplePath", patrolPaths.isEmpty() ? "test path" : patrolPaths.get(0));
        populatePromptVarsWithProperties(promptVars, locations);
        currentChat.setSystem(nluSystemPrompt.apply(promptVars));
        //promptVars.clear();
        promptVars.put("%utterance", utterance.getWordsAsString());
        currentChat.addUserMessage(nluUserPrompt.apply(promptVars));

        return currentChat;
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

    private List<String> getVideoNames() {
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideoNames").argTypes(Boolean.class)).call(List.class,true);
        } catch (TRADEException e) {
            log.error("[getVideoNames]", e);
        }
        return new ArrayList<>();
    }

    private List<String> getImageNames() {
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageNames").argTypes(Boolean.class)).call(List.class,true);
        } catch (TRADEException e) {
            log.error("[getImageNames]", e);
        }
        return new ArrayList<>();
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
        Predicate query = Factory.createPredicate("at","X","Y");
        List<Map<Variable, Symbol>> bindings;
        String robotLocationString = "";
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
                        robotLocationString = String.format("The robot is at the following location: %s.\n", ref.properties.get(0).getName());
                    }
                }
            }
        } catch (TRADEException e) {
            log.error("[getWorldState] ", e);
            return "Nothing is known by the robot";
        }

        String output = robotLocationString;
        if (output.isEmpty()) {
            log.warn("[getWorldState] no matching beliefs found for agent {}", agent);
            output = "<Nothing is known by the robot>";
        }
        return output;
    }

    private String getGoalString(Symbol agent) {
        //TODO: also add pending and scheduled?
        String currentGoalString = "<The robot has no current task>\n";
        try {
            List<Predicate> currentGoalPredicate = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSystemGoalsPredicates").argTypes(Symbol.class)).call(List.class,agent);
            if (!currentGoalPredicate.isEmpty()) {
                currentGoalString = currentGoalPredicate.get(0).toString() + "\n";
            }
        } catch (TRADEException e) {
            log.error("[getWorldStateString] error getting current goal for agent", e);
        }
        return currentGoalString;
    }

    private String getCurrentQuestionsString(Symbol agent) {
        String output = "";
        try {
            String questionString = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestion").argTypes(Symbol.class)).call(String.class,agent);
            List<Term> expectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestionResponseForm").argTypes(Symbol.class)).call(List.class,agent);
            if (!expectedResponseForms.isEmpty()) {
                output = questionString + "\nEXPECTED REPLY FORM:\n" + String.join("\n",expectedResponseForms.stream().map(Term::toString).toList());
            }
        } catch (TRADEException e) {
            log.error("[getCurrentQuestionsString] error getting last question for agent");
        }
        if (output.isEmpty()) {
            output = "<There are no current outstanding questions>";
        }
        return output;
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

    private void initLearnedActions() {
        try {
            List<Predicate> learnedActionPredicates = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLearnedActionPredicates")).call(List.class);
            for (Predicate pred : learnedActionPredicates) {
                learnedActionsString += String.format("%s\nDescription: The user has previously manually taught the definition of this action through the use of the learnNewTask and endActionLearning actions. It will be referred to as \"%s\" when instructed by the user.\n#\n", pred, pred.getName());
            }
            retrievedLearnedActions = true;
        } catch (TRADEException e) {
            log.debug("[initLearnedActions] error getting taught actions from EM on startup");
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
            log.error("[shutdownComponent]", e);
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
                case "describeScheduledGoals":
                case "describeCurrentLocation":
                case "describePendingGoals":
                case "describeAllKnownLocations":
                case "describeAllKnownPatrolPaths":
                case "describeAllLocationsOnPatrolPath":
                case "repeatLastQuestion":
                case "describeCurrentGoal":
                case "reportTaughtActions":
                case "reportInstructableActions":
                case "describeActionSteps":
                case "reportIfCanPerform":
                case "reportKioskSettings":
                case "listSavedVideos":
                case "reportVideoVolume":
                case "listSavedImages":
                case "reportIsVideoSaved":
                case "sayText":
                case "say":
                    return true;
                default:
                    return false;
            }
        }
        return false;
    }

    //TODO: Able to move at least some of this to the base class?
    @TRADEService
    public List<String> getInstructableActions() {
        return new ArrayList<>(exposedActionList.stream().map(a -> a.signature.split("\\(")[0]).toList());
    }

    private String generateLLMActionListString(boolean nlu) {
        StringBuilder sb = new StringBuilder();
        for (LLMActionEntry entry : exposedActionList) {
            sb.append(entry.toString(nlu));
        }
        return sb.toString();
    }

    private void generateExposedActionList() {
        exposedActionList = new ArrayList<>();
        LLMActionEntry entry;
        entry = new LLMActionEntry("goToLocation(?destination:location)",
                "You go to location ?destination:location.",
                "please navigate to %exampleLocation",
                "goToLocation(%exampleLocation:location)",
                10
        );
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportBatteryLevel()",
                "You report the current battery level of the robot as a percentage",
                "how much charge do you have",
                "reportBatteryLevel()",
                10
        );
        exposedActionList.add(entry);
        entry = new LLMActionEntry("follow()",
                "You follow the user",
                "follow me",
                "follow()",
                10
        );
        exposedActionList.add(entry);
        entry = new LLMActionEntry("say(?words)",
                "You say ?words",
                "say hello, it's me",
                "say(hello, it's me)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("stopFollow()",
                "You stop following the user",
                "stop following me",
                "stopFollow()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("escortToLocation(?name, ?destination:location)",
                "You escort ?name to location ?destination:location",
                "bring Owen to %exampleLocation",
                "escortToLocation(owen,%exampleLocation:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("escortFromLocationToLocation(?name, ?start:location, ?destination:location)",
                "You escort ?name from location ?start:location to location ?destination:location",
                "bring our guest from %exampleLocation to the dock",
                "escortFromLocationToLocation(guest,%exampleLocation:location,dock:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("saveLocation(?name)",
                "You save your current location under the name ?name",
                "save this area as location one",
                "saveLocation(location one)", 10);
        exposedActionList.add(entry);
        //entry = new LLMActionEntry("saveLocationsToFile()",
        //        "You backup all your currently saved locations",
        //        "backup your current locations",
        //        "saveLocationsToFile()", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("definePatrolPath(?name)",
        //        "You start learning a new patrol path called ?name",
        //        "I want to teach you a new patrol path named lab circuit",
        //        "definePatrolPath(lab circuit)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("definePatrolPath()",
        //        "You start learning a new patrol path",
        //        "I want to teach you a new patrol path",
        //        "definePatrolPath()", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("patrol(?pathName:path, ?actionName)",
        //        "You navigate along the patrol path ?pathName:path while performing the action ?actionName at each location",
        //        "perform %examplePath patrol with wifi evaluation",
        //        "patrol(%examplePath:path, wifiEvaluation)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("wifiEvaluation(?location:location)",
        //        "You measure the strength of the wifi signal at location ?location:location",
        //        "check wifi near %exampleLocation",
        //        "wifiEvaluation(%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("reportWifiForLocation(?location:location)",
        //        "You report the wifi signal strength at location ?location:location",
        //        "what is the wifi signal quality at %exampleLocation",
        //        "reportWifiForLocation(%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("reportWifiForPatrol(?pathName:path)",
        //        "You report the wifi signal quality at every location along patrol path ?pathName:path",
        //        "report the wifi signal quality for patrol path %examplePath",
        //        "reportWifiForPatrol(%examplePath:path)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("takePhoto(?location:location)",
        //        "You take a photo at location ?location:location",
        //        "take a photo of %exampleLocation",
        //        "takePhoto(%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        entry = new LLMActionEntry("sit()",
                "You lower your screen",
                "sit down",
                "sit()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("stand()",
                "You raise your screen",
                "stand up",
                "stand()", 10);
        exposedActionList.add(entry);
        //entry = new LLMActionEntry("deliver(?objectName, ?location:location)",
        //        "You deliver ?objectName to location ?location:location",
        //        "bring the mug to %exampleLocation",
        //        "deliver(mug,%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        entry = new LLMActionEntry("goHome()",
                "You go to your charging dock and charge",
                "go back to charge",
                "goHome()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("scheduleGoal()",
                "You begin a dialogue interaction with the user to schedule future execution of a task",
                "I want to schedule a new task",
                "scheduleGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("modifyScheduledGoal()",
                "You begin a dialogue interaction with the user to modify the timing of a future already scheduled task",
                "I want to change the timing of an action",
                "modifyScheduledGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeScheduledGoals()",
                "You report what your currently scheduled goals are",
                "what are you scheduled to do",
                "describeScheduledGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeCurrentGoal()",
                "You report what your current task is",
                "what are you doing now",
                "describeCurrentGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describePendingGoals()",
                "You report what your currently pending/upcoming tasks are",
                "what are you doing after your current task",
                "describePendingGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelSystemGoals()",
                "You stop and fully discard your current task",
                "stop your current task",
                "cancelSystemGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("suspendSystemGoalsBlocking()",
                "You pause your current task, which can be later resumed",
                "pause what you're doing",
                "suspendSystemGoalsBlocking()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("resumeSystemGoals()",
                "You resume the currently paused task",
                "resume previous task",
                "resumeSystemGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelAllScheduledGoals()",
                "You cancel all currently scheduled goals which have yet to begin execution",
                "cancel all scheduled tasks",
                "cancelAllScheduledGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelScheduledGoal()",
                "You begin a dialogue interaction with the user to cancel a particular scheduled goal",
                "I want to cancel a scheduled task",
                "cancelScheduledGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("learnNewTask()",
                "You begin a dialogue interaction with the user to learn the steps of a new instructable action",
                "i will teach you a new task",
                "learnNewTask()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("endActionLearning()",
                "You conclude learning the steps of a new instructable action",
                "those are the steps of the task",
                "endActionLearning()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeCurrentLocation()",
                "You report what location you believe you are currently at",
                "what is your current location",
                "describeCurrentLocation()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeAllKnownLocations()",
                "You list all currently saved locations",
                "what are your saved locations",
                "describeAllKnownLocations()", 10);
        exposedActionList.add(entry);
        //entry = new LLMActionEntry("describeAllKnownPatrolPaths()",
        //        "You list all currently saved patrol paths",
        //        "what paths do you know about",
        //        "describeAllKnownPatrolPaths()", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("describeAllLocationsOnPatrolPath(?pathName:path)",
        //        "You list all locations included along the saved patrol path named ?pathName:path",
        //        "what locations are in %examplePath",
        //        "describeAllLocationsOnPatrolPath(%examplePath:path)", 10);
        //exposedActionList.add(entry);
        entry = new LLMActionEntry("repeatLastQuestion()",
                "You repeat the last question you asked, which the human still needs to answer",
                "could you repeat that",
                "repeatLastQuestion()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportTaughtActions()",
                "You list all of the actions which have been taught to you by a user",
                "What actions have you been taught",
                "reportTaughtActions()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("forgetTaughtAction(?actionPredicate)",
                "You forget a previously taught action represented by ?actionPredicate. ?actionPredicate should be replaced by a populated instance of an INSTRUCT Semantics from the previous examples.",
                "forget how to go to %exampleLocation",
                "forgetTaughtAction(goToLocation(%exampleLocation:location))", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportIfCanPerform(?actionPredicate)",
                "You report whether you know how to perform the action represented by ?actionPredicate. ?actionPredicate should be replaced by a populated instance of an INSTRUCT Semantics from the previous examples.",
                "do you know how to escort a patient from %exampleLocation to the dock",
                "reportIfCanPerform(escortFromLocationToLocation(guest,%exampleLocation:location,dock:location))", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportInstructableActions()",
                "You report the list of actions a user is able to instruct you through natural language to perform",
                "what can you be instructed to do",
                "reportInstructableActions())", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeActionSteps(?action)",
                "You describe an action by listing out the steps which would be performed when it is executed. ?action can be replaced by a populated instance of an INSTRUCT Semantics from the previous examples OR be just the name of an action if no specific arguments are supplied.",
                "How would you escort someone",
                "describeActionSteps(escort)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("learnTourAction()",
                "You begin a dialogue interaction with the user where they teach you how to perform a new type of tour",
                "I want to teach you how to perform a tour",
                "learnTourAction()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportKioskSettings()",
                "You report the current configuration of kiosk mode; Whether it is enabled and the currently configured kiosk timeout and action.",
                "what is your kiosk configuration",
                "reportKioskSettings()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("updateKioskTimeout()",
                "You begin a dialogue interaction with the user to update the configured kiosk timeout",
                "I would like to update the kiosk timeout",
                "updateKioskTimeout()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("setKioskTimeout(?duration, ?timeunit)",
                "You set the kiosk timeout to ?duration ?timeunits. ?timeunit can take the possible values of the Java TIMEUNIT enum.",
                "Set the kiosk timeout to 30 minutes",
                "setKioskTimeout(30,minutes)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("setKioskModeEnabled(?enabled)",
                "You set whether or not kiosk mode is enabled. ?enabled can take the values true or false",
                "turn on kiosk mode",
                "setKioskModeEnabled(true)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("updateKioskAction()",
                "You begin a dialogue interaction with the user to update the configured kiosk action",
                "I want to change the kiosk action",
                "updateKioskAction()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("playNextVideo()",
                "You begin playback of the next video in your saved collection, continuing where you left off",
                "Play video",
                "playNextVideo()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("playSpecificVideo(?videoName:video)",
                "You begin playback of the specific video ?videoName",
                "Show the %exampleVideo video",
                "playNextVideo(%exampleVideo)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("playNextVideoAtLocation(?location:location)",
                "You go to ?location and then begin playback of the next video in your saved collection, continuing where you left off",
                "Play videos at %exampleLocation",
                "playNextVideoAtLocation(%exampleLocation:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("playSpecificVideoAtLocation(?videoName:video, ?location:location)",
                "You go to ?location and then begin playback of the specific video ?videoName",
                "Go to %exampleLocation and play the %exampleVideo video.",
                "playSpecificVideoAtLocation(%exampleVideo,%exampleLocation:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("setVideoVolume(?volume)",
                "You set the volume of videos played on your screen. Acceptable values are whole numbers between 0 and 100.",
                "Set the video volume to max",
                "setVideoVolume(100)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("listSavedVideos()",
                "You list out the collection of videos you currently have saved",
                "what videos can you play",
                "listSavedVideos()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportVideoVolume()",
                "You report the current volume videos are set to play at",
                "what is the video volume set to",
                "reportVideoVolume()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportIsVideoSaved(?videoName:video)",
                "You answer whether you have a video saved under the name ?videoName",
                "Do you have the %exampleVideo video saved",
                "reportIsVideoSaved(?videoName)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("waitForDuration(?seconds)",
                "You wait idly for ?seconds seconds. ?seconds must be a positive whole number.",
                "Then wait for a minute",
                "waitForDuration(60)", 10);
        exposedActionList.add(entry);
        //TODO: for a lot the video and image actions - allow then() semantics to be targeted and handled generally rather
        //      than making these variations of the same method (or revisit this specific model and prompt format and
        //      100% prevent misparamaterized actions from being generated - or have smarter recovery in these cases).
        //      e.g. then(goToLocation(...),playVideo(...),waitForDuration(...))
        entry = new LLMActionEntry("displayNamedImage(?imageName:image)",
                "You display the image ?imageName at your current location until told otherwise",
                "Display the %exampleImage image",
                "displayNamedImage(%exampleImage)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("displayNamedImageAtLocation(?imageName:image,?location:location)",
                "You go to location ?location then display the image ?imageName until told otherwise",
                "Display the %exampleImage image at the %exampleLocation location",
                "displayNamedImageAtLocation(%exampleImage,%exampleLocation:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("displayNamedImageForDuration(?imageName:image,?seconds)",
                "You display the image ?imageName for ?seconds seconds",
                "Display the %exampleImage image for a minute",
                "displayNamedImageForDuration(%exampleImage,60)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("displayNamedImageForDurationAtLocation(?imageName:image,?location:location,?seconds)",
                "You go to location ?location then display the image ?imageName for ?seconds seconds",
                "Display the %exampleImage image at the %exampleLocation location for 45 seconds",
                "displayNamedImageForDurationAtLocation(%exampleImage,%exampleLocation:location,45)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("listSavedImages()",
                "You list out the collection of images you currently have saved",
                "what images can you display",
                "listSavedImages()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("urgentPriority(?actionPredicate)",
                "You submit an action represented by ?actionPredicate with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. ?actionPredicate should be replaced by a populated instance of an INSTRUCT Semantics from the previous examples.",
                "go to %exampleLocation right now",
                "urgentPriority(goToLocation(%exampleLocation:location))", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelTaskInQueueByIndex(?index)",
                "You cancel and remove the task in task queue at position ?index if it exists. ?index is a 0-based index.",
                "Remove the first task from the queue",
                "cancelTaskInQueueByIndex(0)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("enableAdminMode()",
                "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.",
                "enable admin mode",
                "enableAdminMode()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("disableAdminMode()",
                "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.",
                "disable admin mode",
                "disableAdminMode()", 10);
        exposedActionList.add(entry);
    }
}
