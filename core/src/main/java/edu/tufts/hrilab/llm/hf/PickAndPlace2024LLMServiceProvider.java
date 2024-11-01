/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PickAndPlace2024LLMServiceProvider extends LLMServiceProvider {
    Prompt nluSystemPrompt = Prompts.getPrompt("pickandplace2024/nlu/systemv3");
    Prompt nluUserPrompt = Prompts.getPrompt("pickandplace2024/nlu/english/userv3");
    Prompt nlgSystemPrompt = Prompts.getPrompt("pickandplace2024/nlg/system");
    Prompt nlgUserPrompt = Prompts.getPrompt("pickandplace2024/nlg/english/user");
    Prompt validationPrompt = Prompts.getPrompt("pickandplace2024/nlg/english/validation");
    private String language = "English";
    private boolean v2 = true;

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = super.additionalUsageInfo();
        options.add(Option.builder("japanese").desc("use japanese versions of prompts").build());
        return options;
    }

    @Override
    protected void parseArgs (CommandLine cmdLine) {
        super.parseArgs(cmdLine);
        if (cmdLine.hasOption("japanese")) {
            nluSystemPrompt = Prompts.getPrompt("pickandplace2024/nlu/systemv3");
            nluUserPrompt = Prompts.getPrompt("pickandplace2024/nlu/japanese/userv3");
            nlgSystemPrompt = Prompts.getPrompt("pickandplace2024/nlg/system");
            nlgUserPrompt = Prompts.getPrompt("pickandplace2024/nlg/japanese/user");
            validationPrompt = Prompts.getPrompt("pickandplace2024/nlg/japanese/validation");
            language = "Japanese";
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
                log.error("[spotPickAndPlaceLLMParser] cannot find Semantics result in LLM response");
                return incomingUtterance;
            }
            Predicate semantics = Factory.createPredicate(m.group(m.groupCount()));
            String semanticsString = semantics.toString();
            List<String> argsStrings = getArgsAsStrings(semantics);

            Pattern typePattern = Pattern.compile(".*\"?Input Type\"?\\s*:\\s*\"?(.*?)\"?,\\s*\"?Confidence\"?:.*", Pattern.DOTALL);
            m = typePattern.matcher(responseString);
            if (!m.matches()) {
                log.error("[spotPickAndPlaceLLMParser] cannot find Input Type result in LLM response");
                return incomingUtterance;
            }
            UtteranceType utteranceType;
            try {
                utteranceType = UtteranceType.valueOf(m.group(m.groupCount()).trim());
            } catch (IllegalArgumentException e) {
                log.error("[spotPickAndPlaceLLMParser] generated Input Type {} invalid", m.group(m.groupCount()));
                return incomingUtterance;
            }

            Pattern confidencePattern = Pattern.compile(".*\"?Confidence\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
            m = confidencePattern.matcher(responseString);
            if (!m.matches()) {
                log.error("[spotPickAndPlaceLLMParser] cannot find Confidence result in LLM response");
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

            int varIndex = 0;
            Map<Symbol, Variable> varMap = new LinkedHashMap<>();
            for (String arg : argsStrings) {
                if (arg.startsWith("?")) {
                    log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}", responseString);
                    return incomingUtterance;
                }
                Symbol argSymbol;
                String descriptorType = getDescriptorType(arg);
                if (descriptorType.isEmpty()) {
                    argSymbol = Factory.createFOL(arg);
                } else {
                    //v2 only
                    argSymbol = Factory.createSymbol(arg, descriptorType);
                }
                //Map<String, String> descriptorTypes = getKnownDescriptorTypes();
                //if (descriptorTypes.containsKey(arg)) {
                //    //v2 only
                //    argSymbol = Factory.createSymbol(arg, descriptorTypes.get(arg));
                //} else {
                //    argSymbol = Factory.createFOL(arg);
                //}

                //Should generally be any reference, just know in this case all references will be locations
                if (argSymbol != null && (argSymbol.getType().equals("pose") || argSymbol.getType().equals("physobj"))) {
                    if (!varMap.containsKey(argSymbol)) {
                        Variable var = Factory.createVariable("VAR" + varIndex++ + ":" + argSymbol.getType());
                        varMap.put(argSymbol, var);
                    }
                }
            }

            for (Map.Entry<Symbol, Variable> entry : varMap.entrySet()) {
                if (v2) {
                    semanticsString = semanticsString.replace(entry.getKey().getName(), entry.getValue().toString());
                } else {
                    semanticsString = semanticsString.replace(entry.getKey().toString(), entry.getValue().toString());
                }
                output.addSupplementalSemantics(Factory.createPredicate(entry.getKey().toUntypedString(), entry.getValue()));

                Symbol tier = Factory.createSymbol("DEFINITE");
                output.addTierAssignment(entry.getValue(), tier);
            }

            if (v2 && utteranceType == UtteranceType.INSTRUCT) {
                String actorName = incomingUtterance.getAddressee().getName();
                semanticsString = semanticsString.replaceFirst("\\(", "("+actorName+",");
                if (semanticsString.endsWith(",)")) {
                    semanticsString = semanticsString.replace(",)", ")");
                }
            }
            output.setSemantics(Factory.createPredicate(semanticsString));

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
        String actorName = utterance.getAddressee().getName();
        try {
            String command = utterance.getSemantics().getName();
            if (command.equals("believeObjectAt")) {
                utterance.setType(UtteranceType.INSTRUCT);
                if (v2) {
                    List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                    args.add(0, Factory.createSymbol(actorName));
                    utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
                }
            } else if (command.startsWith("describe")) {
                utterance.setType(UtteranceType.INSTRUCT);
                if (v2) {
                    List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                    args.add(0, Factory.createSymbol(actorName));
                    utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
                }
            } else if (command.equals("goTo")) {
                List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                utterance.setSemantics(Factory.createPredicate("goToPose", args));
            } else if (v2 && command.equals("knowHowTo")) {
                List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                args.add(0, Factory.createSymbol(actorName));
                Term knownPredicate = (Term) args.remove(1);
                List<Symbol> innerArgs = knownPredicate.getArgs();
                innerArgs.add(0,Factory.createSymbol(actorName));
                args.add(Factory.createPredicate(knownPredicate.getName(), innerArgs));
                utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
            //} else if (v2 && command.equals("see")) {
            //    List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
            //    args.add(0, Factory.createSymbol("robotone"));
            //    utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
            } else if (command.equals("pose")) {
                Symbol arg = ((Term)utterance.getSemantics()).get(0);
                if (!arg.isVariable() && !arg.getName().equals("none") && !(arg.isVariable() && !arg.getType().equals("pose"))) {
                    log.warn("[correctCommonNLUMistakes] LLM returned pose(?pose) parse where ?pose is not a reference. Not allowing");
                    return false;
                }
            } else if (command.equals("objectType")) {
                Symbol arg = ((Term)utterance.getSemantics()).get(0);
                if (!arg.isVariable() && !arg.getName().equals("none")&& !(arg.isVariable() && !arg.getType().equals("physobj"))) {
                    log.warn("[correctCommonNLUMistakes] LLM returned objectType(?object) parse where ?object is not a reference. Not allowing");
                    return false;
                }
            } else if (command.equals("whichObjectsAtCurrentPose")) {
                List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                args.add(0, Factory.createSymbol(actorName));
                utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
            }
        } catch (Exception e) {
            log.warn("[correctCommonNLUMistakes] unexpected semantics format");
            return false;
        }
        return true;
    }

    private List<String> getArgsAsStrings(Symbol fol) {
        List<String> argsStrings = new ArrayList<>();
        if (fol.isPredicate()) {
            for (Symbol arg: ((Predicate)fol).getArgs()) {
                if (arg.isPredicate()) {
                    argsStrings.add(arg.getName());
                    for (Symbol argsArg : ((Predicate)arg).getArgs()) {
                        argsStrings.addAll(getArgsAsStrings(argsArg));
                    }
                } else {
                    argsStrings.add(arg.toString());
                }
            }
        } else {
            argsStrings.add(fol.toString());
        }
        return argsStrings;
    }

    @Override
    public Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse) {
        log.info(llmResponse.getText());
        String responseString = llmResponse.getText().trim();
        //outgoingUtterance.setWords(List.of(responseString.split(" ")));

        Pattern outputPattern = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
        Matcher m = outputPattern.matcher(responseString);
        if (!m.matches()) {
            log.error("[parseNLGCompletionResponse] cannot find Output field in LLM response");
            try {
                Predicate semantics = (Predicate) outgoingUtterance.getSemantics();
                if (semantics.getName().equals("doYouMean")) {
                    ////Forgot this never actually worked with SimpleNLG
                    ////Go from doYouMean semantics back to mean semantics for SimpleNLG
                    //Predicate innerSemantics = (Predicate) semantics.get(1);
                    //innerSemantics = Factory.createPredicate(innerSemantics.getName(), innerSemantics.get(0), innerSemantics.get(1), innerSemantics.get(2));
                    //outgoingUtterance.setSemantics(Factory.createPredicate("mean", semantics.get(0), innerSemantics));
                    //return nlg.convertSemanticsToText(outgoingUtterance);
                    return outgoingUtterance;
                }
            } catch (Exception e) {
                log.error("[llmChatCompletionHelper] exception attempting to transform doYouMean semantics to original form for backup SimpleNLG");
            }
            return nlg.convertSemanticsToText(outgoingUtterance);
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
        Predicate semantics = (Predicate) utterance.getSemantics();
        List<String> objectsEnglish = getObjects(true);
        List<String> objectsJapanese = getObjects(false);
        List<String> englishPoses = getPoses(true);
        List<String> japanesePoses = getPoses(false);

        Map<String,String> promptVars = new LinkedHashMap<>();
        if (semantics.getName().equals("doYouMean")) {
            promptVars.put("%utterance", utterance.toString());
            promptVars = populatePromptVarsWithProperties(promptVars, objectsEnglish, objectsJapanese, englishPoses, japanesePoses);
            userMessage = validationPrompt.apply(promptVars);
        } else {
            utterance = new Utterance.Builder(utterance).build();
            //Doesn't include suppSemantics
            semantics = replaceReferencesInSemantics(semantics);
            utterance.setSemantics(semantics);
            promptVars.put("%utterance", utterance.toString());
            promptVars = populatePromptVarsWithProperties(promptVars, objectsEnglish, objectsJapanese, englishPoses, japanesePoses);
            userMessage = nlgUserPrompt.apply(promptVars);
        }
        
        promptVars = new LinkedHashMap<>();
        promptVars.put("%conversationHistory", getConversationHistoryString(conversationHistory));
        promptVars.put("%language", language);
        promptVars.put("%objectList", String.join("\n", objectsEnglish));
        promptVars.put("%locationList", String.join("\n", englishPoses));
        promptVars.put("%japaneseObjList", String.join("\n", objectsJapanese));
        promptVars.put("%japanesePoseList", String.join("\n", japanesePoses));
        chat.setSystem(nlgSystemPrompt.apply(promptVars));
        chat.addUserMessage(userMessage);

        log.info("generateNLGChat called for: {}", utterance.toString());

        return chat;
    }

    Predicate replaceReferencesInSemantics(Predicate semantics) {
        List<Symbol> transformedArgs = new ArrayList<>();
        for (Symbol arg: semantics.getArgs()) {
            if (arg.isPredicate()) {
                transformedArgs.add(replaceReferencesInSemantics((Predicate) arg));
            } else {
                if (arg.getType().equals("pose") || arg.getType().equals("physobj")) {
                    Map<Symbol, List<Term>> refs = new HashMap<>();
                    try {
                        refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
                        if (refs.get(arg).size() == 1) {
                            transformedArgs.add(Factory.createSymbol(refs.get(arg).get(0).getName(),refs.get(arg).get(0).get(0).getType()));
                        }
                        //TODO: this should only be present for testing
                        else {
                            transformedArgs.add(arg);
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
        List<String> objectsEnglish = getObjects(true);
        List<String> objectsJapanese = getObjects(false);
        List<String> englishPoses = getPoses(true);
        List<String> japanesePoses = getPoses(false);
        promptVars.put("%objectList", String.join("\n", objectsEnglish));
        promptVars.put("%locationList", String.join("\n", englishPoses));
        promptVars.put("%japaneseObjList", String.join("\n", objectsJapanese));
        promptVars.put("%japanesePoseList", String.join("\n", japanesePoses));
        promptVars.put("%worldState", getWorldStateString());
        currentChat.setSystem(nluSystemPrompt.apply(promptVars));
        promptVars.clear();
        populatePromptVarsWithProperties(promptVars, objectsEnglish, objectsJapanese, englishPoses, japanesePoses);
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

    private Map<String,String> populatePromptVarsWithProperties(Map<String,String> promptVars, List<String> objectsEnglish, List<String> objectsJapanese, List<String> posesEnglish, List<String> posesJapanese) {
        int numObjects = objectsEnglish.size();
        int numObjectsJapanese = objectsJapanese.size();
        if (numObjects != 5 || numObjectsJapanese != 5) {
            log.warn("[generateNLUChat] number of objects received is not 5");
        }

        if (numObjects > 0 && numObjectsJapanese > 0) {
            promptVars.put("the red block", objectsEnglish.get(0));
            promptVars.put("red block", objectsEnglish.get(0));
            promptVars.put("赤いブロック", objectsJapanese.get(0));
            if (numObjects > 1 && numObjectsJapanese > 1) {
                promptVars.put("the green block",objectsEnglish.get(1));
                promptVars.put("green block", objectsEnglish.get(1));
                promptVars.put("緑色のブロック", objectsJapanese.get(1));
                if (numObjects > 2 && numObjectsJapanese > 2) {
                    promptVars.put("the yellow block", objectsEnglish.get(2));
                    promptVars.put("yellow block", objectsEnglish.get(2));
                    promptVars.put("黄色いブロック", objectsJapanese.get(2));
                    if (numObjects > 3 && numObjectsJapanese > 3) {
                        promptVars.put("the blue block", objectsEnglish.get(3));
                        promptVars.put("blue block", objectsEnglish.get(3));
                        promptVars.put("青いブロック", objectsJapanese.get(3));
                        if (numObjects > 4 && numObjectsJapanese > 4) {
                            promptVars.put("the orange block", objectsEnglish.get(4));
                            promptVars.put("orange block", objectsEnglish.get(4));
                            promptVars.put("オレンジ色のブロック", objectsJapanese.get(4));
                        }
                    }
                }
            }
        }
        int numPosesEnglish = posesEnglish.size();
        int numPosesJapanese = posesJapanese.size();
        if (numPosesEnglish > 0 && numPosesJapanese > 0) {
                promptVars.put("location one", posesEnglish.get(0));
                promptVars.put("位置1", posesJapanese.get(0));
            if (numPosesEnglish > 1 && numPosesJapanese > 1) {
                    promptVars.put("location two", posesEnglish.get(1));
                    promptVars.put("位置2", posesJapanese.get(1));
                if (numPosesEnglish > 2 && numPosesJapanese > 2) {
                        promptVars.put("location three", posesEnglish.get(2));
                        promptVars.put("位置3", posesJapanese.get(2));
                    if (numPosesEnglish > 3 && numPosesJapanese > 3) {
                            promptVars.put("location four", posesEnglish.get(3));
                            promptVars.put("位置4", posesJapanese.get(3));
                    }
                }
            }
        }
        return promptVars;
    }
    
    private List<String> getObjects(boolean english) {
        try {
            List<String> propertyNames;
            if (english) {
                propertyNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEnglishObjectDescriptors")).call(List.class);
            } else {
                propertyNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getJapaneseObjectDescriptors")).call(List.class);
            }
            return propertyNames;
        } catch (TRADEException e) {
            log.error("[getObjectListString]", e);
        }

        return new ArrayList<>();
    }

    private List<String> getPoses(boolean english) {
        try {
            List<String> propertyNames;
            if (english) {
                propertyNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEnglishPoseDescriptors")).call(List.class);
            } else {
                propertyNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getJapanesePoseDescriptors")).call(List.class);
            }
            return propertyNames;
        } catch (TRADEException e) {
            log.error("[getPoseListString]", e);
        }

        return new ArrayList<>();
    }

    private String getWorldStateString() {
        ////If parser test
        //return "Nothing is known by the robot";

        StringBuilder builder = new StringBuilder();
        Predicate query = Factory.createPredicate("at","X","Y");
        List<Map<Variable, Symbol>> bindings;
        String robotLocationString = "";
        String holdingString = "";
        try {
            bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
            for (Map<Variable, Symbol> binding : bindings) {
                Symbol x = binding.get(new Variable("X"));
                Symbol y = binding.get(new Variable("Y"));
                Symbol object = TRADE.getAvailableService(new TRADEServiceConstraints().name("getDescriptorForID").argTypes(Symbol.class)).call(Symbol.class,x);
                Symbol pose = TRADE.getAvailableService(new TRADEServiceConstraints().name("getDescriptorForPoseID").argTypes(Symbol.class)).call(Symbol.class,y);
                if (object.getName().equals("none")) {
                    robotLocationString = String.format("The robot is at %s. ",pose.getName());
                } else {
                    builder.append(String.format("There is a %s at %s. ",object.getName(),pose.getName()));
                }
            }

            query = Factory.createPredicate("holding","X","Y");
            bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
            for (Map<Variable, Symbol> binding : bindings) {
                Symbol y = binding.get(new Variable("Y"));
                Symbol object = TRADE.getAvailableService(new TRADEServiceConstraints().name("getDescriptorForID").argTypes(Symbol.class)).call(Symbol.class,y);
                holdingString = String.format("The robot is holding a %s. ", object);
            }
        } catch (TRADEException e) {
            log.error("[getWorldState] ", e);
            return "Nothing is known by the robot";
        }

        return robotLocationString + holdingString + builder;
    }

    private String getDescriptorType(String descriptor) {
        try {
            List<String> propertyNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEnglishObjectDescriptors")).call(List.class);
            if (propertyNames.contains(descriptor)) {
                return "physobj";
            }
        } catch (TRADEException e) {
            log.error("[getObjectListString]", e);
        }

        try {
            List<String> propertyNames = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEnglishPoseDescriptors")).call(List.class);
            if (propertyNames.contains(descriptor)) {
                return "pose";
            }
        } catch (TRADEException e) {
            log.error("[getPoseListString]", e);
        }
        return "";
    }

    //Hacky impl of getting japanese version of descriptors used in action scripts for TTS
    @TRADEService
    public String getJapaneseVersionOfDescriptor(Symbol descriptor) {
        return getJapaneseVersionOfDescriptor(descriptor.getName());
    }
    @TRADEService
    public String getJapaneseVersionOfDescriptor(String descriptorString) {
        List<String> englishDescriptors = getObjects(true);
        List<String> japaneseDescriptors = getObjects(false);
        int index = englishDescriptors.indexOf(descriptorString);
        if (index != -1 && index < japaneseDescriptors.size()) {
            return japaneseDescriptors.get(index);
        }
        englishDescriptors = getPoses(true);
        japaneseDescriptors = getPoses(false);
        index = englishDescriptors.indexOf(descriptorString);
        if (index != -1 && index < japaneseDescriptors.size()) {
            return japaneseDescriptors.get(index);
        }
        return descriptorString;
    }
}
