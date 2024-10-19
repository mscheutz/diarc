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
import org.apache.commons.lang3.tuple.Pair;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PickAndPlace2024LLMServiceProvider extends LLMServiceProvider {
    Prompt nluSystemPrompt = Prompts.getPrompt("pickandplace2024/nlu/english/system");;
    Prompt nluUserPrompt = Prompts.getPrompt("pickandplace2024/nlu/english/user");
    Prompt nlgSystemPrompt = Prompts.getPrompt("pickandplace2024/nlg/english/system");
    Prompt nlgUserPrompt = Prompts.getPrompt("pickandplace2024/nlg/english/user");
    Prompt validationPrompt = Prompts.getPrompt("pickandplace2024/nlg/english/validation");

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
            nluSystemPrompt = Prompts.getPrompt("pickandplace2024/nlu/japanese/system");;
            nluUserPrompt = Prompts.getPrompt("pickandplace2024/nlu/japanese/user");
            nlgSystemPrompt = Prompts.getPrompt("pickandplace2024/nlg/japanese/system");
            nlgUserPrompt = Prompts.getPrompt("pickandplace2024/nlg/japanese/user");
            validationPrompt = Prompts.getPrompt("pickandplace2024/nlg/japanese/validation");
        }
    }

    @Override
    public Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse) {
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
        Map<Symbol,Variable> varMap = new LinkedHashMap<>();
        for (String arg : argsStrings) {
            if (arg.startsWith("?")) {
                log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}",responseString);
                return incomingUtterance;
            }
            Symbol argSymbol = Factory.createFOL(arg);
            //Should generally be any reference, just know in this case all references will be locations
            if (argSymbol != null && (argSymbol.getType().equals("pose") || argSymbol.getType().equals("physobj"))) {
                if (!varMap.containsKey(argSymbol)) {
                    Variable var = Factory.createVariable("VAR" + varIndex++ + ":" + argSymbol.getType());
                    varMap.put(argSymbol, var);
                }
            }
        }

        for (Map.Entry<Symbol,Variable> entry : varMap.entrySet()) {
            semanticsString = semanticsString.replace(entry.getKey().toString(), entry.getValue().toString());
            output.addSupplementalSemantics(Factory.createPredicate(entry.getKey().toUntypedString(), entry.getValue()));

            Symbol tier = Factory.createSymbol("DEFINITE");
            output.addTierAssignment(entry.getValue(), tier);
        }
        output.setSemantics(Factory.createPredicate(semanticsString));

        output.setNeedsValidation(true);

        return output.build();
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
        StringBuilder builder = new StringBuilder();
        if (conversationHistory.isEmpty()) {
            builder.append("<there is no conversation history>");
        } else {
            for (Utterance pastUtterance : conversationHistory) {
                builder.append(pastUtterance.getSpeaker()).append(": ").append(pastUtterance.getWordsAsString()).append("\n");
            }
        }

        Map<String,String> promptVars = new HashMap<>();
        if (semantics.getName().equals("doYouMean")) {
            promptVars.put("%utterance", utterance.toString());
            userMessage = validationPrompt.apply(promptVars);
        } else {
            utterance = new Utterance.Builder(utterance).build();
            //Doesn't include suppSemantics
            semantics = replaceReferencesInSemantics(semantics);
            utterance.setSemantics(semantics);
            promptVars.put("%utterance", utterance.toString());
            userMessage = nlgUserPrompt.apply(promptVars);
        }

        //TODO (same as NLU)
        //Map<String,String> nlgSystemPromptParameters = new HashMap<>();
        //////nlgSystemPromptParameters.put("%conversationHistory%",conversationHistory);
        //chat.setSystem(nlgSystemPrompt.apply(nlgSystemPromptParameters));
        //Map<String,String> nlgUserPromptParameters = new HashMap<>();
        ////nlgUserPromptParameters.put("%utteranceSemantics%",utterance.getWordsAsString());
        //chat.addUserMessage(nlgUserPrompt.apply(nlgUserPromptParameters));

        promptVars = new HashMap<>();
        promptVars.put("%conversationHistory", builder.toString());
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

        //TODO: I tried writing this prompt in a format which is closer to a general template which can have domain
        //  specific information applied. Swap out information from the prompt with identifiers and apply parameters
        //  rather than hardcode the information
        //Map<String,String> nluSystemPromptParameters = new HashMap<>();
        //////nluSystemPromptParameters.put("%conversationHistory%",conversationHistory);
        //currentChat.setSystem(nluSystemPrompt.apply(nluSystemPromptParameters));
        //Map<String,String> nluUserPromptParameters = new HashMap<>();
        ////nluUserPromptParameters.put("%utteranceSemantics%",utterance.getWordsAsString());
        //currentChat.addUserMessage(nluUserPrompt.apply(nluUserPromptParameters));
        StringBuilder builder = new StringBuilder();
        if (conversationHistory.isEmpty()) {
            builder.append("<there is no conversation history>");
        } else {
            for (Utterance pastUtterance : conversationHistory) {
                builder.append(pastUtterance.getSpeaker()).append(": ").append(pastUtterance.getWordsAsString()).append("\n");
            }
        }
        Map<String,String> promptVars = new HashMap<>();
        promptVars.put("%conversationHistory", builder.toString());

        currentChat.setSystem(nluSystemPrompt.apply(promptVars));
        currentChat.addUserMessage(nluUserPrompt.getText() + utterance.getWordsAsString());

        return currentChat;
    }

    @Action
    @TRADEService
    public void testLLMNLGWithString(String semantics) {
        Chat chat = new Chat();
        List<Utterance> conversationHistory = new ArrayList<>();
        Predicate semanticsPred = Factory.createPredicate(semantics);
        Utterance utterance = new Utterance.Builder().setSemantics(semanticsPred).setIsInputUtterance(false).setUtteranceType(UtteranceType.REPLY).setSpeaker(Factory.createSymbol("robotone:agent")).setListener(Factory.createSymbol("pete")).build();
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

    //Hacky impl of getting japanese version of descriptors used in action scripts for TTS
    @TRADEService
    public String getJapaneseVersionOfDescriptor(Symbol descriptor) {
        String descriptorString = descriptor.getName();
        HashMap<String,String> descriptorMappings = new HashMap<>();
        descriptorMappings.put("red block", "赤いブロック");
        descriptorMappings.put("green block", "緑色のブロック");
        descriptorMappings.put("blue block", "青いブロック");
        descriptorMappings.put("orange block", "オレンジ色のブロック");
        descriptorMappings.put("yellow block", "黄色いブロック");
        descriptorMappings.put("location one", "位置1");
        descriptorMappings.put("location two", "位置2");
        descriptorMappings.put("location three", "位置3");
        descriptorMappings.put("location four", "位置4");
        descriptorMappings.put("location five", "位置5");
        descriptorMappings.put("locationOne", "位置1");
        descriptorMappings.put("locationTwo", "位置2");
        descriptorMappings.put("locationThree", "位置3");
        descriptorMappings.put("locationFour", "位置4");
        descriptorMappings.put("locationFive", "位置5");
        if (descriptorMappings.get(descriptorString) == null) {
            return descriptorString;
        } else {
            return descriptorMappings.get(descriptorString);
        }
    }
}
