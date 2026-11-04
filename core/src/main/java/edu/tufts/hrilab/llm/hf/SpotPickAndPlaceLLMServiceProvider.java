/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//This layer does not need to be abstract for now. If have more examples that do the same thing similarly, then can
//  have an additional abstraction (Or even when this is actually working with a prompt we can make a prompt template
//  out of it which this class can abstractly use and subclasses can provide specific prompt parameters only
//public abstract class DefaultLLMServiceProvider extends LLMServiceProvider {
public class SpotPickAndPlaceLLMServiceProvider extends LLMServiceProvider {
    Prompt nluSystemPrompt = Prompts.getPrompt("spotpickandplace/nlu/system");;
    Prompt nluUserPrompt = Prompts.getPrompt("spotpickandplace/nlu/user");
    Prompt nlgSystemPrompt = Prompts.getPrompt("spotpickandplace/nlg/system");
    Prompt nlgUserPrompt = Prompts.getPrompt("spotpickandplace/nlg/user");
    Prompt validationPrompt = Prompts.getPrompt("spotpickandplace/nlg/validation");


    @Override
    public Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse) {
        Utterance.Builder output = new Utterance.Builder(incomingUtterance);
        log.info(llmResponse.getText());
        String responseString = llmResponse.getText();

        Pattern semanticsPattern = Pattern.compile(".*Semantics:.*\\s(.*)\\((.*?)\\).*", Pattern.DOTALL);
        Matcher m = semanticsPattern.matcher(responseString);
        if (!m.matches()) {
            log.error("[spotPickAndPlaceLLMParser] cannot find Semantics result in LLM response");
            return incomingUtterance;
        }
        String name = m.group(m.groupCount()-1);
        String argString= m.group(m.groupCount());
        argString = argString.replace("\s","");
        String[] argsStrings;
        if (argString.isEmpty()) {
            argsStrings = new String[0];
        } else {
            argsStrings = argString.split(",");
        }

        Pattern argumentTypesPattern = Pattern.compile(".*Argument Types: (.*)Input Type:.*", Pattern.DOTALL);
        m = argumentTypesPattern.matcher(responseString);
        if (!m.matches()) {
            log.error("[spotPickAndPlaceLLMParser] cannot find Argument Types result in LLM response");
            return incomingUtterance;
        }
        List<String> argumentTypes = Arrays.stream(m.group(m.groupCount()).split(",")).toList();

        Pattern typePattern = Pattern.compile(".*Input Type: (.*)Confidence:.*", Pattern.DOTALL);
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

        Pattern confidencePattern = Pattern.compile(".*Confidence: (.*)", Pattern.DOTALL);
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
        List<Symbol> args = new ArrayList<>();
        List<Variable> vars = new ArrayList<>();
        List<String> descriptors = new ArrayList<>();
        for (String arg : argsStrings) {
            Symbol argSymbol = Factory.createFOL(arg);
            //TODO: should generally be any reference, just know in this case all references will be locations
            if (argSymbol.getType().equals("location") || argSymbol.getType().equals("physobj")) {
                Variable var = Factory.createVariable("VAR"+varIndex+++":"+argSymbol.getType());
                vars.add(var);
                descriptors.add(argSymbol.getName());
                args.add(var);
            } else {
                args.add(Factory.createFOL(arg));
            }
        }

        //Only allowing this to be incorrect because I haven't had much time to test this and am not confident there won't be malformed results
        boolean checkType = true;
        if (!vars.isEmpty() && argumentTypes.size() != vars.size()) {
            log.warn("[spotPickAndPlaceLLMServiceProvider] mismatch between argument type length and vars length, malformed response. Assuming all refs are DEFINITE");
            checkType = false;
        }
        output.setSemantics(Factory.createPredicate(name,args));
        for (int i=0; i<vars.size(); i++) {
            output.addSupplementalSemantics(Factory.createPredicate(descriptors.get(i), vars.get(i)));

            Symbol tier = Factory.createSymbol("DEFINITE");
            if (checkType) {
                String argTypeString = argumentTypes.get(i);
                if (!argTypeString.trim().equals("DEFINITE")) {
                    tier = Factory.createSymbol("INDEFINITE");
                }
            }
            output.addTierAssignment(vars.get(i), tier);
        }

        output.setNeedsValidation(true);

        return output.build();
    }

    @Override
    public Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse) {
        log.info(llmResponse.getText());
        String responseString = llmResponse.getText().trim();
        outgoingUtterance.setWords(List.of(responseString.split(" ")));

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
        promptVars.put("%utterance", utterance.toString());
        if (semantics.getName().equals("doYouMean")) {
            userMessage = validationPrompt.apply(promptVars);
        } else {
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

        log.info("generateNLGChat called for: {}", utterance);

        return chat;
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

    //public abstract Set<Symbol> getRelevantAgents();
    //public abstract Set<Symbol> getRelevantTypes();
    //public abstract Set<Symbol> getRelevantActivatedEntities();
    //public abstract Set<Predicate> getRelevant
    //public abstract String getRoleDescription();
    //public abstract String getTaskDescription();

}
