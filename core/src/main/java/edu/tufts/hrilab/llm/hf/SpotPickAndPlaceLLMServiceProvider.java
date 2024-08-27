/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

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
        Map<Variable,String> vars = new HashMap<>();
        for (String arg : argsStrings) {
            Symbol argSymbol = Factory.createFOL(arg);
            //TODO: should generally be any reference, just know in this case all references will be locations
            if (argSymbol.getType().equals("location")) {
                Variable var = Factory.createVariable("VAR"+varIndex+++":"+argSymbol.getType());
                vars.put(var,argSymbol.getName());
                args.add(var);
            } else {
                args.add(Factory.createFOL(arg));
            }
        }

        output.setSemantics(Factory.createPredicate(name,args));
        for (Map.Entry<Variable,String> entry: vars.entrySet()) {
            output.addSupplementalSemantics(Factory.createPredicate(entry.getValue(), entry.getKey()));

            //TODO: don't always assume DEFINITE
            output.addTierAssignment(entry.getKey(), Factory.createSymbol("DEFINITE"));
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

        //TODO: create standalone validation prompt
        //Hack, intercept doYouMean semantics and replace vars with descriptor
        String output = utterance.toString();
        //Predicate semantics = (Predicate) utterance.getSemantics();
        //if (semantics.getName().equals("doYouMean")) {
        //    Predicate supplementalSemantics = (Predicate)semantics.getArgs().get(2);
        //    String semanticString = semantics.get(1).toString();
        //    for (Symbol suppSemSymbol: supplementalSemantics.getArgs()) {
        //        Predicate suppSemPredicate = (Predicate) suppSemSymbol;
        //        suppSemPredicate = (Predicate) suppSemPredicate.get(0);
        //        String descriptor = suppSemPredicate.getName();
        //        String var = suppSemPredicate.get(0).toString();
        //        semanticString = semanticString.replaceAll(var, descriptor);
        //    }
        //    semantics = Factory.createPredicate(semantics.getName(), semanticString);
        //    log.info(semantics.toString());
        //    output = semantics.toString();
        //} else {
        //    output = utterance.toString();
        //}


        //TODO (same as NLU)
        //Map<String,String> nlgSystemPromptParameters = new HashMap<>();
        //////nlgSystemPromptParameters.put("%conversationHistory%",conversationHistory);
        //chat.setSystem(nlgSystemPrompt.apply(nlgSystemPromptParameters));
        //Map<String,String> nlgUserPromptParameters = new HashMap<>();
        ////nlgUserPromptParameters.put("%utteranceSemantics%",utterance.getWordsAsString());
        //chat.addUserMessage(nlgUserPrompt.apply(nlgUserPromptParameters));

        chat.setSystem(nlgSystemPrompt.getText());
        chat.addUserMessage(nlgUserPrompt.getText() + output);

        log.info("generateNLGChat called with: {}", output);

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

        currentChat.setSystem(nluSystemPrompt.getText());
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
