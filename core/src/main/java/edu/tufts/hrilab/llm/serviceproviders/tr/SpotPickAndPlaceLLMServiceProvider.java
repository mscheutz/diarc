/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders.tr;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//TODO: the listed prompts don't even exist on this branch. 1. where are they or 2. can we get rid of this class?
public class SpotPickAndPlaceLLMServiceProvider extends TRLLMServiceProvider {
    @Override
    protected void init() {
        super.init();
        processMatchingNLUParseLiteralsAsRefs = true;
    }

    @Override
    protected Prompt getNluSystemPrompt() {
        return Prompts.getPrompt("spotpickandplace/nlu/system");
    }

    @Override
    protected Prompt getNluUserPrompt() {
        return Prompts.getPrompt("spotpickandplace/nlu/user");
    }

    @Override
    protected Prompt getNlgSystemPrompt() {
        return Prompts.getPrompt("spotpickandplace/nlg/system");
    }

    @Override
    protected Prompt getNlgUserPrompt() {
        return Prompts.getPrompt("spotpickandplace/nlg/user");
    }

    @Override
    protected Prompt getValidationSystemPrompt() {
        return getNlgSystemPrompt();
    }

    @Override
    protected Prompt getValidationUserPrompt() {
        return Prompts.getPrompt("spotpickandplace/nlg/validation");
    }

    @Override
    protected Map<String, String> populatePromptVars(Utterance utterance, Map<String, String> promptVars, boolean system, boolean parsing, boolean validation) {
        if (parsing) {
            if (!system) {
                //TODO: if this prompt file is ever recovered, need to add %utterance variable to the end of it
                promptVars.put("%utterance", utterance.getWordsAsString());
            }
        } else {
            if (!system) {
                promptVars.put("%utterance", utterance.toString());
            }
        }
        return promptVars;
    }

    @Override
    protected List<String> getRefTypes() {
        return List.of("location", "physobj");
    }

    @Override
    protected Utterance correctCommonNLUMistakes(Utterance utterance) {
        return utterance;
    }

    @Override
    protected String getDescriptorType(String agentName, String descriptor) {
        return "";
    }

    //TODO: update to be in line with base class format
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

        return output.build();
    }

    @Override
    public Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse) {
        log.info(llmResponse.getText());
        String responseString = llmResponse.getText().trim();
        outgoingUtterance.setWords(List.of(responseString.split(" ")));

        return outgoingUtterance;
    }

    //public abstract Set<Symbol> getRelevantAgents();
    //public abstract Set<Symbol> getRelevantTypes();
    //public abstract Set<Symbol> getRelevantActivatedEntities();
    //public abstract Set<Predicate> getRelevant
    //public abstract String getRoleDescription();
    //public abstract String getTaskDescription();

}
