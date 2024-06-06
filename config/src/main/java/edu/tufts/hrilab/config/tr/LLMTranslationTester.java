package edu.tufts.hrilab.config.tr;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.io.FileUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

public class LLMTranslationTester {

    protected static Logger log = LoggerFactory.getLogger(LLMTranslationTester.class);
    int numTranslations = 1;
    String language = "German";
    String service = "llamahf";//"t5hf";//"openai";
    String model = "Meta-Llama-3-70B-Instruct";//"Llama-2-70b-chat-hf";//"gpt-3.5-turbo";
    String inputDirectory = "/config/edu/tufts/hrilab/llm/inputs/";
    String inputFile = "pickAndPlaceAssistaJapanese";
    String outputFile = "/home/eric/code/diarc/core/src/main/resources/config/edu/tufts/hrilab/llm/outputs/%s/%s/%s_prompt_%s.txt";
    List<String> promptFiles = Arrays.asList("pickAndPlace/nlu/pickAndPlaceActionSemanticTranslationAssista");
    List<String> systemPromptFiles = Arrays.asList("");
    boolean performTranslationTask = true;
    boolean performSemanticsToTextTask = false; //old
    boolean performJointTask = false; //old

    public SimSpeechRecognitionComponent simspeech;
    public HybridParserComponent parser;
    public LLMComponent llmComponent;
    public LLMTranslationTester() {
        simspeech = DiarcComponent.createInstance(SimSpeechRecognitionComponent.class, "-speaker brad -config yumiFoodOrdering.simspeech");
        parser = DiarcComponent.createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, "-tldl foodOrdering.dict -tldl yumiFoodOrderingHomophones.dict -patternMatching");

        llmComponent= DiarcComponent.createInstance(LLMComponent.class, String.format("-service %s -model %s -temperature 0.6", service, model));
    }

    /**
     * Input:
     *      Single input file containing a list of utterances to translate
     *      Output filepath to write results (translations) to
     *      List of languages to test
     *      List of prompt format strings to test
     *      Number of times to translate each utterance (often results are not consistent)
     *  Each language will be tested alongside each prompt and have results written to correspondingly labeled fies
     */
    public void run() {
        //Translation task only
        if (performTranslationTask) {
            //for (String language : languages) {
            List<Prompt> prompts = new ArrayList<>();
            List<Prompt> systemPrompts = new ArrayList<>();
            for (int i=0;i<promptFiles.size();i++) {
                prompts.add(Prompts.getPrompt(promptFiles.get(i)));
                String systemPromptFileName = systemPromptFiles.get(i);
                if (systemPromptFileName.isEmpty()) {
                    systemPrompts.add(new Prompt(""));
                } else {
                    systemPrompts.add(Prompts.getPrompt(systemPromptFiles.get(i)));
                }
            }
            //}

            for (int i = 0; i < prompts.size(); i++) {
                log.info("[LLMTranslationTester] prompt: " + i);
                testUtterances(prompts.get(i), systemPrompts.get(i), String.format(outputFile, service, model, inputFile, promptFiles.get(i)),0);
            }
        }

        ////SemanticsToText task only
        //if (performSemanticsToTextTask) {
        //    NLGPrompt prompt = new NLGPrompt();
        //    llmnlComponent.setPerformTranslation(false);
        //    llmnlComponent.setPerformSemanticsToText(true);
        //    llmnlComponent.setNLGPrompt(prompt);
        //    testUtterances("test/resources/config/com/llm/inputs/semantics.txt", "test/resources/config/com/llm/outputs/textToSemanticsOutputs.txt",1);
        //}

        ////Both
        //if (performJointTask) {
        //    llmnlComponent.setPerformTranslation(true);
        //    llmnlComponent.setPerformSemanticsToText(true);
        //    llmnlComponent.setOutputLanguage("german");
        //    testUtterances("test/resources/config/com/llm/inputs/semantics.txt", String.format(outputFile, language, "_combined"),2);
        //}

        System.exit(0);
    }

    /**
     * Collect utterances to translate from input file
     */
    private static List<String> gatherUtterancesFromFile(String filepath) {
        InputStream stream = LLMComponent.class.getResourceAsStream(filepath);
        if (stream == null) {
            log.error("Resource not found: " + filepath);
            return null;
        }

        BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
        return reader.lines().collect(Collectors.toList());
    }

    /**
     * translate each line present in the input file using LLMNLComponent and write results to output file
     */
    private void testUtterances(Prompt prompt, Prompt systemPrompt, String outputFilepath, int task) {
        //Reason this isnt a stringbuilder?
        List<String> translatedUtterances = new ArrayList<>();
        List<String> sanitizedResults = new ArrayList<>();
        if (task == 0) {
            translatedUtterances.add("Translation prompt:\n");
        } else if (task == 1) {
            translatedUtterances.add("SemanticsToText prompt:\n");
        } else if (task == 2) {
            translatedUtterances.add("Joint prompt:\n");
        }
        translatedUtterances.add(prompt.getText()+"\n\n");

        for (String utterance : gatherUtterancesFromFile(inputDirectory + inputFile +".txt")) {
            log.info("[LLMTranslationTester] " + utterance);
            if (utterance.isEmpty() || utterance.startsWith("#")) {
                continue;
            }
            StringBuilder sb = new StringBuilder();
            sb.append(String.format("Input: %s\nOutputs:", utterance));
            for (int i=0;i<numTranslations;i++) {
                String result = translate(prompt,systemPrompt,utterance,language);
                sb.append(result);
                sb.append("\n");
//                Pattern semanticPattern = Pattern.compile(".*Step two:.*\\s(.*\\(.*\\))\\s*", Pattern.DOTALL);
                Pattern semanticPattern = Pattern.compile(".*Step two:(.*)", Pattern.DOTALL);
                Matcher m = semanticPattern.matcher(result);
                if (m.matches()) {
                    result = m.group(m.groupCount());
                    result = result.replace("\s","");
                } else {
                    semanticPattern = Pattern.compile(".*\\s(.*\\(.*\\))\\s*", Pattern.DOTALL);
                    m = semanticPattern.matcher(result);
                    if (m.matches()) {
                        result = m.group(m.groupCount());
                        result = result.replace("\s","");
                    }
                }
                sanitizedResults.add(result);
            }
            translatedUtterances.add(sb.toString());
        }
        translatedUtterances.add("\n\nSanitized Results:\n");
        translatedUtterances.addAll(sanitizedResults);

        writeGeneratedResults(translatedUtterances, outputFilepath);
    }

    /**
     * Write translated results to outputfile
     */
    private void writeGeneratedResults(List<String> results, String outputfilepath) {
        try {
            File file = new File(outputfilepath);
            if (!file.exists()) {
                file.getParentFile().mkdirs();
                file.createNewFile();
            }

            FileUtils.writeLines(file, results);
        } catch (Exception e) {
            log.error("error writing translated utterances to file", e);
        }
    }

    public String translate(Prompt prompt, Prompt systemPrompt, String input, String language) {
        long startTime = System.currentTimeMillis();
        Completion completion;
        String userMessage = prompt.getText() + input;
        String systemMessage = systemPrompt.getText();
        List<Message> chatInput = new ArrayList<>();
        chatInput.add(new Message("user",userMessage));
        log.debug("generated prompt: " + userMessage);
        LlamaHFChat chat = new LlamaHFChat();
        chat.setSystem(systemMessage);
        chat.setMessages(chatInput);
        try {
            if (service.equals("t5hf")) {
                completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(String.class)).call(Completion.class,userMessage);
            } else if (service.equals("llamahf")) {
                completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Symbol.class, Chat.class)).call(Completion.class, Factory.createSymbol(model), chat);
            } else if (service.equals("openai")) {
                completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(List.class)).call(Completion.class,chatInput);
            } else {
                log.error("[translate] unknown service {}", service);
                System.exit(1);
                return "";
            }
        } catch (TRADEException e) {
            log.error("[translateLanguageLLM] error calling chatCompletion", e);
            return input;
        }

        log.info("[translate] time spent: {}", (System.currentTimeMillis() - startTime)/1000f);
        return completion.getText();
    }

    public static void main(String[] args) {
        LLMTranslationTester tester = new LLMTranslationTester();
        tester.run();
    }
}