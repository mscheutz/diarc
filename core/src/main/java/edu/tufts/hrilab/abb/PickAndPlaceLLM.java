package edu.tufts.hrilab.abb;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.NLGInterface;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.NLG;
import edu.tufts.hrilab.slug.parsing.llm.*;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PickAndPlaceLLM extends DiarcComponent implements NLGInterface {
    static private Logger log = LoggerFactory.getLogger(PickAndPlaceLLM.class);
    NLG nlg = new NLG();
    Prompt nluPrompt = Prompts.getPrompt("pickAndPlace/nlu/pickAndPlaceActionSemanticTranslationAssista");
    boolean fromSemantics = false;
    Prompt wordsNLGPrompt = Prompts.getPrompt("pickAndPlace/nlg/pickAndPlaceNLGTranslation");;
    Prompt semanticsNLGPrompt = Prompts.getPrompt("pickAndPlace/nlg/pickAndPlaceSemanticsNLGTranslation");
    String outputLanguage = "en";

    public PickAndPlaceLLM() {
        super();
    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("nluPrompt").numberOfArgs(1).desc("filepath of NLU prompt text").build());
        options.add(Option.builder("wordsNLGPrompt").numberOfArgs(1).desc("filepath of NLG from words prompt text").build());
        options.add(Option.builder("semanticsNLGPrompt").numberOfArgs(1).desc("filepath to NLG from semantics prompt text").build());
        options.add(Option.builder("outputLanguage").numberOfArgs(1).desc("BCP-47 Code defining output language of utterances from LLM NLG prompt").build());
        options.add(Option.builder("fromSemantics").desc("use semanticsNLGPrompt with the LLM to perform NLG straight from semantics. Otherwise uses NLG to get a words string first").build());
        return options;
    }

    @Override
    public void parseArgs(CommandLine cmdLine) {
        if (cmdLine.hasOption("nluPrompt")) {
            nluPrompt = Prompts.getPrompt(cmdLine.getOptionValue("nluPrompt"));
        }
        if (cmdLine.hasOption("semanticsNLGPrompt")) {
            semanticsNLGPrompt = Prompts.getPrompt(cmdLine.getOptionValue("semanticsNLGPrompt"));
        }
        if (cmdLine.hasOption("wordsNLGPrompt")) {
            wordsNLGPrompt = Prompts.getPrompt(cmdLine.getOptionValue("wordsNLGPrompt"));
        }
        if (cmdLine.hasOption("outputLanguage")) {
            outputLanguage = cmdLine.getOptionValue("outputLanguage");
        }
        if (cmdLine.hasOption("fromSemantics")) {
            fromSemantics = true;
        }
    }

    @TRADEService
    public ParserResponse pickAndPlaceLLMParser (String utterance) {
        String userMessage = nluPrompt.getText() + utterance;
        List<Message> chatInput = new ArrayList<>();
        chatInput.add(new Message("user",userMessage));
        Chat chat = new Chat();
        chat.setMessages(chatInput);

        Completion response;
        try {
            response = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
        } catch (TRADEException e) {
            log.error("Error performing chat completion", e);
            return null;
        }

        String responseString = response.toString();
        Pattern responsePattern = Pattern.compile(".*Step two:.*\\s(.*)\\((.*?)\\)", Pattern.DOTALL);
        Matcher m = responsePattern.matcher(responseString);
        if (!m.matches()) {
            responsePattern = Pattern.compile(".*\\s(.*)\\((.*?)\\)", Pattern.DOTALL);
            m = responsePattern.matcher(responseString);
            if (!m.matches()) {
                log.error("[pickAndPlaceLLMParser] LLM response malformed, cannot parse");
                return null;
            }
        }
        String name = m.group(m.groupCount()-1);
        String argString= m.group(m.groupCount());
        argString = argString.replace("\s","");
        String[] args = argString.split(",");

        //Craft ParserResponse from result string
        ParserResponse parserResponse = new ParserResponse();
        String[] propositionArgumentArray = new String[args.length];
        List<Referent> referentList = new ArrayList<>();
        List<Descriptor> descriptorList = new ArrayList<>();
        String utteranceType = "";
        int startIndex = 0;
        //TODO: Get utterance type from LLM through use of chat history
        //      This is domain specific and technically not even specific enough
        //If first argument is not an actor, then we are replying to a question from the system
        if (args.length > 0) {
            String[] arg0Parts = args[0].split(":");
            if (arg0Parts.length > 1) {
                String type = arg0Parts[1];
                if (type.equals("agent") || type.equals("yumi")) {
                    utteranceType = "INSTRUCT";
                    propositionArgumentArray[0] = args[0]; //actor
                    startIndex += 1;
                }
            }
        }
        if (startIndex == 0) {
            utteranceType = "REPLY";
        }

        for (int i = startIndex; i < args.length; i++) {
            String arg = args[i];
            String[] argparts = arg.split(":");
            boolean typed = argparts.length > 1;
            if (typed) {
                String varName = "VAR" + i;
                String argName = argparts[0];
                String argType = argparts[1];

                Referent referent = new Referent();
                referent.text = argName;
                referent.type = argType;
                referent.role = "central"; //?
                referent.variable_name = varName;
                referent.cognitive_status = "DEFINITE";
                referentList.add(referent);

                Descriptor descriptor = new Descriptor();
                descriptor.text = argName;
                descriptor.arguments = new String[]{varName};
                descriptorList.add(descriptor);

                propositionArgumentArray[i] = varName;
            } else {
                propositionArgumentArray[i] = arg;
            }
        }
        Descriptor[] descriptors = descriptorList.toArray(new Descriptor[0]);
        Referent[] referents = referentList.toArray(new Referent[0]);

        parserResponse.referents = referents;
        parserResponse.intention = new Intention();
        parserResponse.intention.intent = utteranceType;
        parserResponse.intention.proposition = new Proposition();
        parserResponse.intention.proposition.text = name;
        //parserResponse.intention.proposition.type = "action";
        parserResponse.intention.proposition.arguments = propositionArgumentArray;
        parserResponse.descriptors = descriptors;

        return parserResponse;
    }

    @Override
    public Utterance convertSemanticsToText(Utterance u) {
        return convertSemanticsToText(u,false);
    }

    @TRADEService
    @Action
    public Utterance convertSemanticsToText(Utterance u, boolean hasWords) {
        String realization;
        Prompt nlgPrompt;
        if (hasWords) {
            realization = u.getWordsAsString();
            nlgPrompt = wordsNLGPrompt;
        } else {
            if (fromSemantics) {
                nlgPrompt = semanticsNLGPrompt;
                realization = u.getWordsAsString();
            } else {
                realization = nlg.translate(u);
                nlgPrompt = wordsNLGPrompt;
            }
        }

        String userMessage = nlgPrompt.getText() + realization;
        List<Message> chatInput = new ArrayList<>();
        chatInput.add(new Message("user",userMessage));
        Chat chat = new Chat();
        chat.setMessages(chatInput);

        Completion response;
        try {
            response = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
        } catch (TRADEException e) {
            log.error("Error performing chat completion", e);
            return null;
        }

        String responseString = response.toString();
        Pattern responsePattern = Pattern.compile(".*Step two:(.*)", Pattern.DOTALL);
        Matcher m = responsePattern.matcher(responseString);
        if (!m.matches()) {
            log.error("[pickAndPlaceLLMParser] LLM response malformed, cannot parse");
            u.setWords(realization);
        } else {
            u.setWords(m.group(m.groupCount()).trim());
            u.setLanguage(outputLanguage);
        }

        return u;
    }

    //To prevent TRADEExceptions when not running TLDL. May need to remove if using HybridParser
    @TRADEService
    public void injectDictionaryEntry(String morpheme, String type, String semantics, String cognitiveStatus) {
    }
}
