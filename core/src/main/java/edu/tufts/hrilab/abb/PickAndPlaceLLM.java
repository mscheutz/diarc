package edu.tufts.hrilab.abb;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.slug.parsing.llm.*;
import org.scalacheck.Gen;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PickAndPlaceLLM extends DiarcComponent {
    static private Logger log = LoggerFactory.getLogger(PickAndPlaceLLM.class);

    @TRADEService
    public ParserResponse pickAndPlaceLLMParser (String utterance) {
        Prompt prompt = Prompts.getPrompt("pickAndPlaceActionSemanticTranslation");
        String userMessage = prompt.getText() + utterance;
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
}
