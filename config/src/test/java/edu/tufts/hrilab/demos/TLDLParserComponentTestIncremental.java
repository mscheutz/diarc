/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;


import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.*;
import java.util.Collection;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.belief.BeliefComponent;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.parsing.tldl.Node;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.interfaces.ASRInterface;
import org.junit.Rule;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.util.resource.Resources;





import static org.junit.Assert.assertTrue;

public class TLDLParserComponentTestIncremental {

    protected Logger log = LoggerFactory.getLogger(this.getClass());
    private TLDLParserComponent component;
    private BeliefComponent beliefComponent;
    private LLMComponent llmcomponent;




    //private static Logger log = LoggerFactory.getLogger(TLDLParserComponentTestIncremental.class);

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info(description.getMethodName());
        }
    };

    public TLDLParserComponentTestIncremental() {
        component = DiarcComponent.createInstance(TLDLParserComponent.class, "");
        beliefComponent = DiarcComponent.createInstance(BeliefComponent.class, "-initfile demos.pl -initfile agents/agents.pl");
        llmcomponent = DiarcComponent.createInstance(edu.tufts.hrilab.llm.LLMComponent.class, "");

        if (llmcomponent == null) {
            throw new IllegalStateException("LLMComponent could not be created.");
        }

        try {
            boolean keyPresent = TRADE.getAvailableService(
                    new TRADEServiceConstraints().name("hasOpenAIKey")
            ).call(Boolean.class);
            log.info("✅ OpenAI key present? " + keyPresent);
        } catch (TRADEException e) {
            log.error("Failed to retrieve OpenAI key presence via TRADE", e);
        }

        try {
            String result = TRADE.getAvailableService(
                    new TRADEServiceConstraints().name("pingLLM")
            ).call(String.class);
            log.info("Ping test: " + result);
        } catch (TRADEException e) {
            log.error("Ping failed", e);
        }

        Completion response = null;

        try {
            response = TRADE.getAvailableService(
                    new TRADEServiceConstraints().name("chatCompletion").argTypes(String.class)
            ).call(Completion.class, "print a random word!");
            log.info("LLM response: {}", response.getText());
        } catch (TRADEException e) {
            log.error("OpenAI test failed", e);
        }




    }


    @Test
    public void phraseByPhraseParsingTest() {
        log.info("Testing phrase-by-phrase incremental parsing for 'grab the mug' and 'by the handle'");

        // Add dictionary containing necessary rules
        component.addDictionary("incrementalTesting.dict");

        // Define the input phrases
        //String[] phrases = {"put the apple", "on the towel", "on the box"};
        String[] phrases = {"put the apple", "on the towel", "in the box", "that is red"};

        // Set up speaker and listener
        Symbol speaker = Factory.createSymbol("speaker");
        Symbol listener = Factory.createSymbol("listener");

        // Parse each phrase incrementally
        for (String phrase : phrases) {
            Utterance partialUtterance = new Utterance.Builder()
                    .setWords(Arrays.asList(phrase.split(" "))) // ✅ Splits full phrase into words
                    .setSpeaker(speaker)
                    .setAddressee(listener)
                    .build();

            Utterance result = component.parseUtterance(partialUtterance);
            log.info("✅ Result after parsing phrase '{}': {}", phrase, result);

            // Verify that the chart is being updated incrementally
            assertTrue("Chart should not be empty after parsing a phrase.", component.isChartInitialized());
        }

        // Add LLM component example here
        // get prompt to interact with incremental parser




        //Prompt IncrementalPrompt = Prompts.getPrompt("incrementalParsing");
        //Chat chat = new Chat(IncrementalPrompt);
        StringBuilder promptString= new StringBuilder();
        promptString.append("generate an AMR for the utterance: \"");
        //promptString.append(humanInput);
        promptString.append("\".");


        log.debug("AMR Prompt:");
        log.debug(promptString.toString());

        String result="";
        try{
            Completion completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, promptString.toString());

            String simplifiedUtterance="";
            if(completion != null) {
                result=completion.getText();
            }
        } catch (TRADEException e) {
            log.error("[generate]",e);
        }





        //chat.setMessages(chatInput);







        // ✅ After parsing all phrases, check final chart state
        int finalChartSize = component.getCurrentChartSize();
        log.info("📝 Final chart size before reset: {}", finalChartSize);

        // ✅ Reset the parser state
        log.info("🔄 Resetting parser...");
        component.resetChart();

        // ✅ Perform final parse on full sentence after reset
        Utterance finalUtterance = new Utterance.Builder()
                .setWords(Arrays.asList("grab", "the", "mug", "by", "the", "handle")) // ✅ Dynamically built from phrases
                .setSpeaker(speaker)
                .setAddressee(listener)
                .build();

        Utterance finalResult = component.parseUtterance(finalUtterance);
        log.info("🚀 Final result after reset: {}", finalResult);
        // TODO add LLM check here for an example

        assertTrue("Final parse should produce a valid utterance.", finalResult != null);
    }

    // Helper function to parse incrementally
    public void incrementalParse(String text) {
        log.info("Incrementally parsing: {}", text);

        Utterance partialUtterance = new Utterance.Builder()
                .setWords(Arrays.asList(text.split(" "))) // Split words dynamically
                .setSpeaker(Factory.createSymbol("speaker"))
                .setAddressee(Factory.createSymbol("listener"))
                .build();

        Utterance result = component.parseUtterance(partialUtterance);
        log.info("Result after incremental parsing '{}': {}", text, result);
    }



}
