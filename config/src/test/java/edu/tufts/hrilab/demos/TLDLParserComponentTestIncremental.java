/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;


import java.util.Arrays;
import java.util.List;
import java.util.Collection;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import edu.tufts.hrilab.diarc.DiarcComponent;
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






import static org.junit.Assert.assertTrue;

public class TLDLParserComponentTestIncremental {
    protected Logger log = LoggerFactory.getLogger(this.getClass());
    private TLDLParserComponent component;
    //private static Logger log = LoggerFactory.getLogger(TLDLParserComponentTestIncremental.class);

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info(description.getMethodName());
        }
    };

    public TLDLParserComponentTestIncremental() {
        component = DiarcComponent.createInstance(TLDLParserComponent.class, "");
    }

    @Test
    public void parsingTest() {
        log.info("Testing incremental parsing with real-time ASR input");

        component.addDictionary("incrementalTesting.dict");


        // Initialize Python ASR wrapper
        String file = "examples.minimal_example"; // Specify the python file you want to run, according to its package path
        PythonWrapper wrapper = new PythonWrapper(file, true); // Declare the wrapper as a TRADE spoke
        wrapper.start(); // Start the wrapper

        // Wait for PyTRADE to start
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Call the 'dock' TRADE service with an empty symbol as an argument
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("dock")).call(void.class, Factory.createSymbol(""));
        } catch (TRADEException e) {
            throw new RuntimeException(e);
        }



        // Get available TRADE services
        Collection<TRADEServiceInfo> myParsingServices = TRADE.getAvailableServices(new TRADEServiceConstraints());

        // Identify the ASR service and fetch incremental words
        String recognizedText = "";
        for (TRADEServiceInfo service : myParsingServices) {
            if (service.serviceString.equals("getLastRecognizedText()")) {
                try {
                    recognizedText = service.call(String.class, ""); // Fetch ASR output
                    log.info("Received from ASR: {}", recognizedText);
                } catch (TRADEException te) {
                    log.error("Error retrieving recognized text from ASR", te);
                }
            }
        }

        // Ensure we received some text from ASR
        if (recognizedText.isEmpty()) {
            log.warn("No recognized text received from ASR.");
            return;
        }

        // Split text incrementally and send to Java parser
        String[] words = recognizedText.split(" ");
        StringBuilder incrementalPhrase = new StringBuilder();

        for (String word : words) {
            incrementalPhrase.append(word).append(" "); // Append new word incrementally
            String partialText = incrementalPhrase.toString().trim();

            // Call incremental parsing function
            incrementalParse(partialText);
        }

        // Reset parser state before final utterance
        component.resetChart();

        // Parse a final reference phrase to test full sentence parsing
        Utterance finalUtterance = new Utterance.Builder()
                .setWords(Arrays.asList("grab", "the", "mug", "by", "the", "top"))
                .setSpeaker(Factory.createSymbol("speaker"))
                .setAddressee(Factory.createSymbol("listener"))
                .build();

        Utterance finalResult = component.parseUtterance(finalUtterance);
        log.info("Final result after reset: {}", finalResult);
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

    @Test
    public void wordByWordParsingTest() {
        log.info("Testing word-by-word incremental parsing for 'grab the mug on the table'");

        // Add a dictionary containing necessary rules
        component.addDictionary("incrementalTesting.dict");

        // Define the input utterance broken into words
        String[] words = {"grab", "the", "mug", "by", "the", "top"};

        // Set up speaker and listener
        Symbol speaker = Factory.createSymbol("speaker");
        Symbol listener = Factory.createSymbol("listener");

        // Parse each word incrementally
        for (String word : words) {
            Utterance partialUtterance = new Utterance.Builder()
                    .setWords(Arrays.asList(word))
                    .setSpeaker(speaker)
                    .setAddressee(listener)
                    .build();

            Utterance result = component.parseUtterance(partialUtterance);
            log.info("Result after parsing '{}': {}", word, result);

            // Verify that the chart is being updated and nodes are added incrementally
            assertTrue("Chart should not be empty after parsing a word.", component.isChartInitialized());
        }

        // After parsing all words, check the final state of the chart
        int finalChartSize = component.getCurrentChartSize();
        log.info("Final chart size: {}", finalChartSize);

        // Optionally, validate the final parses in the chart
        Utterance finalResult = component.buildPartialUtterance(new Utterance.Builder()
                .setWords(Arrays.asList(words))
                .setSpeaker(speaker)
                .setAddressee(listener)
                .build());
        log.info("Final result after parsing all words: {}", finalResult);

        assertTrue("Final parse should produce a valid utterance.", finalResult != null);
    }




}
