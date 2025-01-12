/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.parsing.tldl.Node;
import org.junit.Rule;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import static org.junit.Assert.assertTrue;

public class TLDLParserComponentTestIncremental {
    private TLDLParserComponent component;
    private static Logger log = LoggerFactory.getLogger(TLDLParserComponentTestIncremental.class);

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
        log.info("Testing incremental parse for 'grab the mug' followed by 'on the table'");
        component.addDictionary("incrementalTesting.dict");
        //component.setUseAddressHistory(false);

        // Simulate incremental input
        Utterance part1 = new Utterance.Builder()
                .setWords(Arrays.asList("grab", "the", "mug"))
                .setSpeaker(Factory.createSymbol("speaker"))
                .setAddressee(Factory.createSymbol("listener"))
                .build();

        Utterance part2 = new Utterance.Builder()
                .setWords(Arrays.asList("on", "the", "table"))
                .setSpeaker(Factory.createSymbol("speaker"))
                .setAddressee(Factory.createSymbol("listener"))
                .build();

        // Parse the first part
        Utterance phrase1 = component.parseUtterance(part1);
        log.info("Result of part 1 parsing: {}", phrase1);

        // Parse the second part incrementally and create full utterance that builds on first phrase (e.g. "grab the mug...on the table")
        Utterance phrase2 = component.parseUtterance(part2);
        log.info("Result of part 2 parsing: {}", phrase2);

        // Reset the chart
        component.resetChart();

        // Parse a new input after reset
        Utterance newInput = new Utterance.Builder()
                .setWords(Arrays.asList("grab", "the", "cup"))
                .setSpeaker(Factory.createSymbol("speaker"))
                .setAddressee(Factory.createSymbol("listener"))
                .build();

        Utterance newResult = component.parseUtterance(newInput);
        log.info("Result after reset: {}", newResult);


    }

    @Test
    public void wordByWordParsingTest() {
        log.info("Testing word-by-word incremental parsing for 'grab the mug on the table'");

        // Add a dictionary containing necessary rules
        component.addDictionary("incrementalTesting.dict");

        // Define the input utterance broken into words
        String[] words = {"grab", "the", "mug", "on", "the", "table"};

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
