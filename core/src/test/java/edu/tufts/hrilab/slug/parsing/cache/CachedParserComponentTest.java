/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.slug.parsing.cache;

import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import static org.junit.Assert.assertTrue;

public class CachedParserComponentTest {
    private TLDLParserComponent tldl;
    private CachedParserComponent component;
    private static Logger log = LoggerFactory.getLogger(edu.tufts.hrilab.slug.parsing.cache.CachedParserComponentTest.class);

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info(description.getMethodName());
        }
    };


    public CachedParserComponentTest() {
        tldl = DiarcComponent.createInstance(TLDLParserComponent.class, "-noUpdateAddressee");
        component = DiarcComponent.createInstance(CachedParserComponent.class, "-name cached-parser-test");
    }

    private boolean testUtterance(String words, String desired, String speaker, String addressee) {
        Utterance incoming = new Utterance.Builder()
                .setSpeaker(Factory.createSymbol(speaker))
                .setAddressee(Factory.createSymbol(addressee))
                .setWords(words)
                .setUtteranceType(UtteranceType.UNKNOWN)
                .setIsInputUtterance(true).build();
        Utterance parseResult = tldl.parseUtterance(incoming);
        component.addEntryToCache(parseResult);
        Utterance cacheResult = component.parseUtterance(incoming);

        // the expected strings don't have the words in them, toString does
        String outcome = cacheResult.toString().replace(",\"" + cacheResult.getWordsAsString() + "\"", "");
        if (outcome.equals(desired)) {
            log.info("outcome: " + outcome);
            log.info("desired: " + desired + "\n");
        } else {
            log.error("outcome: " + outcome);
            log.error("desired: " + desired + "\n");
        }
        return outcome.equals(desired);
    }

    @Test
    public void spaceStationTest() {
        log.info("spaceStationTest");
        tldl.addDictionary("pr2Unity.dict");
        //tldl.setUpdateAddressHistory(false);

        assertTrue(testUtterance(
            "go to beta",
            "INSTRUCT(server,unknown,goToArea(unknown,beta),{})",
            "server",
            "unknown"
        ));
        return;
        /*
        assertTrue(testUtterance(
            "robot1 go to beta",
            "INSTRUCT(server,unknown,goToArea(robot1,beta),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot1 go to beta left four",
            "INSTRUCT(server,unknown,goToTube(robot1,beta,leftFour),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot one go to beta left four",
            "INSTRUCT(server,unknown,goToTube(robot1,beta,leftFour),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "repair",
            "INSTRUCT(server,unknown,repairTube(unknown),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot two repair",
            "INSTRUCT(server,unknown,repairTube(robot2),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "repair alpha left one",
            "INSTRUCT(server,unknown,repairTube(unknown,alpha,leftOne),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot two repair beta left four",
            "INSTRUCT(server,unknown,repairTube(robot2,beta,leftFour),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "fix it",
            "INSTRUCT(server,unknown,repairTube(unknown),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot two fix",
            "INSTRUCT(server,unknown,repairTube(robot2),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "fix alpha right three",
            "INSTRUCT(server,unknown,repairTube(unknown,alpha,rightThree),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot one fix alpha right two",
            "INSTRUCT(server,unknown,repairTube(robot1,alpha,rightTwo),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "check alpha",
            "INSTRUCT(server,unknown,check(unknown,alpha),{})",
            "server",
            "unknown"
        ));

        assertTrue(testUtterance(
            "robot 1 check beta",
            "INSTRUCT(server,unknown,check(robot1,beta),{})",
            "server",
            "unknown"
        ));
        */
    }
}