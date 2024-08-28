/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import edu.tufts.hrilab.config.SortingDemoMock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
public class SortingDemoTest {

    protected static Logger log = LoggerFactory.getLogger(OnRobotScrewingActionModificationTest.class);
    private SortingDemoMock config;
    private SimSpeechRecognitionComponent speechInputBrad;

    @Before
    public void initializeConfig() {
        //instantiate DIARCInstance (in mock)
        config = new SortingDemoMock();
        config.runConfiguration();
        speechInputBrad = config.simspeech;
    }

    @After
    public void shutdownDiarc() {
        log.debug("[shutdownDiarc] started");
        config.shutdownConfiguration();
        log.debug("[shutdownDiarc] completed");
    }

    @Test
    public void sortingTest() {

    }
}
