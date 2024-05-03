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
    private CachedParserComponent component;
    private static Logger log = LoggerFactory.getLogger(edu.tufts.hrilab.slug.parsing.cache.CachedParserComponentTest.class);

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info(description.getMethodName());
        }
    };


    public CachedParserComponentTest() {
        component = DiarcComponent.createInstance(CachedParserComponent.class, "");
    }

}