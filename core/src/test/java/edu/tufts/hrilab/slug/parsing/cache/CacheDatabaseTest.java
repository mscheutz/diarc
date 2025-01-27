/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.slug.parsing.cache;

import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.cache.CacheDatabase;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import static org.junit.Assert.assertTrue;

public class CacheDatabaseTest {
  private static Logger log = LoggerFactory.getLogger(edu.tufts.hrilab.slug.parsing.cache.CacheDatabaseTest.class);
  private CacheDatabase database;

  @Rule
  public TestRule watcher = new TestWatcher() {
    protected void starting(Description description) {
      log.info(description.getMethodName());
    }
  };

  public CacheDatabaseTest() {
    //
  }

  private boolean InsertTest (String speaker, String listener, String utterance) {
    Utterance incoming = new Utterance(Factory.createSymbol(speaker),
        Factory.createSymbol(listener),
        Arrays.asList(utterance.split(" ")),
        UtteranceType.UNKNOWN,
        true);
    database.add(incoming);
    Utterance retrieved = database.get(incoming);
    if (retrieved != null && Utilities.equalsIgnoreType(incoming.getSpeaker(), retrieved.getSpeaker())) {
      log.info("Utterance was stored and retrieved for key \"" + incoming.getWordsAsString() + "\"");
    }
    return false;
  }

  @Test
  public void tableTest() {
    database = new CacheDatabase ("cache-database-test", null, false, false, "English");
    assertTrue(database != null);
    log.info("Database exists after construction");
    //assertTrue(InsertTest("matt", "ava", "go home"));
  }

}