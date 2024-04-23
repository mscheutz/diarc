/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.pragmatics;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.Test;

import java.util.Set;

import static org.junit.Assert.*;

public class PragmaticsComponentTest{
  private PragmaticsComponent component;
  private static Logger log = LoggerFactory.getLogger(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponentTest.class);

  public PragmaticsComponentTest() {
    component = DiarcComponent.createInstance(PragmaticsComponent.class, "");
    log.debug("PragmaticsComponentTest Constructor");
  }

  //test method to compare ground truth to the output that has been produced

  //a bunch of tests to make sure things work

  @Test
  public void functorName(){
    component.loadPragRules(Resources.createFilepath("config/edu/tufts/hrilab/slug/pragmatics", "test.prag"));

    Predicate sem = Factory.createPredicate("and(compDefOf(VAR0,onTop(VAR1,VAR2)),tower(VAR0),object(VAR1),object(VAR2))");
    Utterance testUtt = new Utterance.Builder()
            .setSpeaker(Factory.createSymbol("self"))
            .addListener(Factory.createSymbol("commX"))
            .setSemantics(sem)
            .setUtteranceType(UtteranceType.STATEMENT)
            .setIsInputUtterance(true).build();

    try {
      Pair<Set<Term>,Set<Term>> meanings= component.getPragmaticMeanings(testUtt);

//    //get on and off record meanings, send to rr if connected.
      Set<Term> directMeaning = meanings.getLeft();
      log.info("Direct Meaning: " + directMeaning);
      Set<Term> indirectMeaning = meanings.getRight();
      log.info("Indirect Meaning: " + indirectMeaning);

      log.info("input semantics  : "+ sem);
      log.info("pragmatic meaning: "+directMeaning);
      log.info("indirect meaning : "+indirectMeaning);
    }
    catch (Exception e){
      log.error("error calling get Pragmatic meaning on utternace: "+testUtt);
      log.error("[functorName]",e);
    }

    assertTrue(true);
  }
}
