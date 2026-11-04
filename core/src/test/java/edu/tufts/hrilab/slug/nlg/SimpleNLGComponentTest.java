/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg;
import java.util.ArrayList;

import static org.junit.Assert.*;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.Test;

public class SimpleNLGComponentTest {
  private SimpleNLGComponent component;
  private static Logger log = LoggerFactory.getLogger(edu.tufts.hrilab.slug.nlg.SimpleNLGComponentTest.class);

  public SimpleNLGComponentTest() {
    component = DiarcComponent.createInstance(SimpleNLGComponent.class, "");
    log.debug("SimpleNLGComponentTest constructor");
  }

  private boolean testUtterance(String utterance, String desired) {
    // String[] words = utterance.split(" ");
    // ArrayList<String> complete = new ArrayList<>(Arrays.asList(words));
    log.info("Testing utterance");
    // component.addWords(complete);

    Utterance u = UtteranceUtil.createUtterance(utterance);
    log.info("Utterance formed: " + u);

    component.convertSemanticsToText(u);

    String outcome = component.lastTranslation;
    log.info("outcome: " + outcome);
    log.info("desired: " + desired + "\n");

    return outcome.equals(desired);
  }

    private boolean testUtterances(ArrayList<String> utterances, ArrayList<String> desired) {
    // String[] words = utterance.split(" ");
    // ArrayList<String> complete = new ArrayList<>(Arrays.asList(words));

    if (utterances.size() != desired.size()) {
      log.error("input and desired semantics lists aren't the same length");
      return false;
    }

    for (int i = 0; i < utterances.size(); i++) {
      if (!testUtterance(utterances.get(i), desired.get(i))) return false;
    }

    return true;
  }

  @Test
  public void sherpaTemplate() {

    ArrayList<String> text = new ArrayList<>();
    ArrayList<String> semantics = new ArrayList<>();

    //type;speaker;listener;semantics;abverb
    //semantics.add("STATEMENT;brad;self;ahead(X,obstacle);{}");
    //text.add("X aheads obstacle.");

    semantics.add("STATEMENT(self,brad,see(self,brad),{})");
    text.add("I see you");

    semantics.add("STATEMENT(self,brad,not(see(self,evan)),{})");
    text.add("I do not see evan");

    semantics.add("STATEMENT(self,brad,greeting(brad,hello),{})");
    text.add("hello");

    semantics.add("STATEMENT(self,brad,take(did(self,walk(forward)),longest),{})");
    text.add("walking forward takes longest");

    semantics.add("STATEMENT(self,brad,is(did(self,move(forward)),pre(mostLikely,infinitive(fail()))),{})");
    text.add("moving forward is most Likely to fail");

    semantics.add("STATEMENT(self,brad,probabilityOf(if(is(obstacle,solid),did(self,move(forward))),0.2),{})");
    text.add("the Probability that I move forward if obstacle is solid is 0.2");

    semantics.add("STATEMENT(self,brad,probabilityOf(if(did(self,pickUp(knife)),did(self,move(forward))),0.1),{})");
    text.add("the Probability that I move forward if I pick Up knife is 0.1");

    semantics.add("STATEMENT(self,brad,is(did(self,move(forward)),if(did(self,pickUp(knife)),pre(mostLikely,infinitive(fail())))),{})");
    text.add("moving forward is most Likely to fail if I pick Up knife");

    log.info("Added tests");

    assertTrue(testUtterances(semantics, text));

  }
}