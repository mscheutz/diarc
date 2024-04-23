/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.nao.MockNaoInterface;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class MockTwoNaoDemo extends DiarcConfiguration {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(MockTwoNaoDemo.class);

  // for running integration tests
  static public SimSpeechRecognitionComponent trustedSpeechRec;
  static public SimSpeechRecognitionComponent untrustedSpeechRec;
  static public MockNaoInterface dempster;
  static public MockNaoInterface shafer;

  // start the configuration
  @Override
  public void runConfiguration() {

    trustedSpeechRec = createInstance(SimSpeechRecognitionComponent.class,
            "-config demodialogues/heteroAgentsDemo_trusted.simspeech -speaker evan -listener dempster -nogui");
    untrustedSpeechRec = createInstance(SimSpeechRecognitionComponent.class,
            "-config demodialogues/heteroAgentsDemo_untrusted.simspeech -speaker ravenna -listener dempster -nogui");

    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

    createInstance(TLDLParserComponent.class, "-dict templatedict.dict templatedictLearned.dict");

    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

    createInstance(ReferenceResolutionComponent.class);

    createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

    createInstance(SimpleNLGComponent.class);

    dempster = createInstance(MockNaoComponent.class, "-groups agent:dempster");
    shafer = createInstance(MockNaoComponent.class, "-groups agent:shafer");

    createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class, "-beliefinitfile demos.pl agents/twonaoagents.pl " +
            "-asl nao/dance.asl core.asl vision.asl nao/naodemo.asl dialogue/nlg.asl dialogue/handleSemantics.asl dialogue/nlu.asl " +
            "-goal listen(self)");
  }

  public static void main(String[] args) {
    MockTwoNaoDemo demoConfig = new MockTwoNaoDemo();
    demoConfig.runConfiguration();
  }
}
