/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.nao.MockNaoInterface;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class MockTwoNaoDemo extends DiarcConfiguration {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(MockTwoNaoDemo.class);

  // for running integration tests
  static public SimSpeechRecognitionComponent trustedSpeechRec;
  static public SimSpeechRecognitionComponent untrustedSpeechRec;
  static public MockNaoInterface dempster;
  static public MockNaoInterface shafer;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("singleComponentPerContainer").desc("Set each component to launch in separate TRADE container.").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("singleComponentPerContainer")) {
      this.setSingleComponentPerContainer(true);
    }
  }

  // start the configuration
  @Override
  public void runConfiguration() {

    trustedSpeechRec = DiarcComponent.createInstance(SimSpeechRecognitionComponent.class,
            "-config demodialogues/heteroAgentsDemo_trusted.simspeech -speaker evan -addressee dempster -nogui");
    untrustedSpeechRec = DiarcComponent.createInstance(SimSpeechRecognitionComponent.class,
            "-config demodialogues/heteroAgentsDemo_untrusted.simspeech -speaker ravenna -addressee dempster -nogui");

    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

    createInstance(TLDLParserComponent.class, "-dict templatedict.dict templatedictLearned.dict");

    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

    createInstance(ReferenceResolutionComponent.class);

    createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

    createInstance(SimpleNLGComponent.class);

    dempster = DiarcComponent.createInstance(MockNaoComponent.class, "-groups agent:dempster");
    shafer = DiarcComponent.createInstance(MockNaoComponent.class, "-groups agent:shafer");

    createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl agents/twonaoagents.pl ",
            "-asl nao/dance.asl core.asl vision.asl nao/naodemo.asl dialogue/nlg.asl dialogue/handleSemantics.asl dialogue/nlu.asl ",
            "-goal listen(self)");
  }

}
