/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.nao;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.nao.NaoComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.asr.sphinx4.Sphinx4Component;

public class TwoNaoDemo extends DiarcConfiguration {
  /**
   * Set to true to use gui for speech input
   */
  public boolean simSpeech = true;
  /**
   * Set
   */
  public boolean mockNao = true;
  public boolean useSphinx = false;

  // start the configuration
  @Override
  public void runConfiguration() {

    if (simSpeech) {
      createInstance(SimSpeechRecognitionComponent.class,
              "-config demodialogues/heteroAgentsDemo_trusted.simspeech -speaker evan -addressee dempster");
      createInstance(SimSpeechRecognitionComponent.class,
              "-config demodialogues/heteroAgentsDemo_untrusted.simspeech -speaker ravenna -addressee dempster");
    }

    if (useSphinx) {
      createInstance(Sphinx4Component.class, "-mixer 0 -grammar heterogeneousAgents -nlp -speaker ravenna -listener shafer");
      createInstance(Sphinx4Component.class, "-mixer 2 -grammar heterogeneousAgents -nlp -speaker evan -listener dempster");
    }

    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

    createInstance(TLDLParserComponent.class, "-dict templatedict.dict templatedictLearned.dict");

    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

    createInstance(ReferenceResolutionComponent.class);

    createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

    createInstance(SimpleNLGComponent.class);

    if (mockNao) {
      createInstance(MockNaoComponent.class, "-groups agent:dempster -obstacle true"); // sees obstacle
      createInstance(MockNaoComponent.class, "-groups agent:shafer -floorSupport false"); // does not see floor support
    } else {
      createInstance(NaoComponent.class, "-groups agent:dempster -url 192.168.1.7 -unsafe -doNotWakeUp -voice low");
//      createInstance(edu.tufts.hrilab.nao.NaoComponent.class, "-groups agent:shafer -url 192.168.1.15 -unsafe -doNotWakeUp -voice high");
    }

    String gmArgs = "-beliefinitfile demos.pl agents/twonaoagents.pl " +
            "-asl core.asl vision.asl nao/naodemo.asl dialogue/nlg.asl dialogue/handleSemantics.asl dialogue/nlu.asl " +
            "-goal listen(self)";

    createInstance(GoalManagerComponent.class, gmArgs);
  }

}
