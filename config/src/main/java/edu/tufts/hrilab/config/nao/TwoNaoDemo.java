/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.nao;
import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.action.GoalEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.DemoApplication;
import edu.tufts.hrilab.gui.EndpointManagerComponent;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;
import org.springframework.context.annotation.Configuration;

@Configuration
public class TwoNaoDemo extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(TwoNaoDemo.class);

  // start the configuration
  @Override
  public void runConfiguration() {
    createInstance(SimSpeechRecognitionComponent.class,
            "-config demodialogues/heteroAgentsDemo_trusted.simspeech -speaker evan -listener dempster");
    createInstance(SimSpeechRecognitionComponent.class,
            "-config demodialogues/heteroAgentsDemo_untrusted.simspeech -speaker ravenna -listener shafer");

    createInstance(ListenerComponent.class);
    createInstance(TLDLParserComponent.class, "-dict templatedict.dict templatedictLearned.dict");
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);
    createInstance(SimpleNLGComponent.class);

    createInstance(MockNaoComponent.class, "-groups agent:dempster -obstacle true"); // sees obstacle
    createInstance(MockNaoComponent.class, "-groups agent:shafer -floorSupport false"); // does not see floor support

    String gmArgs = "-beliefinitfile demos.pl agents/twonaoagents.pl " +
            "-asl core.asl vision.asl nao/naodemo.asl dialogue/nlg.asl dialogue/handleSemantics.asl dialogue/nlu.asl " +
            "-goal listen(self)";
    createInstance(GoalManagerImpl.class, gmArgs);

    createInstance(ChatEndpointComponent.class, "-n dempster shafer");
    createInstance(GoalEndpointComponent.class);
    createInstance(EndpointManagerComponent.class);
  }

  public static void main(String[] args) {
    TwoNaoDemo demoConfig = new TwoNaoDemo();
    demoConfig.runConfiguration();
    SpringApplication.run(DemoApplication.class, args);
  }
}
