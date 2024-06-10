package edu.tufts.hrilab.config.gui;

import edu.tufts.hrilab.action.GoalManagerEndpointComponent;
import edu.tufts.hrilab.action.GoalViewerEndpointComponent;
import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.DemoApplication;
import edu.tufts.hrilab.gui.EndpointManagerComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;

public class UnifiedGuiConfig extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(UnifiedGuiConfig.class);

    @Override
    public void runConfiguration() {
        // Two Nao demo + web endpoints
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
        createInstance(GoalViewerEndpointComponent.class);
        createInstance(GoalManagerEndpointComponent.class);
        createInstance(EndpointManagerComponent.class);

        // Map demo
        createInstance(MockMoveBaseComponent.class, "-groups agent:fetch:fetch -simExecTime");
    }

    public static void main(String[] args) {
        UnifiedGuiConfig config = new UnifiedGuiConfig();
        config.runConfiguration();
        SpringApplication.run(DemoApplication.class, args);
    }
}
