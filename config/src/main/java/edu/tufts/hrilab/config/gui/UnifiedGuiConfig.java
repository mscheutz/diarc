package edu.tufts.hrilab.config.gui;

import edu.tufts.hrilab.action.GoalManagerEndpointComponent;
import edu.tufts.hrilab.action.GoalViewerEndpointComponent;
import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.belief.gui.BeliefEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.GuiManager;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
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

        createInstance(BeliefEndpointComponent.class);
        createInstance(ChatEndpointComponent.class, "-r dempster shafer");
        createInstance(GoalViewerEndpointComponent.class);
        createInstance(GoalManagerEndpointComponent.class);

        createInstance(MockMoveBaseComponent.class, "-groups agent:fetch:fetch -simExecTime");

        createInstance(MapComponent.class, "-map_folder /home/lucien/Documents/diarc/elevator_lab_test -start_floor 1");

        createInstance(GuiManager.class);
    }

    public static void main(String[] args) {
        UnifiedGuiConfig config = new UnifiedGuiConfig();
        config.runConfiguration();
        SpringApplication.run(GuiManager.class, args);
    }
}
