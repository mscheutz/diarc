package edu.tufts.hrilab.config;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.util.resource.Resources;

import java.util.ArrayList;

public class IHMCConfig extends DiarcConfiguration {

  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(TLDLParserComponent.class, "" +
//            "-dict templatedict.dict " +
            "-dict mfawn/ihmc.dict"
    );
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class, "");
    createInstance(SimpleNLGComponent.class, "");
    createInstance(SimSpeechProductionComponent.class, "");
//

    createInstance(GoalManagerComponent.class,
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                    "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl demos/ihmc.asl "
                    + "-goal listen(self:agent) "
                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
    );

    createInstance(SimSpeechRecognitionComponent.class, "-speaker marlow -config ihmc.simspeech");

    String kbName = "location";
    PoseConsultant consultant = new PoseConsultant(PoseReference.class, kbName, new ArrayList<>());
    try {
      TRADE.registerAllServices(consultant, kbName);
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }

    String refsConfigDir = "config/edu/tufts/hrilab/map";
    String refsConfigFile = "refs/ihmcRefs.json";
    String filename = Resources.createFilepath(refsConfigDir, refsConfigFile);
    consultant.loadReferencesFromFile(filename);

    String file = "ihmc.ihmc_wrapper";
    TRADEServiceConstraints c = new TRADEServiceConstraints().inGroups();

    PythonWrapper wrapper = new PythonWrapper(file);
    wrapper.start();

  }
}
