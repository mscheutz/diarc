package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;

public class IHMCConfig2 extends DiarcConfiguration {

  @Override
  public void runConfiguration() {

//    createInstance(ListenerComponent.class);
//    createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict spot.dict");
//    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
//    createInstance(ReferenceResolutionComponent.class);
//    createInstance(DialogueComponent.class, "");
//    createInstance(SimpleNLGComponent.class, "");
//    createInstance(SimSpeechProductionComponent.class, "");
//

//    createInstance(GoalManagerComponent.class,
//            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
//                    "-beliefinitfile agents/agents.pl "
//                    + "-beliefinitfile demos.pl "
//                    + "-asl core.asl "
//                    + "-asl dialogue/nlg.asl "
//                    + "-asl dialogue/nlu.asl "
//                    + "-asl dialogue/handleSemantics.asl "
////                    + "-goal listen(self:agent) "
//                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
//    );

//    createInstance(SimSpeechRecognitionComponent.class, "-speaker pete -config spotDemo.simspeech");
  }
}
