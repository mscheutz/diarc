/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fetch.MockFetchItComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.vision.MockVisionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FetchPerformanceAssessmentDemoMock extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(FetchPerformanceAssessmentDemoMock.class);

  static public SimSpeechRecognitionComponent simSpeechRec;
  static public GoalManagerImpl gm;
  static boolean showGUI = false;

  // start the configuration
  @Override
  public void runConfiguration() {
    String simRecArgs = "-config performanceAssessment.simspeech -speaker tyler -addressee andy:agent ";
    if (!showGUI) {
      simRecArgs += "-nogui";
    }
    simSpeechRec = createInstance(SimSpeechRecognitionComponent.class, simRecArgs);

    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

    createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict multiRobotCaddyGenerated.dict");

    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

    createInstance(ReferenceResolutionComponent.class);

    createInstance(DialogueComponent.class, "-groups agent:andy:agent");

    createInstance(SimpleNLGComponent.class);

    createInstance(SimSpeechProductionComponent.class);

    createInstance(MockVisionComponent.class, "-belief -refs refs/visionRefs.json -groups agent:andy:agent");

    createInstance(MockFetchItComponent.class);

    createInstance(MapComponent.class, "-refs refs/locationRefs.json -groups agent:andy:agent");

    createInstance(MockMoveBaseComponent.class);

    String gmArgs = "-beliefinitfile demos.pl "
            + "-beliefinitfile agents/agents.pl "
            + "-nomemorymanager "
            + "-asl core.asl "
            + "fetchit/fetchit.asl "
            + "fetchit/fetchit_manipulation.asl vision.asl "
            + "navigation.asl "
            + "dialogue/nlu.asl dialogue/nlg.asl dialogue/handleSemantics.asl "
            + "-performancefile fetchit.json "
            + "-goal listen(self:agent)";

    if (showGUI) {
      gmArgs += "-editor ";
    }
    gm = createInstance(GoalManagerImpl.class, gmArgs);
  }

  public static void main(String[] args) {
    FetchPerformanceAssessmentDemoMock demoConfig = new FetchPerformanceAssessmentDemoMock();
    demoConfig.runConfiguration();
  }
}
