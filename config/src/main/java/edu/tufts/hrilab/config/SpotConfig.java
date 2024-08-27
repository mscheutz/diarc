package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.arm.MockArmComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.MockRos2SpotComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.vision.MockVisionComponent;

public class SpotConfig extends DiarcConfiguration {

  boolean useMockVision = true;
  boolean useMockSpot = true;

  public SpotConfig(boolean mock) {
    useMockVision = mock;
    useMockSpot = mock;
  }

  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict spot.dict");
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class, "");
    createInstance(SimpleNLGComponent.class, "");
    createInstance(SimSpeechProductionComponent.class, "");

    if (useMockSpot) {
      createInstance(MockArmComponent.class, "-groups agent:self:agent");
      createInstance(MockMoveBaseComponent.class, "-groups agent:self:agent");
      createInstance(MockRos2SpotComponent.class, "-groups agent:self:agent -locationRefs vaLocationRefs.json");
    } else {
      createInstance(SpotComponent.class, "-groups agent:self:agent -map /home/pete/dev/spot_maps/va.walk -locationRefs vaLocationRefs.json");
    }

    if (useMockVision) {
      createInstance(MockVisionComponent.class, "-groups agent:self:agent -belief -sceneClass edu.tufts.hrilab.vision.scene.config.SpotSceneGenerator");
    } else {
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-groups agent:self:agent -belief -capture spot.xml -cameraFrame hand_color_image_sensor");
    }

    createInstance(GoalManagerImpl.class,
            "-beliefg " +
                    "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                    "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-beliefinitfile demos/spotVA.pl "
                    + "-editor "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl vision.asl "
                    + "-asl demos/spotDemo.asl "
                    + "-asl spot/spotActions.asl "
                    + "-goal listen(self:agent) "
                    + "-goal init(self:agent) "
//                    + "-goal deliver_to(self:agent,physobj_2:physobj,location_1:location) "
                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "

    );

    createInstance(SimSpeechRecognitionComponent.class, "-speaker evan -config spotDemo.simspeech");
  }

  public static void main(String[] args) {
    SpotConfig config = new SpotConfig(true);
    config.runConfiguration();
  }
}
