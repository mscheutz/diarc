package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.vision.VisionComponent;

public class SpotConfig extends DiarcConfiguration {
  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict spot.dict");
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class, "");
    createInstance(SimpleNLGComponent.class, "");
    createInstance(SimSpeechProductionComponent.class, "");

    createInstance(SpotComponent.class, "-groups agent:self:agent -map /home/pete/dev/spot_maps/short.walk -locationRefs phiLocationRefs.json");
    try {
      Thread.sleep(10000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    createInstance(VisionComponent.class, "-groups agent:self:agent -belief -capture spot.xml -cameraFrame hand_color_image_sensor");


    createInstance(GoalManagerImpl.class,
            "-beliefinitfile agents/agents.pl "
            + "-beliefinitfile demos.pl "
            + "-editor "
            + "-asl core.asl "
            + "-asl dialogue/nlg.asl "
            + "-asl dialogue/nlu.asl "
            + "-asl dialogue/handleSemantics.asl "
            + "-asl vision.asl "
            + "-asl demos/spotDemo.asl "
            + "-goal listen(self:agent) "
            + "-goal init(self:agent) "
    );

    createInstance(SimSpeechRecognitionComponent.class, "-speaker evan -config spotDemo.simspeech");
  }

  public static void main(String[] args) {
    new SpotConfig().runConfiguration();
  }
}
