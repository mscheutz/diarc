package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.vision.VisionComponent;

public class SpotConfig extends DiarcConfiguration {
  @Override
  public void runConfiguration() {
    createInstance(SpotComponent.class, "-map /home/pete/dev/spot_maps/short.walk -locationRefs phiLocationRefs.json");

    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }

//    createInstance(VisionComponent.class, "-groups agent:self:agent -belief");
    createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict spot.dict");

    createInstance(GoalManagerImpl.class,
            "-beliefinitfile agents/agents.pl demos/spotVA.pl "
            + "-editor "
            + "-asl core.asl "
            + "-asl dialogue/nlg.asl "
            + "-asl dialogue/nlu.asl "
            + "-asl dialogue/handleSemantics.asl "
            + "-asl vision.asl "
            + "-asl demos/spotDemo.asl "
            + "-goal init(self:agent)"
    );
  }

  public static void main(String[] args) {
    new SpotConfig().runConfiguration();
  }
}
