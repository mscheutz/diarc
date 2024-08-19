package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.spot.SpotComponent;

public class SpotConfig extends DiarcConfiguration {
  @Override
  public void runConfiguration() {
    createInstance(SpotComponent.class, "-map /home/pete/dev/spot_maps/lab81524.walk -locationRefs spotLocationRefs.json");

    createInstance(GoalManagerImpl.class,
            "-beliefinitfile agents/agents.pl spotVA.pl"
            + "-editor "
            + "-asl demos/spotDemo.asl "
            + "-goal init(self:agent)"
    );
  }

  public static void main(String[] args) {
    new SpotConfig().runConfiguration();
  }
}
