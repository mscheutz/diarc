package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.spot.SpotComponent;

public class SpotConfig extends DiarcConfiguration {
  @Override
  public void runConfiguration() {
    createInstance(SpotComponent.class, "-map /home/code/graph");
    try {
      Thread.sleep(8000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    createInstance(GoalManagerImpl.class, "-beliefinitfile agents/agents.pl -editor");
  }

  public static void main(String[] args) {
    new SpotConfig().runConfiguration();
  }
}
