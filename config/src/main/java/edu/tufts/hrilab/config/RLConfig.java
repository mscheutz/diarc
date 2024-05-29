package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.rl.RLActionManager;

public class RLConfig extends DiarcConfiguration {
    @Override
    public void runConfiguration() {
        createInstance(GoalManagerImpl.class, "-editor -beliefinitfile agents/agents.pl");
        createInstance(RLActionManager.class);
    }

    public static void main(String[] args) {
        new RLConfig().runConfiguration();
    }
}
