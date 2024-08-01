package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.rl.RLActionManager;

public class RLConfig extends DiarcConfiguration {
    @Override
    public void runConfiguration() {
        createInstance(GoalManagerImpl.class, "-editor " +
                "-beliefinitfile agents/agents.pl domains/hanoi.pl " +
                "-asl hanoi/hanoi.asl " +
                "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +
                "-goal goal(self,on(cube3:disc,peg3:stackable)) "
        );
        createInstance(RLActionManager.class);
    }

    public static void main(String[] args) {
        new RLConfig().runConfiguration();
    }
}
