/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.tower;
import edu.tufts.hrilab.vision.VisionComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.action.GoalManagerImpl;

public class VisionExampleConfig extends DiarcConfiguration {
    static public VisionComponent vision;
    static public GoalManagerImpl gm;

    public void runConfiguration() {
        vision = createInstance(VisionComponent.class, "-refs towerRefs.json -belief");
        gm = createInstance(GoalManagerImpl.class, " -dbfile visionExample.asl -editor -beliefg");
    }

    public static void main(String[] args) {
        VisionExampleConfig visionExampleExample = new VisionExampleConfig();
        visionExampleExample.runConfiguration();
    }
}// -goal persistent(lookForObject(self:agent,A)