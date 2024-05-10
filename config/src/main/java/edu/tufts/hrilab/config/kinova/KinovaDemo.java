/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.examples.robots;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.moveit.MoveItComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class KinovaDemo extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(KinovaDemo.class);

    // start the configuration
    public static void main(String[] args) {
        DiarcConfiguration config = new KinovaDemo();
        config.runConfiguration();
    }

    @Override
    public void runConfiguration() {
        MoveItComponent instance = DiarcComponent.createInstance(MoveItComponent.class, "-config kortex.json");
//        DiarcComponent.createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-cameraFrame camera_color_frame -capture kinova.xml -refs refs/visionRefs.json");
        DiarcComponent.createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class,
                "-goal startKinova(self) -beliefinitfile demos.pl agents/agents.pl "
                        + "-dbfile core.asl vision.asl manipulation.asl dialogue/nlg.asl dialogue/handleSemantics.asl "
                        + "kinova/kinova_examples.asl "
                        + "-editor");

    }
}
