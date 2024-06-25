/*
 * DIARC
 * @author Evan Krause
 *
 * Copyright 1997-2023 Matthias Scheutz and HRILab (hrilab.tufts.edu) All rights reserved.
 * Do not copy and use without permission. For questions contact Matthias Scheutz at <matthias.scheutz@tufts.edu>
 */

package edu.tufts.hrilab.config.gui;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.DemoApplication;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.map.MapComponent;

import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.vision.MockVisionComponent;
import org.springframework.boot.SpringApplication;

public class FetchCaddyDemoMock extends DiarcConfiguration {
  // start the configuration
  @Override
  public void runConfiguration() {

    createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict multiRobotCaddy.dict -dict multiRobotCaddyHomophones.dict");

    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

    createInstance(ReferenceResolutionComponent.class);

    createInstance(DialogueComponent.class);

    createInstance(SimpleNLGComponent.class);

    createInstance(MockVisionComponent.class, "-groups agent:fetch:fetch -belief -sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");

    createInstance(edu.tufts.hrilab.fetch.MockFetchItComponent.class, "-groups agent:fetch:fetch -simExecTime");
    createInstance(edu.tufts.hrilab.tf.MockTFComponent.class, "-groups agent:fetch:fetch");

    // locationRefsMultiRobotCaddy.json locations are for the fetchit_map built on 2023_11_08
    createInstance(MapComponent.class, "-kb_name movebaselocation -refs refs/locationRefsMultiRobotCaddy.json -groups agent:fetch:fetch");

    createInstance(MockMoveBaseComponent.class, "-groups agent:fetch:fetch -simExecTime");

    createInstance(SimSpeechProductionComponent.class, "");

    String goalManagerArgs = "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
            + "-beliefinitfile demos.pl "
            + "-beliefinitfile agents/multiRobotCaddyAgents.pl "
            + "-beliefinitfile demos/multiRobotCaddy/multiRobotCaddy.pl "
            + "-beliefinitfile demos/multiRobotCaddy/environmentFacts.pl "
            + "-beliefinitfile demos/multiRobotCaddy/fetchFacts.pl "
            + "-nomemorymanager "
            + "-asl core.asl "
            + "-asl dialogue/nlg.asl "
            + "-asl dialogue/handleSemantics.asl "
            + "-asl domains/tr/multiRobotCaddyRecipe.asl "
            + "-asl demos/multiRobotCaddy.asl "
            + "fetchit/fetchit_manipulation.asl "
            + "manipulation.asl "
            + "vision.asl "
            + "navigation.asl "
            + "brad:human:demos/multiRobotCaddyHuman.asl ";
//            + "-performancefile multiRobotCaddy.json ";

    createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class, goalManagerArgs);

    //createInstance(com.trade.gui.TRADEServiceGui.class, "-class com.fetch.FetchComponentImpl -groups agent:fetch:fetch");

    //brad: this is last so the GUI window is on top
    createInstance(SimSpeechRecognitionComponent.class,
            "-config multiRobotCaddy.simspeech -speaker evan");
//                "-config multiRobotCaddy.simspeech -speaker evan -listener fetch:fetch");
  }

  public static void main(String[] args) {
    FetchCaddyDemoMock demoConfig = new FetchCaddyDemoMock();
    SpringApplication.run(DemoApplication.class, args);
    demoConfig.runConfiguration();
  }
}
