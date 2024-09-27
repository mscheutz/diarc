/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.tower;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fetch.FetchComponent;
import edu.tufts.hrilab.fetch.MockFetchComponent;
import edu.tufts.hrilab.lidar.LidarDoorComponent;
import edu.tufts.hrilab.movebase.ChangeMapComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.tower.ElevatorBoardingComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.vision.MockVisionComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class TowerConfig extends DiarcConfiguration {

  private boolean mock = false;
  private boolean useNLP = true;
  private boolean useGUI = true;

  private String map_folder = "jcc_building_maps";
  private int floor = 4;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("lab").desc("show dialogue gui").build());
    options.add(Option.builder("headless").desc("show dialogue gui").build());
    options.add(Option.builder("mock").desc("show dialogue gui").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("lab")) {
      log.info("Starting inside lab...");
      map_folder = "elevator_lab_test";
      floor = 1;
    }
    if (cmdLine.hasOption("headless")) {
      log.info("Starting headless...");
      useGUI = false;
    }
    if (cmdLine.hasOption("mock")) {
      log.info("Starting with mock...");
      mock = true;
    }
  }

  // start the configuration
  @Override
  public void runConfiguration() {
    if (useNLP) {
      log.info("Starting using NLP pipeline...");

      if (useGUI) {
        createInstance(SimSpeechRecognitionComponent.class,
                "-config demodialogues/fetchconfig.simspeech -speaker brad -addressee self");
      }
      createInstance(TLDLParserComponent.class, "-dict templatedict.dict");

      createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

      createInstance(ReferenceResolutionComponent.class);

      createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

      createInstance(SimpleNLGComponent.class);

      createInstance(SimSpeechProductionComponent.class);

    } else {
      createInstance(ReferenceResolutionComponent.class);
    }

    createInstance(TFComponent.class);
    createInstance(LidarDoorComponent.class);
    createInstance(ElevatorBoardingComponent.class);
    createInstance(ChangeMapComponent.class);

    createInstance(MapComponent.class, "-map_folder " + map_folder + " -ros_package hrilab_maps -start_floor " + floor + " -refs refs/jccRefs.json");

    if (mock) {
      log.info("Starting using mock components...");
      createInstance(MockFetchComponent.class);
      createInstance(MockMoveBaseComponent.class);
      createInstance(MockVisionComponent.class, "-refs towerRefs.json");
    } else {
      createInstance(FetchComponent.class, "-config Fetch_TOWER.json -pose tuck -head 0.5 0.0 1.0 -torso 0.3");
      createInstance(MoveBaseComponent.class);
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-cameraFrame head_camera_rgb_optical_frame -capture fetch.xml -refs refs/towerRefs.json " + (useGUI ? "" : "-hideControls"));
    }

    createInstance(GoalManagerComponent.class,
            "-beliefinitfile demos.pl agents/agents.pl tower/tower.pl "
                    + "-dbfile core.asl vision.asl manipulation.asl dialogue/nlg.asl dialogue/handleSemantics.asl "
                    + "-dbfile tower/pressingButtons.asl tower/boardElevator.asl "
                    + (useGUI ? "-editor" : "")
    );

//    createInstance(com.trade.gui.TRADEServiceGui.class, "-class com.nao.MockNaoComponent");
  }

}
