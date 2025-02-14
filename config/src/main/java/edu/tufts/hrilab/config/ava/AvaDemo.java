package edu.tufts.hrilab.config.ava;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class AvaDemo extends DiarcConfiguration {
  boolean useMockAva = false;
  boolean useMicrophone = false;
  boolean useVision = false;
  boolean useGUI = false;
  boolean useQueueEM = true;
  String avaArgs = "";

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("showGuis").desc("show guis").build());
    options.add(Option.builder("useMic").desc("turn on on board speech recognition").build());
    options.add(Option.builder("useVision").desc("instantiate vision component").build());
    options.add(Option.builder("username").hasArg().desc("set ava username").build());
    options.add(Option.builder("password").hasArg().desc("set ava password").build());
    options.add(Option.builder("uri").hasArg().desc("set ava uri").build());
    options.add(Option.builder("map").hasArg().desc("set map file").build());
    options.add(Option.builder("mock").desc("use mock ava component").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("showGuis")) {
      useGUI = true;
    }
    if (cmdLine.hasOption("mock")) {
      useMockAva = true;
    }
    if (cmdLine.hasOption("useMic")) {
      useMicrophone = true;
    }
    if (cmdLine.hasOption("useVision")) {
      useVision = true;
    }
    if (cmdLine.hasOption("username")) {
      avaArgs += " -username " + cmdLine.getOptionValue("username");
    }
    if (cmdLine.hasOption("password")) {
      avaArgs += "-password " + cmdLine.getOptionValue("password");
    }
    if (cmdLine.hasOption("uri")) {
      avaArgs += " -uri " + cmdLine.getOptionValue("uri");
    }
    if (cmdLine.hasOption("map")) {
      avaArgs += " -map " + cmdLine.getOptionValue("map");
    }
  }
  public void runConfiguration() {
    log.info("Starting Ava Demo Config");

    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, "-tldl avadict.dict avaTemplate.dict -cacheName ava");
    createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class);
    createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);
    createInstance(edu.tufts.hrilab.slug.nlg.SimpleNLGComponent.class);
    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class, "-wakeWord ava robot -wakeBypass evan -wakeTimeout 0");

    if (useGUI) {
      createInstance(edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent.class, "-config ava.simspeech -speaker patient -addressee self ");
    }
    createInstance(edu.tufts.hrilab.tts.MaryTTSComponent.class, " -male -volume 0.0 -showMixers -matchMixer ALC256 Analog" + ((useMicrophone) ? " -muteMics" : ""));
    //createInstance(edu.tufts.hrilab.tts.opentts.OpenTTSComponent.class, "-endpoint http://192.168.122.140:5500 -voice coqui-tts:en_ljspeech -cache");

    if (useMicrophone) {
      createInstance(edu.tufts.hrilab.asr.sphinx4.Sphinx4Component.class, "-showMixers -matchMixer Direct Audio Device: ReSpeaker 4 Mic Array -speaker patient -addressee self -grammar ava -config ava.xml");
//      createInstance(edu.tufts.hrilab.asr.whisper.WhisperComponent.class, "-model small.en -speaker patient -addressee self -matchMixer Direct Audio Device: ReSpeaker 4 Mic Array -showMixers -threshold 200.0");
    }
    //createInstance(edu.tufts.hrilab.slug.http.HTTPListenerComponent.class, "-speaker patient -addressee self");

    createInstance(edu.tufts.hrilab.map.MapComponent.class, "-refs refs/avaLabRefs.json");

//    if (useVision) {
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture ava.xml -refs refs/towerRefs.json " + (useGUI ? "" : "-hideControls"));
//    }

//    createInstance(edu.tufts.hrilab.ava.avautil.AvaDisplayComponent.class, ((useGUI) ? "" : "-fullscreen") + " -chat");
    if (useMockAva) {
      createInstance(edu.tufts.hrilab.ava.MockAvaComponent.class, avaArgs);
    } else {
      createInstance(edu.tufts.hrilab.ava.AvaComponent.class, avaArgs); //Ava_SomervilleHQ_Master_12262023_G
    }

    createInstance(edu.tufts.hrilab.llm.LLMComponent.class);
    createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class,
      "-goal listen(self) monitorBattery(self) -beliefinitfile demos.pl agents/ava.pl"
        + " -dbfile core.asl manipulation.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl vision.asl tower/ava.asl tower/avaVlm.asl"
        + " -priorityFile default.json avaPriority.json"
        + (useQueueEM ? " -executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager" : ""));

    log.info("Finished Ava Demo Config");
  }

}


