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
  boolean useLLMParser = true;
  boolean useLLMNLG = true;
  String service = "openai";
  String model = "Qwen/Qwen3-14B-AWQ";
  String openaiEndpoint = "http://dashboard.thinkingrobots.ai:8000/qwen";
  boolean usePrivateEndpoint = false; //Do not set to true, use arg
  private String locationRefs = "refs/locations.json";
  private String speakerId = "";
  private boolean initKiosk = true;
  private boolean persistKiosk = true;
  private String mediaDir = "ava";
  private boolean useDisplay = true;


  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("showGuis").desc("show guis").build());
    options.add(Option.builder("useMic").desc("turn on on board speech recognition").build());
    options.add(Option.builder("useVision").desc("instantiate vision component").build());
    options.add(Option.builder("uri").hasArg().desc("set ava uri").build());
    options.add(Option.builder("mock").desc("use mock ava component").build());
    options.add(Option.builder("privateEndpoint").desc("run using privately hosted endpoint rather than free serverless endpoint").build());
    options.add(Option.builder("locationRefs").hasArg().desc("set location references file").build());
    options.add(Option.builder("speakerId").hasArg().desc("name of speaker to route local tts through").build());
    options.add(Option.builder("kiosk").desc("start the demo with kiosk mode enabled").build());
    options.add(Option.builder("mediaDir").hasArg().desc("name of directory in $HOME/.diarc where media files are stored").build());
    options.add(Option.builder("disableDisplay").desc("add to run without a display").build());
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
    if (cmdLine.hasOption("uri")) {
      avaArgs += " -uri " + cmdLine.getOptionValue("uri");
    }
    if (cmdLine.hasOption("locationRefs")) {
      locationRefs = cmdLine.getOptionValue("locationRefs");
    }
    if (cmdLine.hasOption("speakerId")) {
      speakerId = cmdLine.getOptionValue("speakerId");
    }
    if (cmdLine.hasOption("mediaDir")) {
      mediaDir = cmdLine.getOptionValue("mediaDir");
    }
    if (cmdLine.hasOption("privateEndpoint")) {
      log.warn("You have configured the system to use the privately hosted HuggingFace endpoint.");// THIS COSTS MONEY AND SHOULD NOT BE USED FOR FRIVOLOUS TESTING. If you are doing this intentionally, please enter 1 into the console");
      usePrivateEndpoint = true;
      //System.out.println("Enter: ");
      //Scanner reader = new Scanner(System.in);
      //int in = reader.nextInt();
      //if (in == 1) {
      //  log.warn("using private endpoint");
      //  usePrivateEndpoint = true;
      //} else {
      //  log.warn("not using private endpoint");
      //}
    } if (cmdLine.hasOption("disableDisplay")) {
      useDisplay = false;
    }
    if (cmdLine.hasOption("kiosk")) {
      initKiosk = true;
    }
  }
  public void runConfiguration() {
    log.info("Starting Ava Demo Config");

    String parserArgs = "-tldl avadict.dict avaTemplate.dict -cache -cacheName ava -cachePersist";
    if (useLLMParser) {
      parserArgs += " -llm";
    }
    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, parserArgs);
    createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class);
    createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);
//    createInstance(edu.tufts.hrilab.slug.nlg.SimpleNLGComponent.class);
    String nlgArgs = "-simpleNLG -cache -cacheName ava-reverse -cachePersist";
    if (useLLMNLG) {
      nlgArgs += " -llm";
    }
    createInstance(edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent.class, nlgArgs);
    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

    if (useLLMNLG || useLLMParser) {
      createInstance(edu.tufts.hrilab.llm.serviceproviders.AvaLLMServiceProvider.class,"");
      String llmComponentArgs = String.format("-service %s llamahf -model %s Qwen/Qwen3-14B -temperature 0.6 0.6", service, model);
      if (service.equals("llamahf")) {
        if (usePrivateEndpoint) {
          llmComponentArgs += " -endpoint https://wwil02mugudnd3ky.us-east-1.aws.endpoints.huggingface.cloud";
        }
      } else if (service.equals("azure")) {
        llmComponentArgs += " -endpoint https://tracs-southcentralus-lezah.southcentralus.inference.ml.azure.com/score";
      } else if (service.equals("openai") && !openaiEndpoint.isEmpty()) {
        llmComponentArgs += " -endpoint " + openaiEndpoint;
      }
      createInstance(edu.tufts.hrilab.llm.LLMComponent.class, llmComponentArgs);
    }

    if (useGUI) {
      createInstance(edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent.class, "-config ava.simspeech -speaker webapp -addressee ava");
    }
//    createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");
    createInstance(edu.tufts.hrilab.tts.MaryTTSComponent.class, " -female -volume 0.3 -showMixers");// -matchMixer Direct Audio Device: HDA Intel PCH, HDMI 0, CS-CODECPLU-3");
    //createInstance(edu.tufts.hrilab.tts.opentts.OpenTTSComponent.class, "-endpoint http://92.168.122.140:5500 -voice coqui-tts:en_ljspeech -cache");

//      createInstance(edu.tufts.hrilab.asr.sphinx4.Sphinx4Component.class, "-showMixers -matchMixer Direct Audio Device: ReSpeaker 4 Mic Array -speaker patient -addressee self -grammar ava -config ava.xml");
//    if (useMicrophone) {
//      createInstance(edu.tufts.hrilab.asr.whisper.WhisperComponent.class, "-model small.en -speaker patient -addressee self -matchMixer Direct Audio Device: C922 Pro Stream Webcam, USB Audio, USB Audio -showMixers -threshold 600.0 -useDisplay");
//      createInstance(edu.tufts.hrilab.asr.whisper.WhisperComponent.class, "-model small.en -speaker patient -addressee self -matchMixer Direct Audio Device: ReSpeaker 4 Mic Array -showMixers -threshold 200.0");
//      createInstance(edu.tufts.hrilab.asr.whisper.WhisperComponent.class, "-model small.en -speaker patient -addressee self -matchMixer Direct Audio Device: UHAUTCA, USB Audio, USB Audio -showMixers -threshold 200.0 -mixer 19 -useDisplay");
      //createInstance(edu.tufts.hrilab.slug.http.HTTPListenerComponent.class, "-speaker patient -addressee self");
//    }


    String mapArgs = "";
    if (useMockAva) {
      mapArgs += "-refs " + locationRefs;
    }
    createInstance(edu.tufts.hrilab.map.MapComponent.class, mapArgs);

//    if (useVision) {
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture ava.xml -refs refs/towerRefs.json " + (useGUI ? "" : "-hideControls"));
//    }

    if (useDisplay) {
      createInstance(edu.tufts.hrilab.ava.AvaDisplayComponent.class, ((useGUI) ? "" : "-fullscreen") + " -chat -mediaDir " + mediaDir + " -videoVolume 10");
    }
    if (useMockAva) {
      createInstance(edu.tufts.hrilab.ava.MockAvaComponent.class, avaArgs);
    } else {
      createInstance(edu.tufts.hrilab.ava.AvaComponent.class, avaArgs);
    }

    String kioskArgs = "";
    if (initKiosk) {
      kioskArgs += " -enabled -timeout 2700";
    }
    if (persistKiosk) {
      kioskArgs += " -kioskSettingsFile ava.ser";
    }

    createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class,
            "-historylength 3600000 "
        +   "-goal listen(self) monitorBattery(ava) listenForWWandASR(ava) " +
                    "-beliefinitfile demos.pl agents/ava.pl domains/ava.pl"
        + " -dbfile " +
              "core.asl " +
              "manipulation.asl " +
              "dialogue/nlg.asl " +
              "dialogue/nlu.asl " +
              "dialogue/handleSemantics.asl " +
              "vision.asl " +
              "ava/ava.asl " +
              "ava/avaIntrospection.asl " +
              "ava/scheduledGoals.asl " +
              "ava/avaMedia.asl " +
              "ava/avaASR.asl " +
              "ava/avaTMCDemo.asl " +
              "demos/queueEM.asl"
        + " -priorityFile default.json avaPriority.json"
        + " -scheduledGoalsFile ava.ser"
        + " -learnedActionsFile avaDemo.asl"
        + (useQueueEM ? " -executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager" : ""));

    createInstance(edu.tufts.hrilab.action.manager.KioskGoalManager.class, kioskArgs);

    createInstance(edu.tufts.hrilab.ava.gui.AvaGuiProvider.class);
    createInstance(edu.tufts.hrilab.ava.gui.AvaMediaProvider.class);

    log.info("Finished Ava Demo Config");
  }

}


