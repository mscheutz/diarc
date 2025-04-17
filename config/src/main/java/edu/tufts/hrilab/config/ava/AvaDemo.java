package edu.tufts.hrilab.config.ava;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class AvaDemo extends DiarcConfiguration {
  boolean useMockAva = false;
  boolean useMicrophone = false;
  boolean useVision = false;
  boolean useGUI = false;
  boolean useQueueEM = true;
  String avaArgs = "";
  boolean useLLMParser = true;
  boolean useLLMNLG = true;
  String service = "llamahf";
  String model = "Llama-3.3-70B-Instruct";
  boolean usePrivateEndpoint = false; //Do not set to true, use arg

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
    options.add(Option.builder("privateEndpoint").desc("run using privately hosted endpoint rather than free serverless endpoint").build());
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
    DiarcComponent.createInstance(edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent.class, nlgArgs);
    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

    if (useLLMNLG || useLLMParser) {
      createInstance(edu.tufts.hrilab.llm.serviceproviders.AvaLLMServiceProvider.class,"");
      String llmComponentArgs = String.format("-service %s -model %s -temperature 0.6", service, model);
      if (usePrivateEndpoint) {
        //llmComponentArgs += " -endpoint https://z6xvb4wfqwn59s1r.us-east-1.aws.endpoints.huggingface.cloud";
        llmComponentArgs += " -endpoint https://yz9rcl3hkr2vkvhj.us-east-1.aws.endpoints.huggingface.cloud";
      }
      createInstance(edu.tufts.hrilab.llm.LLMComponent.class, llmComponentArgs);
    }

    if (useGUI) {
      createInstance(edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent.class, "-config ava.simspeech -speaker evan -addressee ava");
    }
//    createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");
    createInstance(edu.tufts.hrilab.tts.MaryTTSComponent.class, " -male -volume 10.0 -showMixers -matchMixer ALC256 Analog" + ((useMicrophone) ? " -muteMics" : ""));
    //createInstance(edu.tufts.hrilab.tts.opentts.OpenTTSComponent.class, "-endpoint http://92.168.122.140:5500 -voice coqui-tts:en_ljspeech -cache");

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

    createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class,
      "-goal listen(self) monitorBattery(ava) -beliefinitfile demos.pl agents/ava.pl"
        + " -dbfile core.asl manipulation.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl vision.asl ava/ava.asl ava/scheduledGoals.asl demos/queueEM.asl"
        + " -priorityFile default.json avaPriority.json"
        + (useQueueEM ? " -executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager" : ""));

    log.info("Finished Ava Demo Config");
  }

}


