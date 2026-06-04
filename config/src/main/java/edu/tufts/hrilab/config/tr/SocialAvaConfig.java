package edu.tufts.hrilab.config.tr;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.llm.serviceproviders.tr.social.SocialTRLLMServiceProvider;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class SocialAvaConfig extends DiarcConfiguration {
  public boolean useMockAva = false;
  boolean useMicrophone = false;
  boolean useVision = false;
  public boolean useGUI = false;
  boolean useQueueEM = true;
  String avaArgs = "";
  boolean useLLMParser = true;
  boolean useLLMNLG = true;
  String service = "openai";
  String model = "QuantTrio/Qwen3.5-9B-AWQ";
  //  String model = "Qwen/Qwen3-14B-AWQ";
  //  String model = "meta-llama/Llama-3.1-8B-Instruct:scaleway";
  // Default: direct to marvin vLLM via Tailscale (skips facade nginx hop, eliminates 504-under-contention failure mode).
  // Override with -llmEndpoint <url> to route through facade (e.g., https://dashboard.thinkingrobots.ai/qwen).
  // OpenaiClient.setEndpoint appends /v1/chat/completions - pass base URL only.
  String openaiEndpoint = "http://100.113.66.111:9000";
  boolean usePrivateEndpoint = false; //Do not set to true, use arg
  public String locationRefs = "refs/locations.json";
  public String speakerId = "";
  public String videoAudioDevice = "";
  public boolean initKiosk = false;
  public boolean persistKiosk = true;
  public String mediaDir = "ava";
  public boolean useDisplay = true;
  public boolean useCartesia = false;
  public String cartesiaVoiceId = "";
  public boolean useLocalTTS = false;
  public String localVoice = "";
  public String ttsEndpoint = "";
  public String voiceVolume = "1.0";
  public String voiceSpeed = "normal";
  public String voiceClipVolume = "0.4";
  public boolean muteMics = false;
  public SimSpeechRecognitionComponent simSpeechRec;
  public SocialTRLLMServiceProvider serviceProvider;
  public boolean llmOnly = true;


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
    options.add(Option.builder("videoAudioDevice").hasArg().desc("ALSA device for video playback (empty = vlc/system default)").build());
    options.add(Option.builder("kiosk").desc("start the demo with kiosk mode enabled").build());
    options.add(Option.builder("mediaDir").hasArg().desc("name of directory in $HOME/.diarc where media files are stored").build());
    options.add(Option.builder("disableDisplay").desc("add to run without a display").build());
    options.add(Option.builder("cartesia").desc("use Cartesia TTS instead of MaryTTS").build());
    options.add(Option.builder("cartesiaVoiceId").hasArg().desc("Cartesia voice UUID; overrides CARTESIA_VOICE_ID env").build());
    options.add(Option.builder("localtts").desc("use local self-hosted streaming TTS (Kokoro sidecar) instead of MaryTTS").build());
    options.add(Option.builder("localVoice").hasArg().desc("local TTS voice name (default am_onyx)").build());
    options.add(Option.builder("ttsEndpoint").hasArg().desc("local TTS sidecar base URL (default http://127.0.0.1:5002)").build());
    options.add(Option.builder("voiceVolume").hasArg().desc("Cartesia voice volume 0.0-2.0 (default 1.0)").build());
    options.add(Option.builder("voiceSpeed").hasArg().desc("Cartesia voice speed: slowest|slow|normal|fast|fastest or float (default normal)").build());
    options.add(Option.builder("voiceClipVolume").hasArg().desc("ding/cue clip volume 0.0-1.0 (default 0.4)").build());
    options.add(Option.builder("muteMics").desc("disable mics during TTS playback").build());
    options.add(Option.builder("llmEndpoint").hasArg().desc("override LLM endpoint URL (default: direct to marvin vLLM over Tailscale)").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("showGuis")) {
      useGUI = true;
    }
    if (cmdLine.hasOption("mock")) {
      useMockAva = true;
      speakerId = "";
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
    if (cmdLine.hasOption("videoAudioDevice")) {
      videoAudioDevice = cmdLine.getOptionValue("videoAudioDevice");
    }
    if (cmdLine.hasOption("mediaDir")) {
      mediaDir = cmdLine.getOptionValue("mediaDir");
    }
    if (cmdLine.hasOption("privateEndpoint")) {
      log.warn("You have configured the system to use the privately hosted HuggingFace endpoint.");// THIS COSTS MONEY AND SHOULD NOT BE USED FOR FRIVOLOUS TESTING. If you are doing this intentionally, please enter 1 into the console");
      usePrivateEndpoint = true;
    } if (cmdLine.hasOption("disableDisplay")) {
      useDisplay = false;
    }
    if (cmdLine.hasOption("kiosk")) {
      initKiosk = true;
    }
    if (cmdLine.hasOption("cartesia")) {
      useCartesia = true;
    }
    if (cmdLine.hasOption("cartesiaVoiceId")) {
      cartesiaVoiceId = cmdLine.getOptionValue("cartesiaVoiceId");
    }
    if (cmdLine.hasOption("localtts")) {
      useLocalTTS = true;
    }
    if (cmdLine.hasOption("localVoice")) {
      localVoice = cmdLine.getOptionValue("localVoice");
    }
    if (cmdLine.hasOption("ttsEndpoint")) {
      ttsEndpoint = cmdLine.getOptionValue("ttsEndpoint");
    }
    if (cmdLine.hasOption("voiceVolume")) {
      voiceVolume = cmdLine.getOptionValue("voiceVolume");
    }
    if (cmdLine.hasOption("llmEndpoint")) {
      openaiEndpoint = cmdLine.getOptionValue("llmEndpoint");
    }
    if (cmdLine.hasOption("voiceSpeed")) {
      voiceSpeed = cmdLine.getOptionValue("voiceSpeed");
    }
    if (cmdLine.hasOption("voiceClipVolume")) {
      voiceClipVolume = cmdLine.getOptionValue("voiceClipVolume");
    }
    if (cmdLine.hasOption("muteMics")) {
      muteMics = true;
    }
  }
  public void runConfiguration() {
    log.info("Starting Ava Demo Config");

    String parserArgs;
    if (llmOnly) {
      parserArgs = "-llm";
    } else {
      // Added LLM before TLDL in sequence because some TLDL ITK targets (and just in general targets) are problematic/out of date
      //  TODO: ^at least want this for testing. If we want direct ITK versions still available for deployment can swap back
      parserArgs = "-tldl avadict.dict avaTemplate.dict -cache -cacheName ava -cachePersist -llm -sequence cache llm tldl";
    }
    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.domains.AvaHybridParserComponent.class, parserArgs);
    createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class);
    createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);
    String nlgArgs;
    if (llmOnly) {
      nlgArgs = "-llm";
    } else {
      nlgArgs = "-simpleNLG -cache -cacheName ava-reverse -cachePersist -llm -sequence llm cache simple";
    }
    createInstance(edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent.class, nlgArgs);
    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

//    if (useLLMNLG || useLLMParser) {
//      //createInstance(SocialAvaLLMServiceProvider.class,"");
//      createInstance(AvaLLMServiceProvider.class,"");
//      String llmComponentArgs;
////      if (service.equals("hf") && model.startsWith("Qwen/Qwen3-14B")) {
//      llmComponentArgs = String.format("-service %s -model %s -temperature 0.6 -tool", service, model);
////      }
////      else {
////        llmComponentArgs = String.format("-service %s hf -model %s Qwen/Qwen3-14B -temperature 0.6 0.6", service, model);
////      }
//      if (service.equals("llamahf")) {
//        if (usePrivateEndpoint) {
//          llmComponentArgs += " -endpoint https://wwil02mugudnd3ky.us-east-1.aws.endpoints.huggingface.cloud";
//        }
//      } else if (service.equals("azure")) {
//        llmComponentArgs += " -endpoint https://tracs-southcentralus-lezah.southcentralus.inference.ml.azure.com/score";
//      } else if (service.equals("openai") && !openaiEndpoint.isEmpty()) {
//        llmComponentArgs += " -endpoint " + openaiEndpoint;
//      }
//      createInstance(edu.tufts.hrilab.llm.LLMComponent.class, llmComponentArgs);
//    }

    if (useGUI) {
      simSpeechRec = createInstance(edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent.class, "-config ava.simspeech -speaker webapp -addressee ava");
    }
//    createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");
    if (useLocalTTS) {
      String localArgs = " -volume " + voiceVolume + " -speed " + voiceSpeed + " -clipVolume " + voiceClipVolume;
      if (!speakerId.isEmpty()) {
        localArgs += " -matchMixer " + speakerId;
      }
      if (!localVoice.isEmpty()) {
        localArgs += " -voiceId " + localVoice;
      }
      if (!ttsEndpoint.isEmpty()) {
        localArgs += " -ttsEndpoint " + ttsEndpoint;
      }
      if (muteMics) {
        localArgs += " -muteMics";
      }
      createInstance(edu.tufts.hrilab.ava.tts.AvaLocalTTSComponent.class, localArgs);
    } else if (useCartesia) {
      String cartesiaArgs = " -volume " + voiceVolume + " -speed " + voiceSpeed + " -clipVolume " + voiceClipVolume;
      if (!speakerId.isEmpty()) {
        cartesiaArgs += " -matchMixer " + speakerId;
      }
      if (!cartesiaVoiceId.isEmpty()) {
        cartesiaArgs += " -voiceId " + cartesiaVoiceId;
      }
      if (muteMics) {
        cartesiaArgs += " -muteMics";
      }
      createInstance(edu.tufts.hrilab.ava.tts.AvaCartesiaTTSComponent.class, cartesiaArgs);
    } else {
      String maryTTSArgs = " -female -volume 1.0 -showMixers";
      if (!speakerId.isEmpty()) {
        maryTTSArgs += " -matchMixer " + speakerId;
      }
      createInstance(edu.tufts.hrilab.ava.tts.AvaMaryTTSComponent.class, maryTTSArgs);
    }

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
      String displayArgs = ((useGUI) ? "" : "-fullscreen") + " -chat -mediaDir " + mediaDir + " -videoVolume 90";
      if (!videoAudioDevice.isEmpty()) {
        displayArgs += " -videoAudioDevice " + videoAudioDevice;
      }
      createInstance(edu.tufts.hrilab.ava.AvaDisplayComponent.class, displayArgs);
    }
    if (useMockAva) {
      createInstance(edu.tufts.hrilab.ava.MockAvaComponent.class, avaArgs);
    } else {
      if (!speakerId.isEmpty()) {
        avaArgs += " -matchMixer " + speakerId;
      }
      createInstance(edu.tufts.hrilab.ava.AvaComponent.class, avaArgs);
    }

    String kioskArgs = "";
    if (initKiosk) {
      kioskArgs += " -enabled -timeout 2700";
    }
    if (persistKiosk) {
      kioskArgs += " -kioskSettingsFile ava.ser";
    }

    serviceProvider = createInstance(SocialTRLLMServiceProvider.class,"");
    String llmComponentArgs = String.format("-service %s hf -model %s Qwen/Qwen3.5-9B -temperature 0.5 0.5 -endpoint %s", service, model, openaiEndpoint);
    //HF only
    //String llmComponentArgs = String.format("-service hf -model Qwen/Qwen3-14B -temperature 0.7", service, model);
    //Marvin only
    //String llmComponentArgs = String.format("-service %s -model %s -endpoint %s -temperature 0.7", service, model, openaiEndpoint);
    createInstance(edu.tufts.hrilab.llm.LLMComponent.class, llmComponentArgs);

    createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class,
            "-historylength 3600000 "
                    +   "-goal listen(self) monitorBattery(ava) listenForWWandASR(ava) " +
                    "-beliefinitfile demos.pl agents/ava.pl domains/ava.pl"
                    + " -dbfile " +
                    "core.asl " +
                    "manipulation.asl " +
                    "dialogue/nlg.asl " +
                    "dialogue/socialLLMnlu.asl " +
                    "dialogue/handleSemantics.asl " +
                    "vision.asl " +
                    "ava/ava.asl " +
                    "ava/avaIntrospection.asl " +
                    "ava/socialAvaIntrospection.asl " +
                    "ava/scheduledGoals.asl " +
                    "ava/avaMedia.asl " +
                    "ava/avaASR.asl " +
                    "ava/avaTMCDemo.asl " +
                    "ava/wifiScan.asl " +
                    "ava/tempScan.asl " +
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


