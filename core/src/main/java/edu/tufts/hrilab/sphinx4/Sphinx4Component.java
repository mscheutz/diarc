/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.sphinx4;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.cmu.sphinx.api.SpeechResult;
import edu.cmu.sphinx.api.SphinxRecognizer;
import edu.cmu.sphinx.api.Configuration;

import java.util.*;
import java.util.concurrent.LinkedBlockingQueue;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.cmu.sphinx.result.WordResult;

import edu.tufts.hrilab.sphinx4.gui.Sphinx4GUIPanel;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Mixer;

/**
 * The implementation of an DIARCComponent with the Sphinx4 speech recognizer.
 */
public class Sphinx4Component extends DiarcComponent implements Sphinx4Interface {

  private String recognizerConfig;
  private Configuration configuration;
  private SphinxRecognizer recognizer;
  private RecordLoop recLoop = null;
  private ProcessingLoop processLoop = null;

  private Queue<SpeechResult> speechResults = new LinkedList<>();

  boolean useDefaultConfig;
  private final String defaultRecognizerConfig = "resource:/config/edu/tufts/hrilab/sphinx4/defaultConfig.xml";

  private final String defaultAcousticModel = "resource:/edu/cmu/sphinx/models/en-us/en-us";
  private final String defaultDictionary = "resource:/edu/cmu/sphinx/models/en-us/cmudict-en-us.dict";
  private final String defaultLanguageModel = "resource:/edu/cmu/sphinx/models/en-us/en-us.lm.bin";
  private boolean useGrammar;
  private String grammarName;

  private boolean selectMixer;
  private int mixerID;

  private boolean shouldRecord;
  private boolean tempIgnore;

  //  //this holds the most recently recognized text
//  private final Object mostRecentTextLock = new Object();
  private String mostRecentText;
//  private boolean recievedResult;

  public enum ControlState {
    ACCEPT, CONFIRM, REJECT
  }

  private ControlState controlState;
  private final Queue<String> recognizedQ;

  private boolean useOOV;
  private int segmentCount;

  private Lock recognizedQEmptyLock;
  private Condition recognizedQNotEmpty;

  private Symbol speaker;
  private Symbol listener;

  private boolean useGui = false;
  private Sphinx4GUIPanel gui;

  /**
   * Constructor.
   *
   * @see #additionalUsageInfo for command line arguments.
   */
  public Sphinx4Component() {
    super();

    shouldRecord = false;
    tempIgnore = true;
    mostRecentText = "";

    // default accept all state
    controlState = ControlState.ACCEPT;

    useDefaultConfig = true;
    useGrammar = false;
    selectMixer = false;
    mixerID = -1;

    recognizedQ = new LinkedBlockingQueue<>();

    speaker = new Symbol("CommX");
    listener = new Symbol("self");
  }

  protected void init() {

    recognizedQEmptyLock = new ReentrantLock();
    recognizedQNotEmpty = recognizedQEmptyLock.newCondition();

    segmentCount = 0;

    configuration = new Configuration();

    // Set path to acoustic model.
    configuration.setAcousticModelPath(defaultAcousticModel);
    // Set path to dictionary.
    configuration.setDictionaryPath(defaultDictionary);


    if (useGrammar) {
      configuration.setGrammarPath("resource:/config/edu/tufts/hrilab/sphinx4/");
      configuration.setGrammarName(grammarName);
      configuration.setUseGrammar(true);
    } else {
      configuration.setUseGrammar(false);
      //     Set language model.
      configuration.setLanguageModelPath(defaultLanguageModel);
    }

    if (useDefaultConfig) {
      recognizerConfig = defaultRecognizerConfig;
    }

    try {
      log.debug("recognizer config " + recognizerConfig);
      recognizer = new SphinxRecognizer(recognizerConfig, configuration, selectMixer, mixerID);
    } catch (Exception e) {
      log.error("unable to load SphinxRecognizer.", e);
    }

    //Start the recording
    shouldRecord = true;
    (recLoop = new RecordLoop()).start();
    (processLoop = new ProcessingLoop()).start();

    if (useGui) {
      gui = new Sphinx4GUIPanel(this);
    }

    log.info("Sphinx recording started.");
  }

  public void setConfig(String path) {
    recognizerConfig = "resource:/config/edu/tufts/hrilab/sphinx4/" + path;
  }

  public void setGrammar(String path) {
    grammarName = path;
    //TODO: might want to make this more robust
    if (path.contains(".")) {
      String[] split = path.split(".");
      grammarName = split[0];
    }
  }

  public void setMixer(String ID) {

    int id = Integer.valueOf(ID);
    Mixer.Info[] mixers = AudioSystem.getMixerInfo();

    if (id > mixers.length || id < 0) {
      log.error("Invalid mixer Id: " + id + " Total mixer count: " + mixers.length + " Using Default mixer instead.");
    } else {
      log.info("Selected mixer:" + mixers[id].getDescription());
      selectMixer = true;
      mixerID = id;
    }
  }

  public void showMixerOptions() {
    Mixer.Info[] mixers = AudioSystem.getMixerInfo();

    for (int i = 0; i < mixers.length; ++i) {
      log.info("Mixer: " + i + " : " + mixers[i]);
    }
  }

  /**
   * This thread will capture audio from the microphone and do recognition with
   * Sphinx4.
   */
  class RecordLoop extends Thread {

    @Override
    public void run() {
      recognizer.startRecognition(true);
      while (shouldRecord) {
        SpeechResult mostRecentResult = recognizer.getResult();
        log.debug("record loop: " + mostRecentResult.getHypothesis());
        speechResults.add(mostRecentResult);
        recognizedQEmptyLock.lock();
        try {
          recognizedQNotEmpty.signal();
        } finally {
          recognizedQEmptyLock.unlock();
        }
      }
      recognizer.stopRecognition();
    }

    public void halt() {
      log.debug("halting the recordLoop");
      shouldRecord = false;
      if (recognizer != null) {
        recognizer.stopRecognition();
      }
    }

  }

  class ProcessingLoop extends Thread {

    @Override
    public void run() {
      while (shouldRecord) {
        // pop stuff of the recogntion queue and send it out

        //TODO: split this up into recognition and processing threads
        //processing thread takes strings and bytes from queue and outputs them appropriately
        //need to make asr.inputaudio return the classifcation label
        if (tempIgnore) {
          if (!speechResults.isEmpty()) {
            log.debug("Ignoring first utterance, typically from noise");
            speechResults.clear();
            tempIgnore = false;
          }
        }

        if (speechResults.isEmpty()) {
          recognizedQEmptyLock.lock();
          try {
            try {
              recognizedQNotEmpty.await();
            } catch (Exception e) {
              log.error("Exception while waiting", e);
            }
          } finally {
            recognizedQEmptyLock.unlock();
          }
        } else {

          SpeechResult nextResult = speechResults.remove();

          mostRecentText = nextResult.getHypothesis().trim();

          List<WordResult> words = nextResult.getWords();

//          for (WordResult w : words) {
//            if (w.getWord().toString().equals("<unk>") && useOOV) {
//              long start = w.getTimeFrame().getStart();
//              long end = w.getTimeFrame().getEnd();
//
//              byte[] audioData = recognizer.getAudio(segmentCount, start, end);
////            try {
////              FileOutputStream fos = new FileOutputStream("testUnk.raw");
////              fos.write(audioData);
////              fos.close();
////            } catch (IOException e) {
////              log.error(e);
////            }
//              log.debug("audio data length: " + audioData.length);
//              try {
//                mostRecentText = (String) TRADE.callThe("inputAudio", audioData);
//              } catch (TRADEException e) {
//                log.warn("Failed to call understandUtterance service");
//              }
//            }
//            log.debug(w.toString());
//          }

//        if (!recievedResult) {
//          recievedResult = true;
//        }
          log.info("Recognized hypothesis: " + mostRecentText);

          switch (controlState) {
            case ACCEPT:
                Utterance utterance = new Utterance.Builder()
                        .setSpeaker(speaker)
                        .addListener(listener)
                        .setWords(Arrays.asList(mostRecentText.split(" ")))
                        .setUtteranceType(UtteranceType.UNKNOWN)
                        .setIsInputUtterance(true)
                        .build();

                try {
                  TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
                } catch (TRADEException e) {
                  log.warn("Failed to call reportRecognizedSpeech service", e);
                }
              break;
            case CONFIRM:
              recognizedQ.add(mostRecentText);

              // notify gui of recognized text
              if (useGui) {
                gui.notifyRecognizedQueueAddition();
              }
              break;
            case REJECT:
              break;
          }
          segmentCount++;
        }
      }
    }

    public void halt() {
      log.debug("halting the processingLoop");
      shouldRecord = false;
    }

  }

  /**
   * Parses command line arguments specific to this DIARCComponent.
   *
   * @param cmdLine The custom command line arguments
   * {@code false} otherwise
   */
  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("grammar")) {
      try {
        setGrammar(cmdLine.getOptionValue("grammar"));
      } catch (Exception e) {
        log.error("unable to call set args", e);
      }
      useGrammar = true;
    }
    if (cmdLine.hasOption("oov")) {
      useOOV = true;
    }
    if (cmdLine.hasOption("config")) {
      try {
        setConfig(cmdLine.getOptionValue("config"));
      } catch (Exception e) {
        log.error("unable to call set config", e);
      }
      useDefaultConfig = false;
    }
    if (cmdLine.hasOption("showmixers")) {
      showMixerOptions();
    }
    if (cmdLine.hasOption("mixer")) {
      try {
        setMixer(cmdLine.getOptionValue("mixer"));
      } catch (Exception e) {
        log.error("unable to call set config", e);
      }
    }
    if (cmdLine.hasOption("control")) {
      String controlLevel = cmdLine.getOptionValue("control");
      try {
        if (controlLevel.equalsIgnoreCase("a")) {
          controlState = ControlState.ACCEPT;
        } else if (controlLevel.equalsIgnoreCase("c")) {
          controlState = ControlState.CONFIRM;
        } else if (controlLevel.equalsIgnoreCase("r")) {
          controlState = ControlState.REJECT;
        }
      } catch (Exception e) {
        log.error("no control level found, using Accept", e);
      }
    }
    if (cmdLine.hasOption("speaker")) {
      speaker = new Symbol(cmdLine.getOptionValue("speaker"));
    }
    if (cmdLine.hasOption("listener")) {
      listener = new Symbol(cmdLine.getOptionValue("listener"));
    }
    if (cmdLine.hasOption("gui")) {
      useGui = true;
    }
  }

  /**
   * Provides command line argument descriptions.
   *
   * @return Command line argument switches and descriptions
   */
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("grammar").hasArg().argName("path").desc("set grammar. File must be located in /resources/config/edu/tufts/hrilab/sphinx4/").build());
    options.add(Option.builder("nlp").desc("use nlp").build());
    options.add(Option.builder("oov").desc("use ASR for OOV detection").build());
    options.add(Option.builder("config").hasArg().argName("path").desc("set relative path to Sphinx4 config file (if not specified, will use a default config)").build());
    options.add(Option.builder("control").hasArg().argName("a/c/r").desc("set whether the component accepts, waits for confirmation or rejects").build());
    options.add(Option.builder("speaker").hasArg().argName("name").desc("set speaker name").build());
    options.add(Option.builder("listener").hasArg().argName("name").desc("set listener name").build());
    options.add(Option.builder("mixer").hasArg().argName("id").desc("set mixer id").build());
    options.add(Option.builder("showmixers").desc("Display all mixer options.").build());
    options.add(Option.builder("gui").desc("Display GUI.").build());
    return options;
  }

  @Override
  public void setControlState(Sphinx4Component.ControlState state) {
    controlState = state;
  }

  @Override
  public Sphinx4Component.ControlState getControlState() {
    return controlState;
  }

  @Override
  public void acceptUtterance() {
    String utt = recognizedQ.remove().trim();

    Utterance utterance = new Utterance.Builder()
            .setSpeaker(speaker)
            .addListener(listener)
            .setWords(Arrays.asList(utt.split(" ")))
            .setUtteranceType(UtteranceType.UNKNOWN)
            .setIsInputUtterance(true)
            .build();

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
    } catch (TRADEException e) {
      log.warn("Failed to call reportRecognizedSpeech service", e);
    }
    log.info("accepted utterance " + utt);
  }

  @Override
  public void rejectUtterance() {
    if (!recognizedQ.isEmpty()) {
      String utt = recognizedQ.remove();
      log.info("rejected utterance " + utt);
    }
  }

  @Override
  public String getOnDeckText() {
    if (!recognizedQ.isEmpty()) {
      String ret = recognizedQ.peek();
      return ret;
    } else {
      return "";
    }
  }

  @Override
  public void clearRecogntionHistory() {
    recognizedQ.clear();
  }

}
