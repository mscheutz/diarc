/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * MaryTTSComponent implementation class for the TTS server.
 *
 * @author: Cody Canning; cody.canning@tufts.edu
 */
package edu.tufts.hrilab.tts;

import java.io.*;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.interfaces.VoiceProsodyInterface;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.util.resource.Resources;
import marytts.util.MaryUtils;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.io.IOUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Properties;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import javax.sound.sampled.*;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import marytts.datatypes.MaryDataType;
import marytts.modules.synthesis.Voice;
import marytts.server.Mary;
import marytts.server.Request;
import marytts.util.data.audio.AudioPlayer;
import org.apache.log4j.Level;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;

public final class MaryTTSComponent extends DiarcComponent implements SpeechProductionInterface, VoiceProsodyInterface {

  private MaryDataType inputType;//.TEXT or .RAWMARYXML
  private MaryDataType outputType;
  private Voice voice;
  private String defaultEffects;
  private String defaultStyle;
  private Locale locale;
  private int id;
  private AudioFileFormat audioFileFormat;
  private AudioInputStream ais;
  private AudioPlayer ap;
  private Request request;
  private BufferedReader br;

  private boolean useGui = false;
  private MaryTTSComponentVis gui;

  /**
   * The configuration file from which to load canned-phrases.
   */
  private String config;
  private String defaultConfigPath = "config/edu/tufts/hrilab/tts";
  private Document doc;
  private DocumentBuilder db;
  private Transformer tr;
  private String voiceStr;    // default to female voice


  private enum Emotion {
    STRESS, CONFUSION, ANGER, CUSTOM1, NONE
  }

  private Emotion e;

  // boolean markers set by args flags
  private boolean saveToWav;
  private String markup;

  // if saving to a .wav file, save at this location
  private String wavFilename;
  private boolean speaking;

  /**
   * Map from unknown token (i.e., newly learned word) to wave file of the word.
   */
  private Map<String, byte[]> newWordMap;
  /**
   * Executor that executes sayText and playWavFile jobs.
   */
  private ExecutorService executor;


  //Brad: this is an awful hack to get around MaryTTS not using logging appropriately
  public class MyProperties extends Properties {

    public MyProperties(Properties properties) {
      super(properties);
    }

    @Override
    public Object setProperty(String key, String value) {
      //catch the overried call in the Mary TTS jar and while we're in the right scope turn off all of their logging
      if ("log4j.defaultInitOverride".equals(key)) {
        MaryUtils.getLogger("main").getParent().setLevel(Level.FATAL);
        return null;
      } else {
        return super.setProperty(key, value);
      }
    }
  }


  /**
   * MaryTTSComponent constructor. Initializes TTS component.
   */
  public MaryTTSComponent() throws IOException, UnsupportedAudioFileException, InterruptedException {
    super();

    outputType = MaryDataType.AUDIO;
    locale = new Locale("en-US");
    defaultEffects = null;
    defaultStyle = null;
    id = 0;

    audioFileFormat = new AudioFileFormat(AudioFileFormat.Type.WAVE, Voice.AF16000, AudioSystem.NOT_SPECIFIED);
    //audioFileFormat = new AudioFileFormat(AudioFileFormat.Type.WAVE, Voice.AF22050, AudioSystem.NOT_SPECIFIED);

    // initialize components necessary for building and transforming XML doc later
    try {
      DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
      db = dbf.newDocumentBuilder();
      TransformerFactory transformerFactory = TransformerFactory.newInstance();
      this.tr = transformerFactory.newTransformer();
    } catch (Exception ex) {
      log.error("exception initializing MaryTTS", ex);
    }

    e = Emotion.NONE;
    config = "default.tts";
    saveToWav = false;
    markup = "SSML";
    wavFilename = "/tmp/diarcplay.wav";
    speaking = false;

    // default to male voice
    //voiceStr = "cmu-rms=hsmm"
    voiceStr = "cmu-slt-hsmm";

    newWordMap = new HashMap<>();
//    newWordMap.put("unk0", null);
//    newWordMap.put("unk1", null);

    executor = Executors.newSingleThreadExecutor();
  }

  @Override
  protected void init() {
    try {
      if (Mary.currentState() == Mary.STATE_OFF) {
        MyProperties newProps = new MyProperties(System.getProperties());
        System.setProperties(newProps);
        Mary.startup();
        System.setProperty("log4j.defaultInitOverride", "true");
      }
    } catch (Exception ex) {
      log.error("error stating Marytts", ex);
    }

    setVoice(voiceStr);

    if (useGui) {
      gui = new MaryTTSComponentVis(this);
    }
  }

  /**
   * Checks if speech is being produced.
   *
   * @return {@code true} if speech is being produced, {@code false} otherwise
   */
  @Override
  public boolean isSpeaking() {
    return speaking;
  }

  /**
   * Stops an ongoing utterance.
   *
   * @return {@code true} if speech is interrupted, {@code false} otherwise.
   */
  @Override
  public boolean stopUtterance() {
    ap.interrupt();
    return ap.isInterrupted();
  }

  /**
   * sets prosody to STRESS ANGER or CONFUSION
   *
   * @param newEmo the new prosody to be used
   */
  @Override
  public void setEmotion(String newEmo) {
    for (Emotion emo : Emotion.values()) {
      if (emo.toString().equalsIgnoreCase(newEmo)) {
        this.e = emo;
      }
    }
    log.debug("Applying " + this.e.toString());

  }

  /**
   * returns the current vocal emotion
   */
  @Override
  public String getEmotion() {
    return this.e.toString();
  }

  /**
   * @return the current voice used for TTS
   */
  @Override
  public String getVoice() {
    return this.voice.toString();
  }

  /**
   * @param v name of the voice or male or female
   */
  @Override
  public void setVoice(String v) {
    String vc = v;

    if (v.equalsIgnoreCase("male")) {
      vc = "cmu-rms-hsmm";
    } else if (v.equalsIgnoreCase("female")) {
      vc = "cmu-slt-hsmm";
    }

    voice = Voice.getVoice(vc);
  }

  @Override
  public boolean sayText(String text) {
    sayText(text, true);
    return true;
  }

  /**
   * Speaks appropriate text
   *
   * @param text the text to be spoken
   * @param wait whether or not to block until speaking call returns
   * @return true if the text was generated as speech, false otherwise
   */
  @Override
  public boolean sayText(String text, boolean wait) {
    final String finalText = text;

    Future<Boolean> future = executor.submit(new Callable<Boolean>() {

      @Override
      public Boolean call() throws Exception {

        // split on new words
        List<String> splitText = splitOnNewWords(finalText);
        log.info("words", splitText);

        boolean result = true;
        for (String t : splitText) {
          if (newWordMap.containsKey(t)) {
            result &= playWavFile(newWordMap.get(t));
          } else {
            result &= textToSpeech(t, true);
          }
        }
        return result;
      }
    });

    if (wait) {
      try {
        log.debug("Waiting for future to finish.");
        Boolean future_result = future.get();
        log.debug("Future finished.");
        return future_result;
      } catch (InterruptedException | ExecutionException e) {
        log.error("Error waiting on sayText to finish.", e);
      }
    }
    return true;
  }

  private boolean textToSpeech(String text, boolean wait) {
    speaking = true;
    doc = db.newDocument();

    // GB: add punctuation if none-exists, default to '.'
    if (!(text.endsWith(".") || text.endsWith(",") || text.endsWith("?") || text.endsWith("!"))) {
      text += ".";
    }

    try {
      // search for words in ALLCAPS and wrap them with emphasis tags
      String emphUtt = addEmphasis(text);
      // apply appropriate emotion (including NONE)
      applyEmotion(emphUtt);

      request = new Request(inputType,
              outputType,
              locale,
              voice,
              defaultEffects,
              defaultStyle,
              id,
              audioFileFormat);

      request.readInputData(br);
      request.process();
      ais = request.getOutputData().getAudio();

      // either save the audio to .wav or output it as audio
      if (saveToWav) {
        File f = new File(wavFilename);
        AudioSystem.write(ais, audioFileFormat.getType(), f);
      } else {
        ap = new AudioPlayer(ais);
        ap.start();
        if (wait) {
          ap.join();
        }
      }
      log.debug("Exiting textToSpeech");
      speaking = false;
      return true;

    } catch (Exception ex) {
      log.error("exception in textToSpeech", ex);
      speaking = false;
      return false;
    }
  }

  @TRADEService
  public String getConfigFile() {
    return Resources.createFilepath(defaultConfigPath, config);
  }

  @TRADEService
  public void addNewWord(String word, byte[] pcmData) {
    log.debug("adding new word: " + word);
    newWordMap.put(word, pcmData);
  }

  /**
   * Split on new words in the newWordsMap.
   *
   * @return
   */
  private List<String> splitOnNewWords(String text) {
    List<String> splitText = new LinkedList<>();
    splitText.add(text.trim());
    for (String newWord : newWordMap.keySet()) {
      for (int i = 0; i < splitText.size(); ++i) {
        String t = splitText.get(i);
        List<String> tempSplit = splitOnNewWordsHelper(t, newWord);
        splitText.remove(i);
        splitText.addAll(i, tempSplit);
        i += tempSplit.size();
      }
    }
    return splitText;
  }

  /**
   * Helper method to split text on newWordMap.
   *
   * @param text
   * @param splitter
   * @return
   */
  private List<String> splitOnNewWordsHelper(String text, String splitter) {
    List<String> splitText = new ArrayList<>();

    if (text.equals(splitter)) {
      // special case
      splitText.add(splitter);
    } else {
      for (String s : text.split(splitter)) {
        if (!s.trim().isEmpty()) {
          splitText.add(s.trim());
        }
        splitText.add(splitter.trim());
      }
      if (splitText.size() > 1 && !text.endsWith(splitter)) {
        splitText.remove(splitText.size() - 1);
      }
    }
    return splitText;
  }

  /**
   * TODO: write this method
   *
   * @param data
   * @return
   */
  private boolean playWavFile(byte[] data) {
    if (data == null) {
      return false;
    }

    AudioFormat audioFormat = new AudioFormat(16000, 16, 1, true, true);

    Clip playbackClip;
    try {
      playbackClip = AudioSystem.getClip();
      playbackClip.open(audioFormat, data, 0, data.length);
      playbackClip.start();
      // TODO: change this to listen to the STOP event
      while (playbackClip.isRunning()) {
        Util.Sleep(100);
      }
    } catch (LineUnavailableException e1) {
      e1.printStackTrace();
    }

    return true;
  }

  /**
   * kind of a hack... When adding emphasis tags to utterance they aren't
   * escaped by the XML transformer. this function fixes the opening and closing
   * XML brackets.
   *
   * @param fname file name to fix XML tags of
   * @throws IOException
   */
  private void fixXML(String fname) throws IOException {
    FileInputStream fis = new FileInputStream(fname);
    String content = IOUtils.toString(fis);
    content = content.replace("&lt;", "<");
    content = content.replace("&gt;", ">");
    fis.close();
    FileOutputStream fos = new FileOutputStream(fname);
    IOUtils.write(content, fos);
    fos.close();
  }

  /**
   * generate utterance with RAWMARYXML markup. populates doc with MARYXML
   * header and paragraph tag
   *
   * @return the child element
   * @throws ParserConfigurationException
   */
  private Element createRAWMARYXMLDoc() throws ParserConfigurationException {
    inputType = MaryDataType.RAWMARYXML;

    // create elements
    Element maryEl = doc.createElement("maryxml");
    maryEl.setAttribute("version", "0.4");
    maryEl.setAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    maryEl.setAttribute("xmlns", "http://mary.dfki.de/2002/MaryXML");
    maryEl.setAttribute("xml:lang", "en-US");
    doc.appendChild(maryEl);
    Element pEl = doc.createElement("p");
    maryEl.appendChild(pEl);
    return pEl;
  }

  /**
   * generate utterance with SSML markup. populates doc with SSML header and
   * paragraph tag
   *
   * @return the child element
   * @throws ParserConfigurationException
   */
  private Element createSSMLDoc() throws ParserConfigurationException {
    inputType = MaryDataType.SSML;

    // create elements
    Element maryEl = doc.createElement("speak");
    maryEl.setAttribute("version", "1.0");
    maryEl.setAttribute("xmlns", "http://www.w3.org/2001/10/synthesis");
    maryEl.setAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    maryEl.setAttribute("xsi:schemaLocation", "http://www.w3.org/2001/10/synthesis "
            + "http://www.w3.org/TR/speech-synthesis/synthesis.xsd");
    maryEl.setAttribute("xml:lang", "en-US");
    doc.appendChild(maryEl);
    Element pEl = doc.createElement("p");
    maryEl.appendChild(pEl);
    return pEl;
  }

  /**
   * applies Emotion e to input s by wrapping the utterance in Mary XML
   *
   * @param s the string we're applying emotion to
   */
  private void applyEmotion(String s) {
    try {
      Element prosodyEl = doc.createElement("prosody");

      // generate emotive markup
      switch (this.e) {
        case STRESS: // nervous, stressed, fearful
          prosodyEl.setAttribute("contour", "(0%,+3st)(10%,+3st)(20%,+3st)(30%,+10st)(40%,+4st)(50%,+4st)(60%,+4st)(70%,+9st)(80%,+7st)(90%,+10st)(100%,+11st)");
          prosodyEl.setAttribute("rate", "1.15");
          break;
        case ANGER: // angry, frustrated
          prosodyEl.setAttribute("contour", "(0%,-2st)(10%,-2st)(20%,-2st)(30%,-2st)(40%,-2st)(50%,-2st)(60%,-3st)(70%,-3st)(80%,-3st)(90%,-4st)(100%,-4st)");
          prosodyEl.setAttribute("rate", "0.82");
          break;
        case CONFUSION: // confused, puzzled
          prosodyEl.setAttribute("contour", "(0%,-1st)(10%,-1st)(20%,-1st)(30%,-1st)(40%,-1st)(50%,-1st)(60%,-2st)(70%,+3st)(80%,+3st)(90%,+10st)(100%,+6st)");
          prosodyEl.setAttribute("rate", "0.85");
          prosodyEl.setAttribute("volume", "0.0");
          break;
        case CUSTOM1:
          break;
        default:
          break;
      }

      // append the utterance with prosody markup to doc.
      Node utt = doc.createTextNode(s);

      log.debug("Utterance", utt);

      prosodyEl.appendChild(utt);
      if (this.markup.equalsIgnoreCase("SSML")) {
        createSSMLDoc().appendChild(prosodyEl);
      } else {
        createRAWMARYXMLDoc().appendChild(prosodyEl);
      }

      // transform the document to XML
      DOMSource source = new DOMSource(doc);
      StreamResult result = new StreamResult(new File("utterance.xml"));
      this.tr.transform(source, result);

      fixXML("utterance.xml");

      // delegate datastream
      FileInputStream fstream = new FileInputStream("utterance.xml");
      DataInputStream in = new DataInputStream(fstream);
      this.br = new BufferedReader(new InputStreamReader(in));

    } catch (Exception ex) {
      log.error("exception in applyEmotion", ex);
    }
  }

  /**
   * adds emphasis to uppercase words in String s
   *
   * @param utterance the utterance text we are searching for ALLCAPS words and
   *                  emphasizing
   * @return the updated String including emphasis
   */
  private String addEmphasis(String utterance) {
    String[] words = utterance.split("\\s+");
    String newUtterance = "";
    for (int i = 0; i < words.length; i++) {
      if (isAllUppercase(words[i])) {
        words[i] = "<emphasis level=\"strong\">" + words[i] + "</emphasis>";
      }
      newUtterance = newUtterance + " " + words[i];
    }
    return newUtterance;
  }

  /**
   * *
   * Returns true if string s is all uppercase, false otherwise
   *
   * @param s the string to check
   * @return true or false
   */
  private boolean isAllUppercase(String s) {
    boolean allUpper = true;
    for (int i = 0; i < s.length(); i++) {
      if (Character.isLowerCase(s.charAt(i))) {
        allUpper = false;
        break;
      }
    }
    return allUpper;
  }

  public String getGuiHelp() {
    return "sayText: enter any string you want to say\n"
            + "sayText (with boolean): enter any string you want to say, with blocking true/false\n"
            + "isSpeaking: is component currently speaking\n"
            + "stopUtterance: cancel utterance from being said\n"
            + "setEmotion: set either \"stress\", \"anger\", or \"confusion\""
            + " (without quotes).\n"
            + "getEmotion: get currently set emotion\n"
            + "setVoice: specify voice to use by name. Or by gender (\"female\" or \"male\").\n"
            + "getVoice: get the current voice being used";
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("wav").desc("save to wav play instead of sending directly to audioout").build());
    options.add(Option.builder("stress").desc("apply stressed prosody to an utterance").build());
    options.add(Option.builder("anger").desc("apply angry, frustrated prosody to an utterance").build());
    options.add(Option.builder("confusion").desc("apply confused, perplexed prosody to an utterance").build());
    options.add(Option.builder("male").desc("use male voice").build());
    options.add(Option.builder("female").desc("use female voice").build());
    options.add(Option.builder("config").hasArg().argName("file").desc("set configuration file").build());
    options.add(Option.builder("gui").desc("create gui").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("wav")) {
      log.debug("found WAV, saving to file");
      saveToWav = true;
    }
    if (cmdLine.hasOption("stress")) {
      log.debug("setting STRESS");
      this.e = Emotion.STRESS;
    }
    if (cmdLine.hasOption("anger")) {
      log.debug("setting ANGER");
      this.e = Emotion.ANGER;
    }
    if (cmdLine.hasOption("confusion")) {
      log.debug("setting CONFUSION");
      this.e = Emotion.CONFUSION;
    }
    if (cmdLine.hasOption("male")) {
      log.debug("setting MALE");
      voiceStr = "male";
    }
    if (cmdLine.hasOption("female")) {
      log.debug("setting FEMALE");
      voiceStr = "female";
    }
    if (cmdLine.hasOption("config")) {
      config = cmdLine.getOptionValue("config");
    }
    if (cmdLine.hasOption("gui")) {
      useGui = true;
    }
  }

}
