/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.asr.whisper;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.util.Audio;
import edu.tufts.hrilab.util.resource.Resources;

import edu.tufts.hrilab.asr.ASRInterface;
import edu.tufts.hrilab.asr.RealTimeAudioBuffer;

import java.io.*;
import java.net.*;
import java.nio.file.Paths;
import java.util.*;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import io.github.givimad.whisperjni.*;
import org.apache.commons.io.FileUtils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.sound.sampled.*;
/**
 * The implementation of an DIARCComponent with the Whisper speech recognizer.
 */
public class WhisperComponent extends DiarcComponent implements ASRInterface {
  private static String whisperDirectoryPath = "config/edu/tufts/hrilab/asr/whisper";
  private String libwhisper = null; // default on Linux /usr/local/lib/libwhisper.so
  private String modelName = "base";
  private WhisperModel model;
  private WhisperJNI whisper;
  private WhisperContext context;

  // Microphone config
  private ConfigurableMicrophone microphone;
  final private float samplesPerSec = 16000.0f; // 16k samples/second
  final private int sampleSize = 16; // 16 bits (2 bytes)
  final private int sampleSizeInBytes = sampleSize/8;
  final private boolean isBigEndian = false;
  final private boolean isSigned = true;

  private InputStream inputStream = null;
  private RealTimeAudioBuffer audioBuffer;
  private ExecutorService recordingThread;
  private ExecutorService processingThread;
  private String speakerStr = "CommX";
  private String addresseeStr = "self";
  private Symbol speaker;
  private Symbol addressee;
  private URL grammarPath = null;
  private WhisperGrammar grammar = null;
  private float grammarPenalty = 10f;
  private int mixer = 1;
  private boolean showMixers = false;
  private boolean useMicrophone = true;
  private boolean saving = false;
  private boolean enabled = true;
  private boolean useDisplay = false;

  // Transcribing variables
  final private double bufferDuration = 0.1; // 0.1 s
  final private int bufferLength = (int) (samplesPerSec * bufferDuration * sampleSizeInBytes);
  private boolean speechDetected = false;
  private double threshold = 200.0; // Loudness level to be considered speech to the robot
  private int silenceCounter = 0; // Counting how much silent audio clips received
  private float[] microphoneData = new float[(int) (samplesPerSec * sampleSizeInBytes * 30)]; // max 30s
  private int microphoneCursor = 0;

  private ExecutorService executor = Executors.newSingleThreadExecutor();

//  private CircularByteBuffer cbb = new CircularByteBuffer(bufferLength * 2); // 2 samples

  @Override
  protected List<Option> additionalUsageInfo() {

    List<Option> options = new ArrayList<>();
    options.add(Option.builder("model").hasArg().argName("name").desc("Whisper GGML model to use (default: " + modelName + ")").build());
    options.add(Option.builder("lib").hasArg().argName("path").desc("Whisper so/dylib to use").build());
    options.add(Option.builder("speaker").hasArg().argName("name").desc("Set speaker name (default: " + speakerStr + ")").build());
    options.add(Option.builder("addressee").hasArg().argName("name").desc("Set addressee name (default: " + addresseeStr + ")").build());
    options.add(Option.builder("grammar").hasArg().argName("file").desc("GBNF grammar file").build());
    options.add(Option.builder("grammarPenalty").hasArg().argName("float").desc("Penalty for grammar setting (default: " + grammarPenalty + ")").build());
    options.add(Option.builder("showMixers").argName("boolean").desc("Show all mixers").build());
    options.add(Option.builder("matchMixer").hasArgs().argName("name").desc("String to match for mixer in case of device re-ordering. Do not quote argument if spaces needed.").build());
    options.add(Option.builder("mixer").hasArg().argName("id").desc("ID of mixer to select for microphone").build());
    options.add(Option.builder("noMicrophone").argName("boolean").desc("Do not enable a local microphone").build());
    options.add(Option.builder("threshold").hasArg().argName("double").desc("Threshold of mic level to use to determine presence of speech (default: " + threshold + ")").build());
    options.add(Option.builder("save").argName("boolean").desc("Save temporary files of audio input for evaluation").build());
    options.add(Option.builder("useDisplay").argName("boolean").desc("Use display to display transcribing status").build());
    return options;
  }
  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("lib")) {
      libwhisper = cmdLine.getOptionValue("lib");
    }
    log.debug("Using libwhisper " + libwhisper);

    if (cmdLine.hasOption("model")) {
      modelName = cmdLine.getOptionValue("model");
    }
    log.debug("Using model " + modelName);

    if (cmdLine.hasOption("grammar")) {
      grammarPath = Resources.getResource(whisperDirectoryPath, cmdLine.getOptionValue("grammar"));
      log.debug("Using grammar " + grammarPath.toString());
    }
    log.debug("Using whisper model " + modelName);

    if (cmdLine.hasOption("grammarPenalty")) {
      grammarPenalty = Float.valueOf(cmdLine.getOptionValue("grammarPenalty"));
      log.debug("grammarPenalty = " + grammarPenalty);
    }

    if (cmdLine.hasOption("speaker")) {
      speakerStr = cmdLine.getOptionValue("speaker");
    }
    speaker = new Symbol(speakerStr);
    log.debug("speaker = " + speakerStr);

    if (cmdLine.hasOption("addressee")) {
      addresseeStr = cmdLine.getOptionValue("addressee");
    }
    addressee = new Symbol(addresseeStr);
    log.debug("addressee = " + addresseeStr);

    if (cmdLine.hasOption("mixer")) {
      mixer = Integer.parseInt(cmdLine.getOptionValue("mixer"));
    } else if (cmdLine.hasOption("matchMixer")) {
      String matchString = String.join(" ", cmdLine.getOptionValues("matchMixer"));
      mixer = ConfigurableMicrophone.matchMixer(matchString);
      log.debug("Matching mixer string \"" + matchString + "\"");
    }
    log.debug("Using mixer " + mixer);

    if (cmdLine.hasOption("showMixers")) {
      showMixers = true;
    }

    if (cmdLine.hasOption("noMicrophone")) {
      useMicrophone = false;
      log.debug("Not using a local microphone");
    }

    if (cmdLine.hasOption("threshold")) {
      threshold = Double.parseDouble(cmdLine.getOptionValue("threshold"));
    }
    log.debug("Using threshold " + threshold);

    if (cmdLine.hasOption("save")) {
      saving = true;
      log.debug("Saving temporary audio files");
    }

    if (cmdLine.hasOption("useDisplay")) {
      useDisplay = true;
      log.debug("Using display");
    }
  }

  protected void init() {
    WhisperJNI.LoadOptions loadOptions = new WhisperJNI.LoadOptions();
    loadOptions.logger = log::debug;
    String grammarAbsolutePath = null;
    URI grammarURI = null;
    if (libwhisper != null) {
      loadOptions.whisperLib = Paths.get(libwhisper);
    }
    try {
      WhisperJNI.loadLibrary(loadOptions);
    } catch (IOException ex) {
      log.error("Error loading options");
      return;
    }
    try {
      whisper = new WhisperJNI();
    } catch (UnsatisfiedLinkError ex) {
      log.error("libwhisper not linking properly. Do you have whisper.cpp installed and built locally?", ex);
      return;
    }
    if (grammarPath != null) {
      grammarAbsolutePath = fileToTemp(grammarPath);
      if (grammarAbsolutePath != null) {
        try {
          grammar = whisper.parseGrammar(Paths.get(grammarAbsolutePath));
          log.debug("Parsed grammar file: " + grammarAbsolutePath);
        } catch (IOException ex) {
          log.error("Error parsing grammar file " + grammarPath, ex);
        }
      }
    }
    model = new WhisperModel(modelName);
    try {
      context = whisper.init(model.getModelPath());
    } catch (IOException ex) {
      log.error("Error loading whisper model " + modelName, ex);
      return;
    }
    if (showMixers) {
      ConfigurableMicrophone.showMixers();
    }
    if (useMicrophone) {
      microphone = new ConfigurableMicrophone(samplesPerSec, sampleSize, isSigned, isBigEndian, mixer);
      microphone.startRecording();
      inputStream = microphone.getStream();

      audioBuffer = new RealTimeAudioBuffer(bufferLength, 1);

      recordingThread = Executors.newSingleThreadExecutor();
      processingThread = Executors.newSingleThreadExecutor();

      recordingThread.submit(new RecordingRunnable());
      processingThread.submit(new ProcessingRunnable());
    }
  }

  private class RecordingRunnable implements Runnable {
    @Override
    public void run() {
      audioBuffer.bufferAudioData((AudioInputStream) inputStream);
    }
  }

  private class ProcessingRunnable implements Runnable {
    @Override
    public void run() {
      try {
        while (true) {
          byte[] audioData = audioBuffer.getBufferedAudioData();
          processAudioData(audioData);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    private void processAudioData(byte[] audioData) {
      if (!enabled) { // Don't process data if disabled
        return;
      }

      double maxSilenceTime = 0.5; // seconds
      double rms = calculateRMS(audioData);
      boolean isSpeech = rms > threshold;

      if (isSpeech) {
        speechDetected = true;
        // Add the sample to the data
        float[] samples = Audio.bytesToFloats(ByteBuffer.wrap(audioData).order(ByteOrder.LITTLE_ENDIAN));
        System.arraycopy(samples, 0, microphoneData, microphoneCursor, samples.length);
        microphoneCursor += samples.length;
        silenceCounter = 0; // reset silence counter
      } else if (speechDetected){
        silenceCounter++; // increase silence counter
        if (silenceCounter >= (maxSilenceTime / bufferDuration) || microphoneCursor >= microphoneData.length) {
          // if silence exceeds threshold or recording too long
          log.debug("Transcribing...");
          executor.execute(new TranscriptionRunnable(Arrays.copyOfRange(microphoneData, 0, microphoneCursor)));
          microphoneCursor = 0;
          silenceCounter = 0;
          speechDetected = false;
        }
      }
    }
  }

  private class TranscriptionRunnable implements Runnable {
    private float[] audioData;
    public TranscriptionRunnable(float[] audioData) {
      this.audioData = audioData;
    }

    @Override
    public void run() {
      transcribeAndReportSpeech(audioData);
    }
  }


  /**
   * Calculates the root mean square (RMS) amplitude of the audio
   * samples in the provided byte array.
   *
   * @param audioData The byte array containing audio samples
   * @return The root mean square (RMS) amplitude of the audio samples (from 0 to
   */
  private double calculateRMS(byte[] audioData) {
    long sum = 0;

    for (int i = 0; i < audioData.length; i += sampleSizeInBytes) {
      int sample = 0;
      // Combine bytes into a sample value
      if (sampleSizeInBytes == 2) {
        // 16 bit representation
        // byte & 0xFF -> convert to unsigned
        // byte << 8 -> shifts value for another byte
        // (byte1 << 8) | byte2 -> (00010010 << 8) | 00110100 = 00010010 00110100
        if (isBigEndian) {
          // Big Endian Example:
          //   audioData[i]     = 00010010
          //   audioData[i + 1] = 00110100
          //   sample = 00010010 00110100
          sample = ((audioData[i] & 0xFF) << 8) | (audioData[i + 1] & 0xFF);
        } else {
          // Little Endian Example:
          //   audioData[i]     = 00110100
          //   audioData[i + 1] = 00010010
          //   sample = 00010010 00110100
          sample = ((audioData[i + 1] & 0xFF) << 8) | (audioData[i] & 0xFF);
        }

        // Convert to signed representation of 16 bit
        if (isSigned && sample > (1 << 15) - 1) {
          sample -= 1 << 16;
        }
      } else if (sampleSizeInBytes == 1) {
        sample = audioData[i];
      }
      sum += (long) sample * sample;
    }
    double average = sum / (double) (audioData.length / sampleSizeInBytes);
    return Math.sqrt(average);
  }

  /**
   * Copy a file from resources to a temporary file. Used to load
   * grammar files to whisper outside of a build/jar.
   *
   * @param filePath File to copy
   * @return Absolute path to temporary file
   */
  private String fileToTemp (URL filePath) {
    File tempFile = null;
    try {
      tempFile = File.createTempFile("whisper-grammar-", ".gbnf");
    } catch (IOException ex) {
      log.error("Error creating temporary file", ex);
    }
    if (tempFile == null) {
      return null;
    }
    try {
      FileUtils.copyURLToFile(filePath, tempFile);
    } catch (IOException ex) {
      log.error("Error copying " + filePath.toString() + " to " + tempFile.toString(), ex);
    }
    return tempFile.getAbsolutePath();
  }

  /**
   * Transcribe audio data in 16kHz, mono F32
   * @param samples Audio.java data
   * @return transcribed text
   */
  @Override
  public String transcribeSpeech (float[] samples) {
    WhisperFullParams params = new WhisperFullParams();
    String utt = null;
    int result = -1;
    if (saving) {
      saveTempAudio(samples);
    }
    if (grammar != null) {
      params.grammar = grammar;
      params.grammarPenalty = grammarPenalty;
    }
    if (samples.length < 16000 * 2) {
      float[] padded = new float[16000 * 2];
      System.arraycopy(samples, 0, padded, 0, samples.length);
      samples = padded;
    }
    try {
      result = whisper.full(context, params, samples, samples.length);
    } catch (Exception ex) {
      log.error("Error transcribing audio data", ex);
    }
    if (result != 0) {
      log.info("Got to runtime exception. Result: " + result);
      throw new RuntimeException("Transcription failed with code " + result);
    }
    try {
      utt = whisper.fullGetSegmentText(context, 0);
    } catch (IndexOutOfBoundsException ex) {
      log.warn("Could not transcribing speech");
      return null;
    }
    log.debug("Raw transcription: \"" + utt + "\"");
    // Replacing sound effects
    utt = utt.replaceAll("\\(.*?\\)|\\[.*?\\]|\\*.*?\\*", "notspeech");
    return utt.replaceAll("[^a-zA-Z0-9\\s]", "").toLowerCase().trim();
  }

  /**
   * Transcribe audio in 16kHz, mono F32 and report to listener component.
   * @param samples Audio.java data
   */
  @Override
  public void transcribeAndReportSpeech (float[] samples) {
    if (useDisplay) {
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class, Boolean.class));
        service.call(void.class,"Understanding speech", false);
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
    }

    String utt = transcribeSpeech(samples);

    if (useDisplay) {
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
    }

    if (utt == null ||  utt.equals("notspeech")) {
      log.debug("Ignoring non speech transcription");
      return;
    }

    Utterance utterance = new Utterance.Builder()
        .setSpeaker(speaker)
        .setAddressee(addressee)
        .setWords(Arrays.asList(utt.split(" ")))
        .setUtteranceType(UtteranceType.UNKNOWN)
        .setIsInputUtterance(true)
        .build();

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
    } catch (TRADEException ex) {
      log.warn("Failed to report speech \"" + utt + "\"");
      return;
    }
    log.info("Reported utterance \"" + utt + "\"");
  }

  /**
   * Transcribes speech from the audio file located at the specified file path.
   * Expects a 16 bit 16000hz little endian wav file.
   *
   * @param filePath The file path of the audio file to transcribe.
   * @throws UnsupportedAudioFileException If the audio file format is not supported.
   * @throws IOException                   If an I/O error occurs while reading the audio file.
   */
  private void transcribeAudioFile (String filePath) throws UnsupportedAudioFileException, IOException {
    AudioInputStream audioInputStream = AudioSystem.getAudioInputStream(Path.of(filePath).toFile());
    ByteBuffer captureBuffer = ByteBuffer.allocate(audioInputStream.available());
    captureBuffer.order(ByteOrder.LITTLE_ENDIAN);
    int read = audioInputStream.read(captureBuffer.array());
    if (read == -1) {
      throw new IOException("Empty file");
    }
    float[] samples = Audio.bytesToFloats(captureBuffer);
    long start = System.currentTimeMillis();
    log.info(transcribeSpeech(samples));
    log.info((System.currentTimeMillis() - start) + "ms");
  }

  /**
   * Create a wav file from audio sample data.
   *
   * @param audioData audio data in floating point representation
   * @param filePath file to save audio to
   * @param sampleRate sample rate of the audio
   */
  @TRADEService
  public void writeFloatArrayToWav (float[] audioData, String filePath, int sampleRate) {
    AudioFormat format = new AudioFormat(sampleRate, 16, 1, true, false);
    byte[] byteData = new byte[audioData.length * 2];
    for (int i = 0; i < audioData.length; i++) {
      short temp = (short) (audioData[i] * 32767.0f);
      byteData[i * 2] = (byte) (temp & 0xFF);
      byteData[i * 2 + 1] = (byte) ((temp >> 8) & 0xFF);
    }

    AudioInputStream audioInputStream = new AudioInputStream(new ByteArrayInputStream(byteData), format, audioData.length);
    try {
      AudioSystem.write(audioInputStream, AudioFileFormat.Type.WAVE, new File(filePath));
    } catch (IOException ex) {
      log.error("Error writing samples to " + filePath, ex);
    }
  }

  @Override
  public void disableMicrophone () {
    log.debug("[disableMicrophone] Disabling microphone");
    enabled = false;
  };
  @Override
  public void enableMicrophone () {
    log.debug("[enableMicrophone] Enabling microphone");
    enabled = true;
  }

  /**
   * Save a temporary audio file containing transcribed audio data for evaluation
   * @param samples audio data to save
   */
  private void saveTempAudio (float[] samples) {
    File tempFile = null;
    String tempFilePath = null;
    try {
      tempFile = File.createTempFile("whisper-tmp-", ".wav");
    } catch (IOException ex) {
      log.error("Error creating temporary file", ex);
      return;
    }
    tempFilePath = tempFile.getPath();
    writeFloatArrayToWav(samples, tempFilePath, 16000);
    log.debug("Wrote " + String.valueOf(samples.length) + " sample" + (samples.length == 1 ? "" : "s") + " to " + tempFilePath);
  }
}
