/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tts.opentts;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.util.Util;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.http.client.utils.URIBuilder;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import javax.sound.sampled.*;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class OpenTTSComponent extends DiarcComponent implements SpeechProductionInterface {
  private String openTTSEndpoint = "http://localhost:5500";
  private String openTTSPath = "/api/tts";
  private String uriTemplate;
  private String voice = "marytts:cmu-bdl-hsmm";
  private String speakerId = "";

  // mixer vars set by flags
  private int mixerID;
  private boolean showMixers = false;
  private Mixer.Info mixer;

  private boolean muteMics;

  private boolean cache = false;
  private boolean speaking = false;

  private float volume = 1.0f;

  private Path cacheDir;

  private ExecutorService executor = Executors.newSingleThreadExecutor();

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("endpoint").hasArg().desc("set the endpoint of the OpenTTS service (default: " + openTTSEndpoint + ")").build());
    options.add(Option.builder("voice").hasArg().desc("set the voice of the OpenTTS service (default: " + voice + ")").build());
    options.add(Option.builder("speakerId").hasArg().desc("set the speaker ID of the OpenTTS service, if needed (default: none)").build());
    options.add(Option.builder("cache").desc("cache audio to disk for fast playback of frequent utterances").build());
    options.add(Option.builder("volume").hasArg().desc("set volume of output").build());
    options.add(Option.builder("showMixers").argName("boolean").desc("Show all mixers").build());
    options.add(Option.builder("matchMixer").hasArgs().argName("name").desc("String to match for mixer in case of device re-ordering. Do not quote argument if spaces needed.").build());
    options.add(Option.builder("mixer").hasArg().argName("id").desc("ID of mixer to select for speaker").build());
    options.add(Option.builder("muteMics").argName("boolean").desc("Mute mics when speaking").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    String volumeStr;
    if (cmdLine.hasOption("endpoint")) {
      openTTSEndpoint = cmdLine.getOptionValue("endpoint");
      log.debug("setting endpoint " + openTTSEndpoint);
    }
    if (cmdLine.hasOption("voice")) {
      voice = cmdLine.getOptionValue("voice");
      log.debug("setting voice " + voice);
    }
    if (cmdLine.hasOption("speakerId")) {
      speakerId = cmdLine.getOptionValue("speakerId");
      log.debug("setting speaker ID " + speakerId);
    }
    if (cmdLine.hasOption("cache")) {
      cache = true;
      log.debug("using cache");
    }
    if (cmdLine.hasOption("volume")) {
      volumeStr = cmdLine.getOptionValue("volume");
      volume = Float.parseFloat(volumeStr);
      if (volume < 0.0f) {
        volume = 0.0f;
        log.warn("volume argument " + volumeStr + " < 0.0, using 0.0");
      } else if (volume > 1.0f) {
        volume = 1.0f;
        log.warn("volume argument " + volumeStr + " > 1.0, using 1.0");
      }
      log.debug("setting volume " + volume);
    }
    if (cmdLine.hasOption("showMixers")) {
      showMixers = true;
    }
    if (cmdLine.hasOption("muteMics")) {
      muteMics = true;
    }
    if (cmdLine.hasOption("mixer") || cmdLine.hasOption("matchMixer")){
      if (cmdLine.hasOption("mixer")) {
        mixerID = Integer.parseInt(cmdLine.getOptionValue("mixer"));
      } else if (cmdLine.hasOption("matchMixer")) {
        String matchString = String.join(" ", cmdLine.getOptionValues("matchMixer"));
        mixerID = ConfigurableMicrophone.matchMixer(matchString);
        log.debug("Matching mixer string \"" + matchString + "\"");
      }
      log.debug("Using mixer " + mixerID);
      mixer = AudioSystem.getMixerInfo()[mixerID];
    } else {
      log.debug("Using default mixer");
    }


  }

  @Override
  protected void init () {
    try {
      URIBuilder uri = new URIBuilder(openTTSEndpoint);
      uri.setPath(openTTSPath);
      uri.addParameter("lang", "en");
      uri.addParameter("vocoder", "high");
      uri.addParameter("denoiserStrength", "0.01");
      uri.addParameter("ssml", "true");
      uri.addParameter("ssmlNumbers", "true");
      uri.addParameter("ssmlDates", "true");
      uri.addParameter("ssmlCurrency", "true");
      uri.addParameter("cache", "true");
      uri.addParameter("voice", voice);
      if (!speakerId.equals("")) {
        uri.addParameter("speakerId", speakerId);
      }
      uriTemplate = uri.build().toString();
    } catch (URISyntaxException ex) {
      log.error("error building URI", ex);
    }
    if (showMixers) {
      ConfigurableMicrophone.showMixers();
    }
    if (cache) {
      Path tmpDir = Paths.get(System.getProperty("java.io.tmpdir"));
      cacheDir = Paths.get(tmpDir.toString(),"OpenTTS");
      if (!Files.exists(cacheDir)) {
        new File(cacheDir.toString()).mkdir();
        log.debug("creating cache dir" + cacheDir.toString());
      }
    }
  }

  /**
   * Constructs the cache path with hashed file name.
   *
   * @param text string to synthesize into speech
   * @return File object of cached file location
   */
  private File getCachePath (String text) {
    String textHash = hash(voice + speakerId + text);
    return new File(Paths.get(cacheDir.toString(), textHash + ".wav").toString());
  }

  /**
   * Constructs the OpenTTS request url with arguments.
   *
   * @param text string to synthesize into speech
   * @return string of request url
   */
  private String getUrl (String text) {
    URIBuilder uri = null;
    String url = null;
    try {
      uri = new URIBuilder(uriTemplate);
      uri.addParameter("text", text);
      url = uri.build().toString();
      log.info(url);
    } catch (URISyntaxException ex) {
      log.error("error building URI", ex);
    }
    return url;
  }

  @Override
  public boolean sayText(String text, boolean wait) {
    byte[] wav = null;
    speaking = true;

    // Turn off the microphone
    if (muteMics) {
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("disableMicrophone").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[sayText] Exception disabling microphone", e);
      }
    }

    Future<Boolean> future = executor.submit(new Callable<Boolean>() {
      @Override
      public Boolean call() throws Exception {
        File cachedPath = null;
        String url = null;

        log.info("sayText(" + text + ")");

        if (cache) {
          cachedPath = getCachePath(text);
        }

        if (cache && cachedPath.exists()) {
          log.info("cache hit " + cachedPath.toString());
          playWavFile("file://" + cachedPath.toString(), wait, null);
        } else {
          url = getUrl(text);
          playWavFile(url, wait, cachedPath);
        }
        return true;
      }
    });

    Boolean future_result = true;
    if (wait) {
      try {
        log.debug("Waiting for future to finish.");
        future_result = future.get();
        log.debug("Future finished.");
      } catch (InterruptedException | ExecutionException e) {
        log.error("Error waiting on sayText to finish.", e);
      }
    }

    // Turn on the microphone
    if (muteMics) {
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("enableMicrophone").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[sayText] Exception disabling microphone", e);
      }
    }

    return future_result;
  }

  @Override
  public boolean sayText(String text) {
    return sayText(text, true);
  }

  @Override
  public boolean isSpeaking() {
    return speaking;
  }

  @Override
  public boolean stopUtterance() {
    return false;
  }


  /**
   * Plays an audio file from a source.
   *
   * @param audioFile either a URL or path to file
   * @param cached whether the file is cached
   * @return true if successful
   */
  private boolean playWavFile(String audioFile, boolean wait, File cachedPath) {
    AudioInputStream ais = null;
    Clip playbackClip = null;
    AtomicBoolean isDone = new AtomicBoolean(false);

    try {
      ais = AudioSystem.getAudioInputStream(new URL(audioFile));
    } catch (MalformedURLException ex) {
      log.error("malformed URL", ex);
    } catch (UnsupportedAudioFileException ex) {
      log.error("unsupported file type", ex);
    } catch (IOException ex) {
      log.error("io exception reading file", ex);
    }

    if (ais == null) {
      log.warn("No audio stream, not playing");
      return false;
    }

    if (cache && cachedPath != null) {
      try {
        AudioSystem.write(ais, AudioFileFormat.Type.WAVE, cachedPath);
        log.info("cached file " + cachedPath.toString());
      } catch (IOException ex) {
        log.error("error caching audio file " + cachedPath.toString(), ex);
      }
      try {
        ais = AudioSystem.getAudioInputStream(new URL("file://" + cachedPath.toString()));
      } catch (MalformedURLException ex) {
        log.error("malformed URL", ex);
      } catch (UnsupportedAudioFileException ex) {
        log.error("unsupported file type", ex);
      } catch (IOException ex) {
        log.error("io exception reading file", ex);
      }
    }

    try {
      playbackClip = AudioSystem.getClip(mixer);
      playbackClip.open(ais);

      // LineListener to close the clip when done
      Clip finalPlaybackClip = playbackClip;
      playbackClip.addLineListener(event -> {
        if (event.getType() == LineEvent.Type.STOP) {
          finalPlaybackClip.close();
          isDone.set(true);
        }
      });
    } catch (LineUnavailableException ex) {
      log.error("line unavailable", ex);
    } catch (IOException ex) {
      log.error("io exception reading file", ex);
    }

    if (volume < 1.0f) {
      FloatControl gainControl = (FloatControl) playbackClip.getControl(FloatControl.Type.MASTER_GAIN);
      gainControl.setValue(20f * (float) Math.log10(volume));
    }

    playbackClip.loop(0);
    playbackClip.start();

    while (wait && !isDone.get()) {
      Util.Sleep(1);
    }

    speaking = false;
    return true;
  }

  /**
   * Create a unique SHA-1 hash of a string for the cache. If SHA-1 is not
   * available use less-than-ideal hashCode().
   *
   * @param input string to generate hash from
   * @return hex of hash as string
   */
  private String hash (String input) {
    MessageDigest md = null;
    try {
      md = MessageDigest.getInstance("SHA-1");
    } catch (NoSuchAlgorithmException ex) {
      log.error("no such algorithm", ex);
    }
    if (md == null) {
      return (input.hashCode() + "");
    }
    md.update(input.getBytes(StandardCharsets.UTF_8));
    byte[] digest = md.digest();
    StringBuilder hexString = new StringBuilder();
    for (byte b : digest) {
      hexString.append(String.format("%02x", b));
    }
    return hexString.toString();
  }

}
