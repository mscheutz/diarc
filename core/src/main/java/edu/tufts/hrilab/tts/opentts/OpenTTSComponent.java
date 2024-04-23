/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tts.opentts;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.util.Http;
import edu.tufts.hrilab.util.Util;

import javax.sound.sampled.*;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public class OpenTTSComponent extends DiarcComponent implements SpeechProductionInterface {
  private String openTTSEndpoint = "http://local.tts.hrilab.xyz/api/tts" +
          "?voice=%s" +
          "&lang=en" +
          "&vocoder=high&denoiserStrength=0.01" +
          "&text=%s" +
//          "&speakerId=%s" +
          "&ssml=true&ssmlNumbers=true" +
          "&ssmlDates=true" +
          "&ssmlCurrency=true" +
          "&cache=true";

  @Override
  public boolean sayText(String text) {
    String endpoint = String.format(openTTSEndpoint, "marytts:cmu-bdl-hsmm", text);
    Map<String, String> headers = new HashMap<>();
    String response = Http.sendGetRequest(endpoint, headers);
    byte[] wav = response.getBytes(StandardCharsets.ISO_8859_1);
    playWavFile(wav);
//    log.info("Response: " + response);
    return true;
  }

  @Override
  public boolean sayText(String text, boolean wait) {
    return false;
  }

  @Override
  public boolean isSpeaking() {
    return false;
  }

  @Override
  public boolean stopUtterance() {
    return false;
  }


  /**
   * TODO: write this method
   *
   * @param data
   * @return
   */
  private boolean playWavFile(byte[] data) {


    AudioFormat audioFormat = new AudioFormat(22050, 8, 1, true, true);

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

}
