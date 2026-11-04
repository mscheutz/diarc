/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tts.opentts;

public class OpenTTSRequestBody {
  public String text;

  public String voice = "coqui-tts:en_vctk";
  public String lang = "en";
  public String vocoder = "high";
  public float denoiserStrength = 0.005f;
  public String speakerId = "p362";
  public boolean ssml = false;
  public boolean ssmlDates = true;
  public boolean ssmlCurrency = true;
  public boolean ssmlNumbers = true;
  public boolean cache = true;


  public OpenTTSRequestBody() {

  }

  public OpenTTSRequestBody(String input) {
    text = input;
  }
}
