package edu.tufts.hrilab.ava.tts;

import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.tts.MaryTTSComponent;

import javax.sound.sampled.UnsupportedAudioFileException;
import java.io.IOException;

//TODO: Pulled Ava specific logic out of nlg.asl and created this instead for branch merging purposes.
// Look into how to do this properly (Motivation is not wanting robot to speak locally when a response is meant for remote user)
//In general, where/how do we decide what form of communication occurs based off of the addressee of an utterance and the configuration of a domain?
public class AvaMaryTTSComponent extends MaryTTSComponent {
  public AvaMaryTTSComponent() throws IOException, UnsupportedAudioFileException, InterruptedException {
    super();
  }

  @Override
  public boolean sayText(Utterance utterance) {
    return sayText(utterance,true);
  }


  @Override
  public boolean sayText(Utterance utterance, boolean wait) {
    if (!utterance.getAddressee().getName().equals("webapp")) {
      return sayText(utterance.getWordsAsString(), wait);
    } else {
      log.info("published in webapp chat: {}", utterance.getWordsAsString());
      return false;
    }
  }
}
