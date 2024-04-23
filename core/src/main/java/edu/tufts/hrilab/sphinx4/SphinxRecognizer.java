/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.cmu.sphinx.api;

import java.io.IOException;
import java.util.Arrays;

import edu.cmu.sphinx.frontend.util.SegmentStorer;
import edu.cmu.sphinx.frontend.util.StreamDataSource;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * High-level class for live speech recognition.
 */
public class SphinxRecognizer extends AbstractSpeechRecognizer {
    protected Logger log = LoggerFactory.getLogger(this.getClass());
  private final Microphone microphone;
  private final edu.cmu.sphinx.api.ConfigurableMicrophone cMicrophone;
  private boolean mixerSelected;

  /**
   * Constructs new live recognition object.
   *
   * @param configuration common configuration
   * @throws IOException if model IO went wrong
   */
  public SphinxRecognizer(String path, Configuration configuration, boolean selectMixer, int mixerID) throws IOException {

    //make a context here
    super(new Context(path, configuration));
//    log = LoggerFactory.getLogger(this.getClass());
    mixerSelected = selectMixer;
    if (selectMixer) {
      microphone = null;
      cMicrophone = new edu.cmu.sphinx.api.ConfigurableMicrophone(16000.0F, 16, true, false, mixerID);
      context.getInstance(StreamDataSource.class).setInputStream(cMicrophone.getStream());
    } else {
      microphone = speechSourceProvider.getMicrophone();
      cMicrophone = null;
      context.getInstance(StreamDataSource.class).setInputStream(microphone.getStream());
    }
  }

  //we pass in the global time indicies for the unk, so we can ground them with the overall segment global time indicies
  public byte[] getAudio(int segmentIndex, long unkStart, long unkEnd) {
    //this is all pretty janky because of how sphinx deals with time
    //for validation, be sure to look at the actual audio
//    logger.warning("segmentIndex: " +segmentIndex);
//    logger.warning("unk start: " + unkStart + " unk end: " + unkEnd);
    byte[] segment = context.getInstance(SegmentStorer.class).getSegment(segmentIndex);
    long segmentStart = context.getInstance(SegmentStorer.class).getStartTime(segmentIndex);
    long segmentEnd = context.getInstance(SegmentStorer.class).getEndTime(segmentIndex);
//    long segmentStart2 = context.getInstance(SegmentStorer.class).getStartTime2(segmentIndex);
//    long segmentEnd2 = context.getInstance(SegmentStorer.class).getEndTime2(segmentIndex);

//    logger.warning("Segment length: " + segment.length + " segment start: " + segmentStart + " segment end: " + segmentEnd);
//    logger.warning("Segment length: " + segment.length + " segment start2: " + segmentStart + " segment end2: " + segmentEnd);

    long timeDiff= (segment.length- ((segmentEnd-segmentStart)*2*16))/32;

    long startTime= (unkStart + timeDiff) - segmentStart;
    long endTime= (unkEnd +timeDiff) - segmentStart;
//    logger.warning("startTime: " + startTime + " endTime: " + endTime);

    long startFrame= startTime*16;
    long endFrame= endTime*16;
//    logger.warning("startFrame: " + startFrame + " endFrame: " + endFrame);

    long startByte= startFrame*2;
    long endByte= endFrame*2;
//    logger.warning("startByte: " + startByte + " endByte: " + endByte);

    //Hopefully the long to int conversion deosnt matter
    //TODO: should we throw an error message here or is the 0 floor enough?
    byte[] data = Arrays.copyOfRange(segment, Math.max(0,(int)(startByte)), Math.max(0,(int)(endByte)));

//    try {
//      FileOutputStream fos = new FileOutputStream("data.raw");
//      fos.write(data);
//      fos.close();
//    } catch (IOException e) {
////      log.error(e);
//    }
////
//    try {
//      FileOutputStream fos = new FileOutputStream("segment.raw");
//      fos.write(segment);
//      fos.close();
//    } catch (IOException e) {
////      log.error(e);
//    }

    return data;
  }

  /**
   * Starts recognition process.
   *
   * @param clear clear cached microphone data
   */
  public void startRecognition(boolean clear) {
    recognizer.allocate();
    if (mixerSelected) cMicrophone.startRecording();
    else microphone.startRecording();
  }

  /**
   * Stops recognition process.
   * <p>
   * Recognition process is paused until the next call to startRecognition.
   */
  public void stopRecognition() {
    if (mixerSelected) cMicrophone.stopRecording();
    else microphone.stopRecording();
    recognizer.deallocate();
  }
}

