/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.cmu.sphinx.frontend.util;

import edu.cmu.sphinx.frontend.*;
import edu.cmu.sphinx.frontend.endpoint.SpeechEndSignal;
import edu.cmu.sphinx.frontend.endpoint.SpeechStartSignal;
import edu.cmu.sphinx.util.props.*;

import javax.sound.sampled.AudioFileFormat;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import java.io.*;
import java.util.ArrayList;
import java.util.Queue;
import java.util.logging.Level;


/**
 * Stores audio data into numbered (MS-)wav files.
 * TODO: currently the WavWriter buffers all audio data until a DataEndSignal occurs.
 *
 * @author Holger Brandl
 */
public class SegmentStorer extends BaseDataProcessor {

  /**
   * The property for the number of bits per value.
   */
  @S4Integer(defaultValue = 16)
  public static final String PROP_BITS_PER_SAMPLE = "bitsPerSample";

  /**
   * The property specifying whether the input data is signed.
   */
  @S4Boolean(defaultValue = true)
  public static final String PROP_SIGNED_DATA = "signedData";

  private ByteArrayOutputStream baos;
  private DataOutputStream dos;

  private int sampleRate;
  private boolean isInSpeech;

  private boolean isSigned = true;
  private int bitsPerSample;

  //TODO: redundant ultimately want to get rid of it
  protected boolean captureUtts;

  //added by brad
  private ArrayList<byte[]> segments;
  private ArrayList<Long> startTimes;
  private ArrayList<Long> endTimes;

  boolean needStartTime=true;
  long curStartTime=0;
  long curEndTime=0;

  public SegmentStorer(String dumpFilePath, boolean isCompletePath, int bitsPerSample, boolean isSigned) {
//    initLogger();

    this.bitsPerSample = bitsPerSample;
    if (bitsPerSample % 8 != 0) {
      throw new Error("StreamDataSource: bits per sample must be a multiple of 8.");
    }

    this.isSigned = isSigned;
    this.captureUtts = true;
    segments = new ArrayList<>();
    startTimes = new ArrayList<>();

    initialize();
  }

  public SegmentStorer() {
  }

  /*
  * @see edu.cmu.sphinx.util.props.Configurable#newProperties(edu.cmu.sphinx.util.props.PropertySheet)
  */
  @Override
  public void newProperties(PropertySheet ps) throws PropertyException {
    super.newProperties(ps);

    bitsPerSample = ps.getInt(PROP_BITS_PER_SAMPLE);
    if (bitsPerSample % 8 != 0) {
      throw new Error("StreamDataSource: bits per sample must be a multiple of 8.");
    }

    isSigned = ps.getBoolean(PROP_SIGNED_DATA);
    captureUtts = true;
    segments = new ArrayList<>();
    startTimes = new ArrayList<>();
    endTimes = new ArrayList<>();
    initialize();
  }

  @Override
  public Data getData() throws DataProcessingException {
    Data data = getPredecessor().getData();

    if (data instanceof DataStartSignal) sampleRate = ((DataStartSignal) data).getSampleRate();

    if (data instanceof DataStartSignal || (data instanceof SpeechStartSignal && captureUtts)) {
      baos = new ByteArrayOutputStream();
      dos = new DataOutputStream(baos);

      if (data instanceof SpeechStartSignal && captureUtts) {
        long time = ((SpeechStartSignal) data).getTime();
        startTimes.add(time);
      }

    }

    if ((data instanceof DataEndSignal && !captureUtts) ||
        (data instanceof SpeechEndSignal && captureUtts)) {

      if (data instanceof SpeechEndSignal) {
        long time = ((SpeechEndSignal) data).getTime();
        endTimes.add(time);
      }

      recordSegment();

      isInSpeech = false;
    }

    if ((data instanceof DoubleData || data instanceof FloatData)
        && (isInSpeech || !captureUtts)) {
      DoubleData dd = data instanceof DoubleData ? (DoubleData) data : DataUtil.FloatData2DoubleData((FloatData) data);

      if(needStartTime){
        curStartTime = ((DoubleData)data).getCollectTime();
      }
      else{
        curEndTime = ((DoubleData)data).getCollectTime();
      }
      double[] values = dd.getValues();

      for (double value : values) {
        try {
          dos.writeShort(((Double)value).shortValue());
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }

    if (data instanceof SpeechStartSignal) isInSpeech = true;

    return data;
  }


  /**
   * Initializes this DataProcessor. This is typically called after the DataProcessor has been configured.
   */
  @Override
  public void initialize() {
    super.initialize();

    baos = new ByteArrayOutputStream();
  }


  /**
   * Converts a big-endian byte array into an array of doubles. Each consecutive bytes in the byte array are converted
   * into a double, and becomes the next element in the double array. The size of the returned array is
   * (length/bytesPerValue). Currently, only 1 byte (8-bit) or 2 bytes (16-bit) samples are supported.
   *
   * @param values        source values
   * @param bytesPerValue the number of bytes per value
   * @param signedData    whether the data is signed
   * @return a double array, or <code>null</code> if byteArray is of zero length
   * @throws ArrayIndexOutOfBoundsException if boundary fails
   */
  public static byte[] valuesToBytes(double[] values, int bytesPerValue, boolean signedData)
      throws ArrayIndexOutOfBoundsException {

    byte[] byteArray = new byte[bytesPerValue * values.length];

    int byteArInd = 0;

    for (double value : values) {
      int val = (int) value;


      for (int j = bytesPerValue - 1; j >= 0; j++) {
        byteArray[byteArInd + j] = (byte) (val & 0xff);
        val = val >> 8;
      }

      byteArInd += bytesPerValue;
    }

    return byteArray;
  }

  protected void recordSegment() {

    byte[] abAudioData = baos.toByteArray();

    segments.add((abAudioData));
//    logger.log(Level.INFO, "Added segment. Segment size:" + abAudioData.length + "Segments size: " + segments.size());
    needStartTime= true;
  }

  public byte[] getSegment(int index) {
    if (segments.size() >= index && index >= 0) {
      return segments.get(index);
    } else {
//      logger.log(Level.WARNING, "Invalid segment index: " + index + " . Returning null");
      return null;
    }
  }

  public long getStartTime(int index) {
    if (segments.size() >= index && index >= 0) {
      return startTimes.get(index);
    } else {
//      logger.log(Level.WARNING, "Invalid start time index: " + index + " . Returning -1");
      return -1;
    }
  }

  public long getEndTime(int index) {
    if (segments.size() >= index && index >= 0) {
      return endTimes.get(index);
//      return -1;
    } else {
//      logger.log(Level.WARNING, "Invalid end time index: " + index + " . Returning -1");
      return -1;
    }
  }

}
