/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

public class Audio {
  /**
   * Converts the given ByteBuffer containing audio data represented as 16-bit shorts
   * to an array of floating-point values normalized between -1.0 and 1.0.
   *
   * @param buffer The ByteBuffer containing audio data represented as 16-bit shorts
   * @return An array of floating-point values representing the normalized audio samples
   */
  public static float[] bytesToFloats (ByteBuffer buffer) {
    ShortBuffer shortBuffer = buffer.order(ByteOrder.LITTLE_ENDIAN).asShortBuffer();
    float[] samples = new float[buffer.capacity() / 2];
    var i = 0;
    while (shortBuffer.hasRemaining()) {
      samples[i++] = Float.max(-1f, Float.min(((float) shortBuffer.get()) / (float) Short.MAX_VALUE, 1f));
    }
    return samples;
  }
}