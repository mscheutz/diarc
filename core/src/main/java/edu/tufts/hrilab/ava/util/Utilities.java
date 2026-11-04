package edu.tufts.hrilab.ava.util;

import java.util.Base64;

public class Utilities {

  /**
   * Encode username and password into auth token header. Prevents
   * need to store username and password in memory.
   *
   * @param username Ava RMS username
   * @param password Ava RMS password
   * @return encoded auth token
   */
  static public String createAuthToken (String username, String password) {
    Base64.Encoder enc = Base64.getEncoder();
    String authStr = username + ":" + password;
    return enc.encodeToString(authStr.getBytes());
  }

  /**
   * Convert a value in pixels to meters via scalar.
   *
   * @param val pixel value
   * @param scale meters/pixel
   * @return meter value
   */
  static public double fromMapPixelScale (double val, double scale) {
    return val * scale;
  }

  /**
   * Convert a value in meters to pixel via scalar.
   *
   * @param val meters to convert
   * @param scale meters/pixel
   * @return pixel value
   */
  static public double toMapPixelScale (double val, double scale) {
    return val / scale;
  }
}
