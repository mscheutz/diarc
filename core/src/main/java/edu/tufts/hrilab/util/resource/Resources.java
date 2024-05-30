/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util.resource;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.net.URL;

public class Resources {
  private static Logger log = LoggerFactory.getLogger(Resources.class);

  /**
   * Helper method to pre-pend dir to filename if the dir/filename combination results in a valid file, otherwise
   * filename is returned. This is a utility method only for loading files from the classpath.
   *
   * @param dir      directory path
   * @param filename name of file
   * @return
   */
  public static String createFilepath(String dir, String filename) {

    if (dir != null && !dir.isEmpty()) {
      String tmpFilename;
      if (dir.equals("/")) {
        tmpFilename = "/" + filename;
      } else {
        tmpFilename = "/" + dir + "/" + filename;
      }
      if (Resources.class.getResource(tmpFilename) != null) {
        return tmpFilename;
      }
    }
    return filename;
  }


  /**
   * Helper method to get a resource. Attempts to use the dir/filename combination to find a resource, otherwise
   * just the filename is used.
   *
   * @param dir      directory path
   * @param filename name of file
   * @return
   */
  public static URL getResource(String dir, String filename) {
    if (dir != null && !dir.isEmpty()) {
      String tmpFilename;
      if (dir.equals("/")) {
        tmpFilename = "/" + filename;
      } else {
        tmpFilename = "/" + dir + File.separator + filename;
      }

      URL url = Resources.class.getResource(tmpFilename);
      if (url != null) {
        return url;
      }
    }

    return Resources.class.getResource(filename);
  }
}
