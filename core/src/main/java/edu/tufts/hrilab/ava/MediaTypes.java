/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import java.util.Set;

public final class MediaTypes {

  private MediaTypes() {}

  public static final Set<String> VIDEO_EXTENSIONS = Set.of(
      "mp4", "mov", "avi", "webm", "mkv", "wmv", "flv", "mpeg", "mpg", "m4v", "3gp", "ogv"
  );

  public static final Set<String> IMAGE_EXTENSIONS = Set.of(
      "png", "jpg", "jpeg", "gif", "bmp", "tiff", "tif", "webp"
  );

  public static String getExtension(String filename) {
    if (filename == null || filename.isEmpty()) return "";
    int lastDot = filename.lastIndexOf('.');
    if (lastDot < 0 || lastDot == filename.length() - 1) return "";
    return filename.substring(lastDot + 1).toLowerCase();
  }

  public static boolean isVideo(String filename) {
    return VIDEO_EXTENSIONS.contains(getExtension(filename));
  }

  public static boolean isImage(String filename) {
    return IMAGE_EXTENSIONS.contains(getExtension(filename));
  }

  public static String getMediaType(String filename) {
    if (isVideo(filename)) return "video";
    if (isImage(filename)) return "image";
    return "unknown";
  }
}
