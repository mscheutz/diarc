/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.gui.service;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.ava.MediaTypes;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.Comparator;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;

@Service
public class FileHandlerService {
  private static final int NORMALIZE_TIMEOUT_SEC = 600;

  String userVideoDir;
  String userImageDir;

  /** From application.properties (ava.video.normalize-audio); env AVA_VIDEO_NORMALIZE_AUDIO or -Dava.video.normalizeAudio override. */
  private final boolean normalizeVideoAudio;

  protected Logger log = LoggerFactory.getLogger(this.getClass());

  private final ConcurrentHashMap<String, ChunkedUploadInfo> activeUploads = new ConcurrentHashMap<>();

  public FileHandlerService(@Value("${ava.video.normalize-audio:true}") boolean normalizeVideoAudio) {
    this.normalizeVideoAudio = normalizeVideoAudio;
    TRADEServiceInfo tsi = null;
    try {
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideosDir").argTypes());
      userVideoDir = tsi.call(String.class);
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getUserImageDir").argTypes());
      userImageDir = tsi.call(String.class);
    } catch (TRADEException e) {
      log.error("[FileHandlerService] error calling TRADE services to identify media paths for Ava media management");
    }
  }

  private static class ChunkedUploadInfo {
    final String fileName;
    final int totalChunks;
    final Path tempDir;

    ChunkedUploadInfo(String fileName, int totalChunks, Path tempDir) {
      this.fileName = fileName;
      this.totalChunks = totalChunks;
      this.tempDir = tempDir;
    }
  }

  public String store(MultipartFile file) {
    String filename = file.getOriginalFilename();
    Path targetLocation;
    
    if (MediaTypes.isVideo(filename)) {
      targetLocation = Paths.get(userVideoDir, filename);
    } else if (MediaTypes.isImage(filename)) {
      targetLocation = Paths.get(userImageDir, filename);
    } else {
      // Default to video directory for unknown types
      targetLocation = Paths.get(userVideoDir, filename);
    }
    
    log.info("[store] path: " + targetLocation);
    try {
      Files.copy(file.getInputStream(), targetLocation);
      if (MediaTypes.isVideo(filename) && normalizeVideoAudio) {
        normalizeVideoAudio(targetLocation);
      }
    } catch (IOException e) {
      log.error("[store] error attempting to write file from stream");
    }
    return filename;
  }

  public boolean initChunkedUpload(String uploadId, String fileName, int totalChunks) {
    try {
      Path tempDir = Files.createTempDirectory("upload-" + uploadId);
      activeUploads.put(uploadId, new ChunkedUploadInfo(fileName, totalChunks, tempDir));
      log.info("[initChunkedUpload] {} -> {} ({} chunks)", uploadId, fileName, totalChunks);
      return true;
    } catch (IOException e) {
      log.error("[initChunkedUpload] failed for {}", uploadId, e);
      return false;
    }
  }

  public boolean storeChunk(String uploadId, int chunkIndex, MultipartFile chunk) {
    ChunkedUploadInfo info = activeUploads.get(uploadId);
    if (info == null) {
      log.error("[storeChunk] unknown uploadId: {}", uploadId);
      return false;
    }

    Path chunkFile = info.tempDir.resolve("chunk-" + String.format("%06d", chunkIndex));
    try {
      Files.write(chunkFile, chunk.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
      return true;
    } catch (IOException e) {
      log.error("[storeChunk] failed for {} chunk {}", uploadId, chunkIndex, e);
      return false;
    }
  }

  public String completeChunkedUpload(String uploadId) {
    ChunkedUploadInfo info = activeUploads.get(uploadId);
    if (info == null) {
      log.error("[completeChunkedUpload] unknown uploadId: {}", uploadId);
      return null;
    }

    try {
      Path targetLocation;
      
      if (MediaTypes.isVideo(info.fileName)) {
        targetLocation = Paths.get(userVideoDir, info.fileName);
      } else if (MediaTypes.isImage(info.fileName)) {
        targetLocation = Paths.get(userImageDir, info.fileName);
      } else {
        // Default to video directory for unknown types
        targetLocation = Paths.get(userVideoDir, info.fileName);
      }

      log.info("[completeChunkedUpload] assembling {} -> {}", uploadId, targetLocation);

      try (OutputStream out = new BufferedOutputStream(Files.newOutputStream(targetLocation, 
          StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING))) {
        Files.list(info.tempDir)
            .filter(p -> p.getFileName().toString().startsWith("chunk-"))
            .sorted(Comparator.comparing(p -> p.getFileName().toString()))
            .forEach(chunkPath -> {
              try {
                Files.copy(chunkPath, out);
              } catch (IOException e) {
                throw new UncheckedIOException(e);
              }
            });
      }

      cleanupTempDir(info.tempDir);
      activeUploads.remove(uploadId);

      if (MediaTypes.isVideo(info.fileName) && normalizeVideoAudio) {
        normalizeVideoAudio(targetLocation);
      }

      return info.fileName;

    } catch (IOException | UncheckedIOException e) {
      log.error("[completeChunkedUpload] failed for {}", uploadId, e);
      cleanupTempDir(info.tempDir);
      activeUploads.remove(uploadId);
      return null;
    }
  }

  public void cancelChunkedUpload(String uploadId) {
    ChunkedUploadInfo info = activeUploads.remove(uploadId);
    if (info != null) {
      cleanupTempDir(info.tempDir);
      log.info("[cancelChunkedUpload] {}", uploadId);
    }
  }

  private void cleanupTempDir(Path tempDir) {
    try {
      if (tempDir != null && Files.exists(tempDir)) {
        Files.walk(tempDir)
            .sorted(Comparator.reverseOrder())
            .forEach(path -> {
              try {
                Files.delete(path);
              } catch (IOException e) {
                log.warn("[cleanupTempDir] failed to delete {}", path);
              }
            });
      }
    } catch (IOException e) {
      log.warn("[cleanupTempDir] failed for {}", tempDir);
    }
  }

  /**
   * Normalize video audio to a consistent loudness (EBU R128-style -24 LUFS) so playback
   * volume is predictable across videos. If ffmpeg is missing or fails, the original file
   * is left unchanged.
   */
  private void normalizeVideoAudio(Path videoPath) {
    Path dir = videoPath.getParent();
    String name = videoPath.getFileName().toString();
    Path normalized = dir.resolve(".normalize." + name);
    try {
      ProcessBuilder pb = new ProcessBuilder(
          "ffmpeg",
          "-i", videoPath.toAbsolutePath().toString(),
          "-filter:a", "loudnorm=I=-24:LRA=7:TP=-2",
          "-c:v", "copy",
          "-c:a", "aac",
          "-b:a", "192k",
          "-y",
          normalized.toAbsolutePath().toString()
      );
      pb.redirectErrorStream(true);
      Process p = pb.start();
      StringBuilder err = new StringBuilder();
      try (BufferedReader r = new BufferedReader(
          new InputStreamReader(p.getInputStream()))) {
        String line;
        while ((line = r.readLine()) != null) err.append(line).append('\n');
      }
      boolean finished = p.waitFor(NORMALIZE_TIMEOUT_SEC, TimeUnit.SECONDS);
      if (!finished) {
        p.destroyForcibly();
        log.warn("[normalizeVideoAudio] ffmpeg timed out for {}; leaving original", name);
        return;
      }
      if (p.exitValue() != 0) {
        log.warn("[normalizeVideoAudio] ffmpeg failed for {} (exit {}); leaving original. stderr: {}",
            name, p.exitValue(), err.length() > 500 ? err.substring(0, 500) + "..." : err);
        return;
      }
      Files.move(normalized, videoPath, StandardCopyOption.REPLACE_EXISTING);
      log.info("[normalizeVideoAudio] normalized audio for {}", name);
    } catch (IOException e) {
      log.warn("[normalizeVideoAudio] failed for {}: {}; leaving original", name, e.getMessage());
      try {
        if (Files.exists(normalized)) Files.delete(normalized);
      } catch (IOException ignored) {}
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      log.warn("[normalizeVideoAudio] interrupted for {}; leaving original", name);
      try {
        if (Files.exists(normalized)) Files.delete(normalized);
      } catch (IOException ignored) {}
    }
  }
}
