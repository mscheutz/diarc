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
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.Comparator;
import java.util.concurrent.ConcurrentHashMap;

@Service
public class FileHandlerService {
  String userVideoDir;
  String userImageDir;

  protected Logger log = LoggerFactory.getLogger(this.getClass());

  private final ConcurrentHashMap<String, ChunkedUploadInfo> activeUploads = new ConcurrentHashMap<>();

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

  public FileHandlerService() {
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
}
