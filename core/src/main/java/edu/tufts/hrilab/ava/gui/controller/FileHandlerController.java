/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ava.gui.controller;

import edu.tufts.hrilab.ava.gui.service.FileHandlerService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

@RestController
@RequestMapping("/api/files")
public class FileHandlerController {

  private final FileHandlerService fileHandlerService;

  @Autowired
  public FileHandlerController(FileHandlerService fileHandlerService) {
    this.fileHandlerService = fileHandlerService;
  }

  @PostMapping("/upload")
  public ResponseEntity<String> uploadFile(@RequestParam("file") MultipartFile file) {
    String fileName = fileHandlerService.store(file);
    return ResponseEntity.ok(fileName);
  }

  @PostMapping("/upload/init")
  public ResponseEntity<String> initChunkedUpload(
      @RequestParam("uploadId") String uploadId,
      @RequestParam("fileName") String fileName,
      @RequestParam("totalChunks") int totalChunks) {
    if (fileHandlerService.initChunkedUpload(uploadId, fileName, totalChunks)) {
      return ResponseEntity.ok("OK");
    }
    return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body("Failed");
  }

  @PostMapping("/upload/chunk")
  public ResponseEntity<String> uploadChunk(
      @RequestParam("uploadId") String uploadId,
      @RequestParam("chunkIndex") int chunkIndex,
      @RequestParam("chunk") MultipartFile chunk) {
    if (fileHandlerService.storeChunk(uploadId, chunkIndex, chunk)) {
      return ResponseEntity.ok("OK");
    }
    return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Failed");
  }

  @PostMapping("/upload/complete")
  public ResponseEntity<String> completeChunkedUpload(@RequestParam("uploadId") String uploadId) {
    String fileName = fileHandlerService.completeChunkedUpload(uploadId);
    if (fileName != null) {
      return ResponseEntity.ok(fileName);
    }
    return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body("Failed");
  }

  @PostMapping("/upload/cancel")
  public ResponseEntity<String> cancelChunkedUpload(@RequestParam("uploadId") String uploadId) {
    fileHandlerService.cancelChunkedUpload(uploadId);
    return ResponseEntity.ok("OK");
  }
}
