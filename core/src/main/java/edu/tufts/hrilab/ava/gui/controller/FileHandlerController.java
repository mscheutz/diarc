/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ava.gui.controller;

import edu.tufts.hrilab.ava.gui.service.FileHandlerService;
import org.springframework.beans.factory.annotation.Autowired;
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
}