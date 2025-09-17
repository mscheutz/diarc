/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.gui.service;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

@Service
public class FileHandlerService {
  String userVideoDir;
  String userImageDir;

  protected Logger log = LoggerFactory.getLogger(this.getClass());

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
    String[] split = file.getOriginalFilename().split("\\.");
    Path targetLocation = Path.of("");
    if (split[1].equals("mp4")) {
      targetLocation = Paths.get(userVideoDir, file.getOriginalFilename());
    } else if  (split[1].equals("png")) {
      targetLocation = Paths.get(userImageDir, file.getOriginalFilename());
    }
    log.info("[store] path: " + targetLocation);
    try {
      Files.copy(file.getInputStream(), targetLocation);
    } catch (IOException e) {
      log.error("[store] error attempting to write file from stream");
    }
    return file.getOriginalFilename();
  }


  }
