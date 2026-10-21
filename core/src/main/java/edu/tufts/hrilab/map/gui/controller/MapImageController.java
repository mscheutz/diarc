/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map.gui.controller;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

@RestController
@RequestMapping("/api/map")
public class MapImageController {

  private static final Logger log = LoggerFactory.getLogger(MapImageController.class);

  @GetMapping("/image")
  public ResponseEntity<byte[]> getMapImage() {
    try {
      BufferedImage mapImage = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapImage").argTypes()).call(BufferedImage.class);

      if (mapImage == null) {
        return ResponseEntity.notFound().build();
      }

      ByteArrayOutputStream baos = new ByteArrayOutputStream();
      ImageIO.write(mapImage, "png", baos);

      HttpHeaders headers = new HttpHeaders();
      headers.setContentType(MediaType.IMAGE_PNG);
      headers.setCacheControl("no-cache");

      return new ResponseEntity<>(baos.toByteArray(), headers, HttpStatus.OK);
    } catch (TRADEException e) {
      log.error("[getMapImage] could not get the map image", e);
      return ResponseEntity.status(HttpStatus.SERVICE_UNAVAILABLE).build();
    } catch (IOException e) {
      log.error("[getMapImage] could not write the map to png", e);
      return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).build();
    }
  }
}
