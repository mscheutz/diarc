/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.scene;

import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.util.ArrayList;
import java.util.List;

public class Scene {

  private String sceneName;
  private List<MemoryObject> detectionResults;

  public Scene(String sceneName) {
    this.sceneName = sceneName;
    this.detectionResults = new ArrayList<>();
  }

  public Scene(String sceneName, List<MemoryObject> detectionResults) {
    this.sceneName = sceneName;
    this.detectionResults = new ArrayList<>(detectionResults);
  }

  public void addDetectionResult(MemoryObject mo) {
    detectionResults.add(mo);
  }

  public String getSceneName() {
    return sceneName;
  }

  public List<MemoryObject> getDetectionResults() {
    return new ArrayList<>(detectionResults);
  }
}
