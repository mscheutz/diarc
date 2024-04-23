/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.scene;

import edu.tufts.hrilab.fol.Term;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class SceneCollection {

  private List<Term> properties;
  private float noise;
  private List<Scene> scenes = new ArrayList<>();
  private int sceneIndex = 0;

  private static Logger log = LoggerFactory.getLogger(SceneCollection.class);
  private static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

  public SceneCollection(List<Term> properties, float noise) {
    this.properties = properties;
    this.noise = noise;
  }

  public void addScene(Scene scene) {
    scenes.add(scene);
  }

  public List<Term> getProperties() {
    return new ArrayList<>(properties);
  }

  public boolean setSceneIndex(int index) {
    if (index < scenes.size()) {
      sceneIndex = index;
      return true;
    }
    return false;
  }

  public List<MemoryObject> getSceneDetectionResults() {
    List<MemoryObject> results;
    if (sceneIndex < scenes.size()) {
      results = scenes.get(sceneIndex).getDetectionResults();
    } else {
      log.error("Scene index does not exist. Return empty results.");
      results = new ArrayList<>();
    }
    return results;
  }

  public float getNoise() {
    return noise;
  }

  public int getNumScenes() {
    return scenes.size();
  }
}
