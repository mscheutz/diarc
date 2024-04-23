/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.scene;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.util.IdGenerator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;

public abstract class SceneGenerator {

  protected Logger log = LoggerFactory.getLogger(SceneGenerator.class);
  protected Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();
  protected IdGenerator tokenIdGenerator = new IdGenerator();

  public abstract SceneCollection generateSceneCollection();

  public boolean writeSceneCollectionTofile(String filename, SceneCollection sceneCollection) {
    try {
      Writer writer = new FileWriter(filename);
      gson.toJson(sceneCollection, writer);
      writer.flush();
    } catch (IOException e) {
      log.error("Trying to write json to file.", e);
      return false;
    }
    return true;
  }
}
