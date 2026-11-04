/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.scene.config;

import edu.tufts.hrilab.vision.scene.SceneCollection;
import edu.tufts.hrilab.vision.scene.SceneGenerator;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;

public class ConfigFileSceneGenerator extends SceneGenerator {

    private String filename;

    public ConfigFileSceneGenerator(String filename) {
        super();
        this.filename = filename;
    }

    @Override
    public SceneCollection generateSceneCollection() {
        SceneCollection sceneCollection = null;
        try {
            Reader reader = new FileReader(filename);
            sceneCollection = gson.fromJson(reader, SceneCollection.class);
            log.info("Parsed scene collection:\n" + gson.toJson(sceneCollection));
        } catch (FileNotFoundException e) {
            log.error("Trying to read json to file: " + filename, e);
        }

        return sceneCollection;
    }
}
