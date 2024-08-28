/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.Map;
import java.util.HashMap;
import java.util.stream.Collectors;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.InputStream;

import edu.tufts.hrilab.util.resource.Resources;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class Prompts {
  static private Logger log = LoggerFactory.getLogger(Prompts.class);
  private static String promptsDirectoryPath = "config/edu/tufts/hrilab/llm/prompts";
  public static Map<String, Prompt> prompts = new HashMap<>();

  /**
   * Load a prompt into the prompts map from the resources directory.
   * @param name Filename of the prompt template minus .txt
   */
  private static void loadPrompt(String name) {
    String promptPath = Resources.createFilepath(promptsDirectoryPath, name + ".txt");
    String text = null;
    try {
      text = getResourceFileAsString(promptPath);
    } catch (IOException ex) {
      log.error("Error loading file " + promptPath, ex);
    }
    prompts.put(name, new Prompt(text));
    if (text != null && !text.trim().equals("")) {
      log.debug("Loaded prompt \"" + name + "\"");
    } else {
      log.warn("Prompt \"" + name + "\" is empty");
    }
  }

  /**
   * Reads given resource file as a string.
   *
   * @param fileName Path to the prompt text resource file
   * @return String containing the text of the prompt
   * @throws IOException if read fails
   */
  private static String getResourceFileAsString(String filename) throws IOException {
    InputStream stream = Prompts.class.getResourceAsStream(filename);
    if (stream == null) {
      log.error("Resource not found: " + filename);
      return null;
    }

    BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
    return reader.lines().collect(Collectors.joining("\n"));
  }

  /**
   * Returns the Prompt object from the name of the corresponding prompt text file.
   * @param name Filename of the prompt template minus .txt
   * @return Prompt object
   */
  public static Prompt getPrompt (String name) {
    if (!prompts.containsKey(name)) {
      loadPrompt(name);
    }
    return prompts.get(name);
  }
}
