package edu.tufts.hrilab.asr.whisper;

import java.io.*;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class WhisperModel {
  static private Logger log = LoggerFactory.getLogger(WhisperModel.class);

  private String cacheDirPath;
  private String baseUrl = "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/";
  private String filePrefix = "ggml-";
  private String fileExtension = ".bin";
  private String models[] = {
    "tiny",
    "tiny.en",
    "tiny-q5_1",
    "tiny.en-q5_1",
    "base",
    "base.en",
    "base-q5_1",
    "base.en-q5_1",
    "small",
    "small.en",
    "small.en-tdrz",
    "small-q5_1",
    "small.en-q5_1",
    "medium",
    "medium.en",
    "medium-q5_0",
    "medium.en-q5_0",
    "large-v1",
    "large-v2",
    "large-v2-q5_0",
    "large-v3",
    "large-v3-q5_0"
  };
  private String modelPath;
  private boolean valid = false;

  public WhisperModel (String modelName) {
    if (!Arrays.asList(models).contains(modelName)) {
      log.error("Model " + modelName + " is not a valid GGML whisper model, choose one of the following: ");
      log.error(String.join(", ", models));
      return;
    }
    if (modelName.contains("tdrz")) {
      baseUrl = "https://huggingface.co/akashmjn/tinydiarize-whisper.cpp/resolve/main/";
    }
    if (cacheDirectory()) {
      modelPath = Paths.get(cacheDirPath,  filePrefix + modelName + fileExtension).toString();
      if (!modelExists()) {
        downloadModel(modelName);
      } else {
        valid = true;
        log.debug("Model " + modelName + " exists in cache");
      }
    } else {
      log.error("Can't use whisper model for ASR");
    }
  }

  /**
   * Create the whisper cache directory in $HOME/.diarc if it does not exist.
   *
   * @return true if no error creating or finding directory
   */
  private boolean cacheDirectory () {
    String homeDirPath = System.getProperty("user.home");
    String diarcDirPath;

    try {
      diarcDirPath = Paths.get(homeDirPath, ".diarc").toString();
      Files.createDirectories(Paths.get(homeDirPath));
    } catch (IOException ex) {
      log.error("Couldn't create .diarc directory in user home directory", ex);
      return false;
    }
    try {
      cacheDirPath = Paths.get(diarcDirPath, "whisper").toString();
      Files.createDirectories(Paths.get(cacheDirPath));
    } catch (IOException ex) {
      log.error("Couldn't create .diarc/whisper cache directory in user home directory", ex);
      return false;
    }
    return true;
  }

  /**
   * Determine if the whisper model exists in the cache dir.
   *
   * @return true if file exists
   */
  private boolean modelExists () {
    File f = new File(modelPath);
    if(f.exists() && !f.isDirectory()) {
      return true;
    }
    return false;
  }

  /**
   * Downloads model to cache.
   *
   * @param modelName name of model to download
   */
  private void downloadModel (String modelName) {
    String modelUrlStr = baseUrl + filePrefix + modelName + fileExtension;
    URL modelUrl = null;
    try {
      modelUrl = new URL(modelUrlStr);
    } catch (MalformedURLException ex) {
      log.error("URL " + modelUrlStr + " is malformed ", ex);
    }
    if (modelUrl == null) {
      return;
    }
    log.debug("Downloading model " + modelName + " from " + modelUrlStr);
    InputStream is = null;
    try {
      is = modelUrl.openStream();
    } catch (IOException ex) {
      log.error("Error opening stream from " + modelUrlStr, ex);
    }
    if (is == null) {
      return;
    }
    BufferedInputStream in = new BufferedInputStream(is);
    FileOutputStream fileOutputStream = null;
    try {
      fileOutputStream = new FileOutputStream(modelPath);
    } catch (FileNotFoundException ex) {
      log.error("File " + modelPath + " not found", ex);
    }
    if (fileOutputStream == null) {
      return;
    }
    byte dataBuffer[] = new byte[1024];
    int bytesRead;
    try {
      while ((bytesRead = in.read(dataBuffer, 0, 1024)) != -1) {
        fileOutputStream.write(dataBuffer, 0, bytesRead);
      }
    } catch (IOException ex) {
      log.error("Error saving file " + modelName, ex);
      return;
    }
    valid = true;
    log.info("Downloaded model " + modelName);
  }

  /**
   * Returns the model path if valid. Null if not.
   *
   * @return full path to model
   */
  public Path getModelPath () {
    if (valid) {
      return Path.of(modelPath);
    }
    return null;
  }
}
