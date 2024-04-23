/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.tester;

import ai.thinkingrobots.trade.TRADEService;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.tufts.hrilab.llm.gson.PromptInfo;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.test.framework.gson.ServiceCallInstance;
import edu.tufts.hrilab.test.framework.gson.TestServiceCalls;
import edu.tufts.hrilab.test.framework.gson.TestUtterances;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.regex.Pattern;

/**
 * Tester class used for generative integration tests. This class handles reading/writing
 * the expected test output to/from file.
 */
public class LLMDataGenratorTester extends DiarcConfigTester {

  /**
   * Base path for writing generated controls (i.e., expected output).
   */
  private final String pathPrefix = "test/resources/controls/";

  /**
   * File extension for writing/reading controls (i.e., expected output) to/from file.
   */
  private final String extension = ".json";

  /**
   * When executing tests (i.e., non-generative mode), load expected results from file and store here.
   * When generating expected output (i.e., generative mode), collect results during test execution here
   * and save to file.
   */
  protected TestUtterances expectedOutputs = null;

  /**
   * Distinguish between "execution" (false) and "generative" (true) test mode.
   */
  public final boolean generativeMode = Boolean.getBoolean("generativeMode");

  /**
   * Override the super method so that generative mode can use the observed output,
   * otherwise pass the observed output to the super class.
   *
   * @param args method name of service that is being wrapped followed by its args.
   */
  @Override
  public void addObservedOutput(Object[] args) {
//    log.debug("adding observed output:  " + Arrays.deepToString(args));
//
//    if (generativeMode) {
//      log.debug("Recorded trade call: " + Arrays.deepToString(args));
//
//      List<String> stringArgs = new ArrayList<>();
//      if (args.length > 0) {
//        for (Object a : Arrays.copyOfRange(args, 1, args.length)) {
//          if (a == null) {
//            stringArgs.add("null");
//          } else {
//            stringArgs.add(a.toString());
//          }
//        }
//      }
//
//      if (args.length == 0) {
//        log.warn("addObservedOutput called in invalid context, and received no args");
//      } else {
//        ServiceCallInstance serviceCall = new ServiceCallInstance(args[0].toString(), stringArgs.toArray(new String[0]));
//        expectedOutputs.addTestletServiceCall(serviceCall);
//      }
//    } else {
//      super.addObservedOutput(args);
//    }
  }

  @TRADEService
  public void addObservedUtteranceText(Utterance utterance) {
    log.info("[addObservedUtteranceText] " + utterance);
    if (generativeMode) {
      log.debug("Recorded trade call: " + utterance);
        expectedOutputs.addUtteranceText(utterance.getWordsAsString());
    }
  }

  @TRADEService
  public void addDesiredSemantics(Object[] args) {
    log.info("[addDesiredSemantics] " + Arrays.deepToString(args));
    if (generativeMode) {
      log.debug("Recorded trade call: " + Arrays.deepToString(args));
      expectedOutputs.addDesiredSemantics(((Utterance)args[1]).toString());
    }
  }

  @TRADEService
  public void addPromptInfo(Object[] args) {
    log.info("[addPromptInfo] " + Arrays.deepToString(args));
    if (generativeMode) {
      log.debug("Recorded trade call: " + Arrays.deepToString(args));
      expectedOutputs.addPromptInfo((PromptInfo) args[1]);
    }
  }


  /**
   * Call this when starting a new @Test.
   */
  public void startTest(Class clazz, String testName) {
    if (generativeMode) {
      expectedOutputs = new TestUtterances(testName);
    } else {
      loadExpectedOutputFromFile(clazz, testName);
    }
  }

  /**
   * Mark new user input during a single @Test.
   */
  public void markNewInput() {
    log.info("\n\n\n");
    if (generativeMode) {
      expectedOutputs.startNextUtterance();
    }
  }

  /**
   * This should be called after each @Test method to put the recorded results in the tests to be written to file.
   *
   * @param testName
   */
  public void endTest(Class clazz, String testName) {
    if (generativeMode) {
      writeGeneratedResults(createFilename(clazz, testName));
    } else {
      clearObservedOutput();
    }
  }

  /**
   * Write recorded expected output to file for a single @Test.
   */
  private void writeGeneratedResults(String filename) {
    if (generativeMode) {
      try {
        File file = new File(filename);
        if (!file.exists()) {
          file.getParentFile().mkdirs();
          file.createNewFile();
        }

        ObjectMapper mapper = new ObjectMapper();
        mapper.writerWithDefaultPrettyPrinter().writeValue(file, expectedOutputs);
      } catch (Exception e) {
        log.error("error writing JSON file for generated test output", e);
      }
    }
  }

  /**
   * Load expected output from file that was written while running the test in generative mode.
   * This loads the expected output for a single @Test.
   * @param clazz
   * @param testName
   */
  private void loadExpectedOutputFromFile(Class clazz, String testName) {
    if (generativeMode) {
      return;
    }

    String filename = createFilename(clazz, testName);

    try {
      File file = new File(filename);
      ObjectMapper mapper = new ObjectMapper();
      expectedOutputs = mapper.readValue(file, TestUtterances.class);
    } catch (IOException e) {
      log.error("Exception loading expected outputs from file: " + filename, e);
    }
  }

  private String createFilename(Class clazz, String testName) {
    // set path (used for reading and writing)
    String pathOfTestClass = clazz.getCanonicalName().replaceAll(Pattern.quote("."), File.separator);
    String path = pathPrefix + pathOfTestClass + File.separator;
    String filename = testName + extension;
    return path + filename;
  }

}
