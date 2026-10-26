/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* Mocks behavior that would be provided by a Unity server & simulation. Provides TRADE services
 * to give information about station health, tube status, etc.
 */

package edu.tufts.hrilab.unity.space_station;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import com.google.gson.Gson;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MockUnitySpaceStationComponent extends DiarcComponent implements UnitySpaceStationInterface {

  private static final int NUM_TRIALS = 1;

  /**
   * Path to simulator json config.
   */
  private String configFile;
  /**
   * Parsed simulator config json info.
   */
  private SimulatorConfig simulatorConfig;
  /**
   * Space station tubes, hashed by diarc name (e.g., alphaLeftThree).
   */
  private Map<String, MockUnityTube> tubes;
  /**
   * Estimated time (sec) since start of component. Used to control tube damage.
   */
  private long estimatedTimeSeconds = 0;
  /**
   * Index into simulatorConfig's tube damage schedule.
   */
  private int tubeEventIndex = 0;

  private static Map<String, String> digitsToWords = new HashMap<>();

  static {
    digitsToWords.put("1", "One");
    digitsToWords.put("2", "Two");
    digitsToWords.put("3", "Three");
    digitsToWords.put("4", "Four");
    digitsToWords.put("5", "Five");
    digitsToWords.put("6", "Six");
    digitsToWords.put("7", "Seven");
    digitsToWords.put("8", "Eight");
    digitsToWords.put("9", "Nine");
    digitsToWords.put("10", "Ten");
    digitsToWords.put("11", "Eleven");
    digitsToWords.put("12", "Twelve");
  }

  public MockUnitySpaceStationComponent() {
    super();
    this.executionLoopCycleTime = 1000;
    this.shouldRunExecutionLoop = true;
  }

  /**
   * Helper method to go from sim tube names (e.g., Gamma:L:7) to diarc tube names (gammaLeftSeven).
   *
   * @param simTubeName
   * @return
   */
  private String toDiarcTubeName(String simTubeName) {
    String[] splitName = simTubeName.split(":");
    return splitName[0].toLowerCase() + (splitName[0].equals("R") ? "Right" : "Left") + digitsToWords.get(splitName[2]);
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("config").hasArg().argName("file").desc("load simulator configuration from provided json file").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("config")) {
      configFile = cmdLine.getOptionValue("config");
    } else {
      log.error("[parseArgs] missing required argument config");
    }
  }

  @Override
  protected void init() {
    Gson gson = new Gson();
    if (configFile != null && !configFile.isEmpty()) {
      try {
        InputStream stream = getClass().getResourceAsStream(Resources.createFilepath("config/edu/tufts/hrilab/unity/space_station", configFile));
        BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
        simulatorConfig = gson.fromJson(reader, SimulatorConfig.class);
        if (simulatorConfig.trials.length != NUM_TRIALS) {
          log.error("[init] mock unity simulator config must have exactly " + NUM_TRIALS + " trials, not " + simulatorConfig.trials.length + " trials");
        }
      } catch (Exception e) {
        log.error("Failed to read config file: " + configFile, e);
      }
    }
    initializeTubes();
  }

  private void initializeTubes() {
    tubes = new HashMap<>();
    for (String wing : new String[]{"alpha", "beta", "gamma"}) {
      for (String side : new String[]{"Left", "Right"}) {
        for (String number : new String[]{"One", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight", "Nine", "Ten", "Eleven", "Twelve"}) {
          MockUnityTube tube = new MockUnityTube(wing, side, number, simulatorConfig.tubeOnDecayRate, simulatorConfig.tubeOffDecayRate);
          tubes.put(tube.getName(), tube);
        }
      }
    }
  }

  @Override
  protected void executionLoop() {
    ++estimatedTimeSeconds; // assumes loop time is one second

    if (tubeEventIndex < simulatorConfig.trials[0].tubes.length) {
      //handle tube breakage specified in an event if clock matches
      if (estimatedTimeSeconds > simulatorConfig.trials[0].tubes[tubeEventIndex].time) {
        String tube = simulatorConfig.trials[0].tubes[tubeEventIndex].tube;
        if (tube == null || tube.isEmpty()) {
          log.error("[executionLoop] tube damaged events require tube id specified in valid format e.g. Alpha:L:1");
        }
        MockUnityTube currentTube = tubes.get(toDiarcTubeName(tube));
        currentTube.startTubeDamage();
        tubeEventIndex++;
      }
    }

    //handle tube decay for the current step
    for (MockUnityTube tube : tubes.values()) {
      tube.updateStep();
    }
  }

  @Override
  public float getSpaceStationHealth() {
    float avg = 0.0f;
    for (MockUnityTube tube : tubes.values()) {
      avg += tube.getHealth();
    }
    return avg / tubes.values().size();
  }

  @Override
  public List<Symbol> getSpaceStationTubesDamaged() {
    List<Symbol> damaged = new ArrayList<>();
    for (MockUnityTube t : tubes.values()) {
      if (t.isDamaged()) {
        damaged.add(t.getNameAsSymbol());
      }
    }
    return damaged;
  }

  @Override
  public List<Symbol> getSpaceStationTubesDamaged(Symbol area) {
    List<Symbol> damaged = new ArrayList<>();
    for (MockUnityTube t : tubes.values()) {
      if (t.isDamaged() && t.inArea(area)) {
        damaged.add(t.getNameAsSymbol());
      }
    }
    return damaged;
  }

  @Override
  public List<Symbol> getSpaceStationTubesBroken() {
    List<Symbol> broken = new ArrayList<>();
    for (MockUnityTube t : tubes.values()) {
      if (t.isBroken()) {
        broken.add(t.getNameAsSymbol());
      }
    }
    return broken;
  }

  @Override
  public List<Symbol> getSpaceStationTubesBroken(Symbol area) {
    List<Symbol> broken = new ArrayList<>();
    for (MockUnityTube t : tubes.values()) {
      if (t.isBroken() && t.inArea(area)) {
        broken.add(t.getNameAsSymbol());
      }
    }
    return broken;
  }

  @Override
  public List<Symbol> getSpaceStationTubesOff() {
    List<Symbol> tubesOff = new ArrayList<>();
    for (MockUnityTube t : tubes.values()) {
      if (t.isOff()) {
        tubesOff.add(t.getNameAsSymbol());
      }
    }
    return tubesOff;
  }

  @Override
  public List<Symbol> getSpaceStationTubesOff(Symbol area) {
    List<Symbol> tubesOff = new ArrayList<>();
    for (MockUnityTube t : tubes.values()) {
      if (t.isOff() && t.inArea(area)) {
        tubesOff.add(t.getNameAsSymbol());
      }
    }
    return tubesOff;
  }

  @Override
  public float getSpaceStationTubeHealth(Symbol tube) {
    return tubes.get(tube.getName()).getHealth();
  }

  @Override
  public boolean getSpaceStationTubeDamaged(Symbol tube) {
    return tubes.get(tube.getName()).isDamaged();
  }

  @Override
  public boolean getSpaceStationTubeBroken(Symbol tube) {
    return tubes.get(tube.getName()).isBroken();
  }

  @Override
  public boolean getSpaceStationTubeOff(Symbol tube) {
    return tubes.get(tube.getName()).isOff();
  }

  @TRADEService
  public boolean repairMockTube(Symbol tube) {
    return tubes.get(tube.getName()).repairTube();
  }

  //////////////////////////////////////////////////////////
  // Helper methods to expose turning tubes on/off via a gui
  //////////////////////////////////////////////////////////
  @TRADEService
  public void turnTubeOn(Symbol tube) {
    tubes.get(tube.getName()).turnOn();
  }

  @TRADEService
  public void turnTubeOff(Symbol tube) {
    tubes.get(tube.getName()).turnOff();
  }
  //////////////////////////////////////////////////////////

  //Classes Required for JSON parsing of config file.

  private static class SimulatorConfig {
    float tubeOnDecayRate;
    float tubeOffDecayRate;
    double tubeRepairRate;
    SimulatorConfigTrial[] trials;
  }

  private static class SimulatorConfigRovers {
    int time;
    int specificity;
    String position;
  }

  private static class SimulatorConfigTrial {
    int seconds;
    int robots;
    boolean survey;
    SimulatorConfigTube[] tubes;
    SimulatorConfigRovers[] rovers;
  }

  private static class SimulatorConfigTube {
    int time;
    int specificity;
    String tube;
  }
}
