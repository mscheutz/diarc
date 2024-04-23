/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* Mocks behavior that would be provided by a Unity server & simulation. Provides TRADE services
 * to give information about station health, tube status, etc.
 */

package edu.tufts.hrilab.unity;

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

public class MockUnitySpaceStationSimulationImpl extends DiarcComponent {

    private static final int NUM_TRIALS = 1;

    private static final int TUBES_PER_SIDE = 12;

    private String configFile;

    private SimulatorConfig simulatorConfig;

    private Map<String, MockUnityTube> tubes;

    public MockUnitySpaceStationSimulationImpl() {
        super();
        this.shouldRunExecutionLoop = true;
    }

    @Override
    protected void executionLoop() {
        long startTimeMillis = System.currentTimeMillis();
        //todo: should this be killable via some other method?
        long estimatedTimeSeconds;
        long prevSecond = 0;
        int tubeEventIndex = 0;
        int numTubeEvents = simulatorConfig.trials[0].tubes.length;
        while (true) {
            estimatedTimeSeconds = (System.currentTimeMillis() - startTimeMillis) / 1000;

            if (estimatedTimeSeconds - prevSecond > 1 && tubeEventIndex < numTubeEvents) {
                prevSecond = estimatedTimeSeconds;
                //handle tube breakage specified in an event if clock matches
                if (estimatedTimeSeconds > simulatorConfig.trials[0].tubes[tubeEventIndex].time) {

                    String tube = simulatorConfig.trials[0].tubes[tubeEventIndex].tube;
                    //todo: handle if random tube(?) for now it is hardcoded assuming the tube id will be in config
                    if (tube == null || tube.isEmpty()) {
                        log.error("[executionLoop] tube break events require tube id specified in valid format e.g. Alpha:L:1");
                    }
                    MockUnityTube currentTube = tubes.get(tube);
                    currentTube.breakTube();
                    log.info("[MockUnitySpaceStationSimulationImpl] broken tube at " + currentTube.getName());

                    tubeEventIndex++;
                }
            }

            //handle tube decay for the current step
            for (MockUnityTube tube : tubes.values()) {
                tube.updateStep();
            }
        }
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
                InputStream stream = getClass().getResourceAsStream(Resources.createFilepath("/",configFile));
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
                    MockUnityTube tube = new MockUnityTube(wing, side, number, false, simulatorConfig.tubeOnDecayRate, simulatorConfig.tubeOffDecayRate);
                    tubes.put(tube.getSimName(), tube);
                }
            }
        }
    }

    @TRADEService
    public void repairTube(String tubeName){
        for (MockUnityTube t : tubes.values()) {
            if (t.getName().equals(tubeName)) {
                t.repairTube();
            }
        }
    }

    @TRADEService
    public double getSpaceStationHealth() {
        double avg = 0.0;
        for (MockUnityTube tube : tubes.values()) {
            avg += tube.getHealth();
        }
        return avg / ((double) tubes.values().size());
    }

    @TRADEService
    public List<String> getSpaceStationTubesDamagedSemanticNames() {
        List<MockUnityTube> damagedTubes = getSpaceStationRawTubesDamaged();
        ArrayList<String> damagedStrings = new ArrayList<>();
        for (MockUnityTube t : damagedTubes) {
            damagedStrings.add(t.getName());
        }
        return damagedStrings;
    }

    @TRADEService
    public List<String> getSpaceStationTubesDamagedInWing(Symbol wing) {
        List<MockUnityTube> damagedTubes = getSpaceStationRawTubesDamaged();
        ArrayList<String> damagedStrings = new ArrayList<>();
        for (MockUnityTube t : damagedTubes) {
            String wingName = t.getWing();
            if (wingName.equals(wing.getName())) {
                damagedStrings.add(t.getName());
            }
        }
        return damagedStrings;
    }

    public List<MockUnityTube> getSpaceStationRawTubesDamaged() {
        ArrayList<MockUnityTube> damaged = new ArrayList<>();
        for (MockUnityTube t : tubes.values()) {
            if (t.getIsBroken()) {
                damaged.add(t);
            }
        }
        return damaged;
    }

    @TRADEService
    public List<String> getWingsWithDamagedTubes() {
        List<String> brokenWings = new ArrayList<>();
        List<String> alertWings = new ArrayList<>();
        for (MockUnityTube tube : getSpaceStationRawTubesDamaged()) {
            String wing = tube.getWing();
            if (!brokenWings.contains(wing)) {
                alertWings.add(wing);
            }
        }
        return alertWings;
    }

    @TRADEService
    public Boolean isTubeBrokenWithSemanticName(String tubeSimName) {
        List<String> damagedStrings = getSpaceStationTubesDamagedSemanticNames();
        if (damagedStrings.contains(tubeSimName)) {
            return Boolean.TRUE;
        }
        return Boolean.FALSE;
    }

    //Classes Required for JSON parsing of config file.

    private static class SimulatorConfig {
        double tubeOnDecayRate;
        double tubeOffDecayRate;
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
