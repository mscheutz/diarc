/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationConsultant;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationReference;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class MockSpotNavGraphComponent extends DiarcComponent implements SpotNavigationInterface {

    private String mapDescriptor = "location";
    private String refsConfigFile;
    private String refsConfigDir = "config/edu/tufts/hrilab/spot";
    boolean shouldSimExecTime = false; // should the primitive actions simulate execution times
    long simExecTimeout = 1000; // timeout to simulate execution time
    private SpotNavGraphLocationConsultant consultant;
    private Symbol currentLocation;

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("mapDescriptor").hasArg().argName("file").desc("map name for location types and consultant kbName").build());
        options.add(Option.builder("refs").longOpt("references").hasArg().argName("file").desc("load pre-defined object references and their properties").build());
        options.add(Option.builder("simExecTime").desc("Simulate execution time").build());
        options.addAll(super.additionalUsageInfo());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        super.parseArgs(cmdLine);
        if (cmdLine.hasOption("mapDescriptor")) {
            mapDescriptor = cmdLine.getOptionValue("mapDescriptor");
        }
        if (cmdLine.hasOption("refs")) {
            refsConfigFile = cmdLine.getOptionValue("refs");
        }
        if (cmdLine.hasOption("simExecTime")) {
            shouldSimExecTime = true;
        }
    }

    @Override
    protected void init() {
        consultant = new SpotNavGraphLocationConsultant(SpotNavGraphLocationReference.class, mapDescriptor);
        try {
            Collection<String> consultantGroups = this.getMyGroups();
            consultantGroups.add(consultant.getKBName());
            TRADE.registerAllServices(consultant,consultantGroups);
        } catch (TRADEException e) {
            log.error("exception registering spot mission location consultant ",e);
        }
        if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
            consultant.loadReferencesFromFile(refsConfigDir, refsConfigFile);
        }
    }

    @Override
    public Justification goToLocation(Symbol location) {
        log.info("[goToLocation] " + "going to location " + location);
        simExecTime();
        currentLocation = location;
        return new ConditionJustification(true);
    }

    private void simExecTime() {
        if (shouldSimExecTime) {
            try {
                Thread.sleep(simExecTimeout);
            } catch (InterruptedException e) {
            }
        }
    }

    public Justification initLocation(Symbol location) {
        currentLocation = location;
        return new ConditionJustification(true);
    }
}
