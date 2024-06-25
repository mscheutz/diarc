/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.DockingInterface;

import edu.tufts.hrilab.diarcros.fetch.DockController;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FetchDockingComponent extends DiarcComponent implements DockingInterface {

    private final DockController dockController;
    private final Map<String, Pair<Point3d, Quat4d>> dockCoordinates;
    String dockLocationsFile;

    public FetchDockingComponent() {
        super();
        dockController = new DockController();
        dockCoordinates = new HashMap<>();
        populateDockCoordinates();
    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("dockfile").hasArg(true).desc("dockfile").build());
        options.addAll(super.additionalUsageInfo());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        super.parseArgs(cmdLine);

        if (cmdLine.hasOption("dockfile")) {
            dockLocationsFile = cmdLine.getOptionValue("dockfile");
        } else {
            throw new RuntimeException("No dockfile specified! Use the -dockfile option to provide the FetchDockingComponent knowledge of where the docks are.");
        }
    }

    private void populateDockCoordinates() {
        if (dockLocationsFile != null && !dockLocationsFile.isEmpty()) {
            int lineNumber = 0;
            try {
                FileReader fileReader = new FileReader(dockLocationsFile);
                BufferedReader bufferedReader = new BufferedReader(fileReader);
                String line;
                while ((line = bufferedReader.readLine()) != null) {
                    lineNumber++;
                    String parts1[] = line.split(":");
                    String parts2[] = parts1[1].split(" ");
                    dockCoordinates.put(
                            parts1[0].replace(" ", ""),
                            new ImmutablePair<>(
                                    new Point3d(Float.parseFloat(parts2[0]), Float.parseFloat(parts2[1]), Float.parseFloat(parts2[2])),
                                    new Quat4d(Float.parseFloat(parts2[3]), Float.parseFloat(parts2[4]), Float.parseFloat(parts2[5]), Float.parseFloat(parts2[6]))
                            )
                    );
                }

                bufferedReader.close();
            } catch (IOException e) {
                log.error("bad dock locations file: " + dockLocationsFile + "(line "+lineNumber+")", e);
            }
        } else {
            log.error("No dock locations file specified! Not sure how you go to this error to be honest, pretty sure an error should have been thrown by now.");
        }
    }

    private boolean callTradeGoToLocation(Point3d dock_coordinates, Quat4d dock_orientation) {
        // call movebase trade service to go to dock
        Justification result;
        try {
            TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("goToLocation").argTypes(Double.class,Double.class,Double.class,Double.class,Double.class,Double.class,Boolean.class));
            result = tsi.call(Justification.class, dock_coordinates.x, dock_coordinates.y, dock_orientation.x, dock_orientation.y, dock_orientation.z, dock_orientation.w, true);
        } catch (TRADEException e) {
            return false;
        }
        return result.getValue();
    }

    public Justification dock(Symbol dockId) {
        String key = dockId.getName();
        if (dockCoordinates.containsKey(key)) {
            Point3d p = dockCoordinates.get(key).getLeft();
            Quat4d q = dockCoordinates.get(key).getRight();
            boolean success = callTradeGoToLocation(p, q);
            if (!success)
                return new ConditionJustification(false);
            return dockToVisible();
        }
        log.error("Key not found: " + key);
        return new ConditionJustification(false);
    }

    private Justification dockToVisible() {
        boolean success;
        try {
            success = dockController.callDock();
        } catch (Exception e) {
            log.error("Undocking failed: " + e.getMessage());
            return new ConditionJustification(false);
        }
        return new ConditionJustification(success);
    }

    public Justification undock() {
        boolean success;
        try {
            success = dockController.callUndock();
        } catch (Exception e) {
            log.error("Undocking failed: " + e.getMessage());
            return new ConditionJustification(false);
        }
        return new ConditionJustification(success);
    }

}
