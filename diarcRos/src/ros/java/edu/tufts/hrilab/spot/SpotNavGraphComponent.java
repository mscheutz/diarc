/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.diarcros.msg.spot_msgs.NavigateToGoal;
import edu.tufts.hrilab.diarcros.spot_ros.SpotDoorDetector;
import edu.tufts.hrilab.diarcros.spot_ros.SpotNavGraph;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationConsultant;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationReference;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.util.ArrayList;
import java.util.List;

//TODO:brad: There is an issue with the DoorOpen messages . To build things rn the need to be delete from : rosjava_core/rosjava_messages/build/generated-src/spot_msgs

public class SpotNavGraphComponent extends DiarcComponent implements SpotNavigationInterface {

    private SpotNavGraph navGraph;
    private String mapDescriptor = "location";
    private String refsConfigFile;
    private String refsConfigDir = "config/edu/tufts/hrilab/spot";

    //Path to nav graph file stored on Spot
    private String navGraphPath ="/";
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
    }

    @Override
    protected void init() {
        navGraph = new SpotNavGraph();

        consultant = new SpotNavGraphLocationConsultant(SpotNavGraphLocationReference.class, mapDescriptor);
        try {
            TRADE.registerAllServices(consultant,this.getMyGroups());
        } catch (TRADEException e) {
            log.error("exception registering spot mission location consultant ",e);
        }
        if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
            consultant.loadReferencesFromFile(refsConfigDir, refsConfigFile);
        }
    }

    @Override
    public Justification goToLocation(Symbol location) {
        log.info("[goToLocation]: "+location);

        /**
         *        map file,
         *        destination waypoint,
         *        use fiducials to localize (unclear how, maybe it needs to see one?),
         *        current waypoint
         */

        if (currentLocation == null) {
            log.error("[goToLocation] location not initialized");
        }
        SpotNavGraphLocationReference currentLocationRef = consultant.getReference(currentLocation);
        SpotNavGraphLocationReference destinationLocation = consultant.getReference(location);

        edu.tufts.hrilab.diarcros.msg.spot_msgs.NavigateToGoal goal = new NavigateToGoal(
                "/home/evan/ws_spot/hrilabtr5.12.23.walk",
                destinationLocation.getGraphId(),
                false,
                currentLocationRef.getGraphId()
        );
        log.info("[goToLocation] goal: "+goal);
        try {
            log.info("[goToLocation] " + "going to location " + location);
            navGraph.sendSpotMsgsNavigateToGoal(goal);
        } catch (RosException e) {
            log.error("[goToLocation] ",e);
        }
        try {
            log.info("[goToLocation] waiting for spot to reach goal");
            navGraph.waitForSpotMsgsNavigateToResult();
        } catch (InterruptedException e) {
            log.error("[goToLocation] ",e);
        }


        SimpleClientGoalState state = navGraph.getSpotMsgsNavigateToState();
        boolean succeeded = (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
        if (succeeded == false) {
            log.error("[goToLocation] failed to go to location " + location);
            return new ConditionJustification(succeeded, Factory.createPredicate("goToLocation", location));
        }
        if (succeeded) {
            log.info("[goToLocation] successfully went from " + currentLocation + " to " + location);
            currentLocation = location;
        }
        return new ConditionJustification(succeeded);
    }

    @Override
    public Justification initLocation(Symbol location) {
        if (currentLocation != null) {
            log.error("[initLocation] location being re-initialized");
        }
        currentLocation = location;
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification setMap() {
        double[] jointPositions = new double[]{-0.20964136123657227, -0.8237118721008301, 0.6861574649810791, -0.002443552017211914, 0.8542506694793701, -0.002493619918823242};
        edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapResponse response = new edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapResponse();
        navGraph.callSpotSetMap(new edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapRequest("/home/evan/ws_spot/hrilabtr5.12.23.walk"), response);
        return new ConditionJustification(response.getSuccess());
    }

}
