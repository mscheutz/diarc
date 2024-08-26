package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.DockingInterface;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationConsultant;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationReference;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class MockRos2SpotComponent extends DiarcComponent implements DockingInterface, MockSpotInterface {

  public MockRos2SpotComponent() {
    super();
  }

  private SpotNavGraphLocationConsultant locationConsultant;
  private Symbol currentLocationId;
  private String refsConfigFile;
  private String refsConfigDir = "config/edu/tufts/hrilab/spot";

  @Override
  protected void init() {
    super.init();

    locationConsultant = new SpotNavGraphLocationConsultant("location");
    locationConsultant.loadReferencesFromFile(refsConfigDir, refsConfigFile);

    List<String> groups = this.getMyGroups();
    groups.add("location");
    try {
      TRADE.registerAllServices(locationConsultant, groups);
    } catch (TRADEException e) {
      log.error("[init] error registering location consultant with TRADE", e);
    }

  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("locationRefs").hasArg().argName("locationRefs").desc("Location ref json filename").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("locationRefs")) {
      refsConfigFile = cmdLine.getOptionValue("locationRefs");
    }
  }

  @Override
  public Justification dock(Symbol dockId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification undock() {
    return new ConditionJustification(true);
  }


  @Override
  public void setInitialLocation(Symbol refId) {
    SpotNavGraphLocationReference ref = locationConsultant.getRefWithId(refId);
    if (ref == null) {
      log.error("[setInitialLocation] current location id does not match a known location");
    }
    currentLocationId = refId;
  }

  @Override
  public Justification setLocationWithFiducial(Symbol refId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification command(String input) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification claim() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification release() {
    return new ConditionJustification(true);
  }

  @Override
  public void bindLocationToCurrentPose(Symbol refId) {

  }

  @Override
  public Justification dock(int id) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification powerOn() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification powerOff() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification stand() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification sit() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification stow() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification unstow() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification carry() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification pickUpObject(Symbol refId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification closeGripper() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification openGripper() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification goToLocationUnsafe(Symbol location) {
    return new ConditionJustification(true);
  }

  @Override
  public boolean hasBeenLocalized() {
    return true;
  }

  @Override
  public Justification localize() {
    return new ConditionJustification(true);
  }
}
