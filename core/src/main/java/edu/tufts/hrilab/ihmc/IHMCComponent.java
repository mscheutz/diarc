/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ihmc;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.python.PythonWrapper;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class IHMCComponent extends DiarcComponent {
  /**
   * Consultant mapping diarc refs to ihmc simulator objects (e.g., loc_1 --> Drill1)
   */
  private IHMCConsultant consultant;
  /**
   * ip@port of hub container that python spokes should connect to
   */
  private String hubContainer;


  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("hubContainer").hasArg().argName("ip@port").desc("Set internal python modules to run as spoke with this hub").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("hubContainer")) {
      hubContainer = cmdLine.getOptionValue("hubContainer");
    }
  }

  @Override
  public void init() {
    PythonWrapper ihmc_wrapper = new PythonWrapper("diarc.ihmc.ihmc_wrapper", true, hubContainer);
    ihmc_wrapper.start();

    PythonWrapper ros2_tf2_wrapper = new PythonWrapper("diarc.ros2_tf2.ros2_tf2_wrapper", true, hubContainer);
    ros2_tf2_wrapper.start();

    consultant = new IHMCConsultant(IHMCReference.class, "location", new ArrayList<>());

    try {
      Collection<String> consultantGroups= this.getMyGroups();
      consultantGroups.add(consultant.getKBName());
      TRADE.registerAllServices(consultant, consultantGroups);
    } catch (TRADEException e) {
      log.error("Error registering consultant.", e);
    }

    // TODO: load pre-defined references
//    if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
//      String filename = Resources.createFilepath(refsConfigDir, refsConfigFile);
//      consultant.loadReferencesFromFile(filename);
//    }

    // add Table1
    List<Term> properties = new ArrayList<>();
    properties.add(Factory.createPredicate("table(X:location)"));
    IHMCReference tableRef = consultant.createReference(Factory.createVariable("X:location"), properties);
    tableRef.setIHMCName("Table1");
    consultant.addReference(tableRef);

    // add Drill1
    properties.clear();
    properties.add(Factory.createPredicate("drill(X:location)"));
    IHMCReference drillRef = consultant.createReference(Factory.createVariable("X:location"), properties);
    drillRef.setIHMCName("Drill1");
    consultant.addReference(drillRef);

    // add Charge1
    properties.clear();
    properties.add(Factory.createPredicate("charge(X:location)"));
    IHMCReference chargeRef = consultant.createReference(Factory.createVariable("X:location"), properties);
    chargeRef.setIHMCName("Charge1");
    consultant.addReference(chargeRef);
  }

  @Action
  @TRADEService
  public Justification goToLocation(Symbol refId) {
    IHMCReference ref = consultant.getReference(refId);
    String ihmcName = ref.getIHMCName();

    Justification justification = null;
    try {
      justification = TRADE.getAvailableService(new TRADEServiceConstraints().name("goto_primitive").argTypes(String.class))
              .call(Justification.class, ihmcName);
    } catch (TRADEException e) {
      log.error("Error calling python goto_primitive.", e);
    }

    return justification;
  }

  @Action
  @TRADEService
  public Justification pickUp(Symbol refId) {
    // TODO: HACK: replace this with refId --> MemoryObject once IHMC supports more natural pickup actions
    String ihmcName = "Drill1";

    Justification justification = null;
    try {
      justification = TRADE.getAvailableService(new TRADEServiceConstraints().name("pickup_primitive").argTypes(String.class))
              .call(Justification.class, ihmcName);
    } catch (TRADEException e) {
      log.error("Error calling python pickup_primitive.", e);
    }

    return justification;
  }
}
