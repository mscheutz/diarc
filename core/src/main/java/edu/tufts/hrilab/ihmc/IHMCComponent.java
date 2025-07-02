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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class IHMCComponent extends DiarcComponent {
  private IHMCConsultant consultant;

  @Override
  public void init() {
    PythonWrapper ihmc_wrapper = new PythonWrapper("ihmc.ihmc_wrapper", true);
    ihmc_wrapper.start();

    PythonWrapper ros2_tf2_wrapper = new PythonWrapper("ros2_tf2.ros2_tf2_wrapper", true);
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
  }

  @Action
  @TRADEService
  public Justification goToLocation(Symbol location) {
    IHMCReference ref = consultant.getReference(location);
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
}
