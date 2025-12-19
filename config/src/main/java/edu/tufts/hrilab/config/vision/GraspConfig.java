package edu.tufts.hrilab.config.vision;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.util.List;

public class GraspConfig extends DiarcConfiguration {
  @Override
  public void runConfiguration() {
    createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture pcd.xml");
    createInstance(edu.tufts.hrilab.grasp.GraspComponent.class);

    // start vision search
    try {
      long typeId = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTypeId").argTypes(Symbol.class)).call(Long.class, Factory.createSymbol("doorknob"));

      // get vision results
      List<MemoryObject> mos = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTokens").argTypes(Long.class)).call(List.class, typeId);

      // stop vision search
      TRADE.getAvailableService(new TRADEServiceConstraints().name("stopType").argTypes(Long.class)).call(void.class, typeId);

      // call grasp component with vision results
      TRADEServiceInfo graspService = TRADE.getAvailableService(new TRADEServiceConstraints().name("calculateGraspOptions").argTypes(MemoryObject.class));

      mos.forEach(mo -> {
        try {
          List<Grasp> grasps = graspService.call(List.class, mo);
        } catch (TRADEException e) {
          log.error("Error calculating grasp options for object: {}", mo.toString(), e);
        }
      });
    } catch (TRADEException e) {
      log.error("Error getting vision service.", e);
    }
  }

}
