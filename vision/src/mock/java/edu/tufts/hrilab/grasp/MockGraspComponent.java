package edu.tufts.hrilab.grasp;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.util.ArrayList;
import java.util.List;

public class MockGraspComponent extends DiarcComponent {

  @TRADEService
  public List<Grasp> calculateGraspOptions(Symbol refId) {
    return calculateGraspOptions(refId, new ArrayList<>());
  }

  @TRADEService
  public List<Grasp> calculateGraspOptions(Symbol refId, List<? extends Term> constraints) {
    return new ArrayList<>();
  }

  public List<Grasp> calculateGraspOptions(MemoryObject mo) {
    return new ArrayList<>();
  }

  //tood: (pete) put this somewhere sensible
  @TRADEService
  public Grasp calculateSingleGraspOption(MemoryObject mo) {
    return null;
  }

  @TRADEService
  public double calculateAngleOfObject(MemoryObject mo) {
    return 0;
  }
}
