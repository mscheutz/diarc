/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashSet;
import java.util.Set;

public class BlockInFront extends Msg {
  public String name;
  public String variant;

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.add(generateFacingObjectPredicate());
    return assertions;
  }

  public Predicate generateFacingObjectPredicate() {
    //facing_obj
    Predicate facingObjPred;
    String blockName = SymbolResolver.toGridItem(name, variant);
    if (blockName.equals("air") || Utilities.queryBeliefSupport(Factory.createPredicate("typeobject", blockName, "air"))) {
      // don't use variants of air for facing_obj. We sadly have lots of hard-coded uses of "air"
      facingObjPred = Factory.createPredicate("facing_obj", actor, Factory.createSymbol("air"), Factory.createSymbol("one"));
    } else {
      facingObjPred = Factory.createPredicate("facing_obj", actor, Factory.createSymbol(blockName), Factory.createSymbol("one"));
    }
    return facingObjPred;
  }
}
