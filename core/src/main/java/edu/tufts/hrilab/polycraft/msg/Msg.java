/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Set;

abstract public class Msg {
  protected Logger log = LoggerFactory.getLogger(this.getClass());
  protected Symbol actor = Factory.createSymbol("self");

  abstract public Set<Predicate> generateAssertions();
}
