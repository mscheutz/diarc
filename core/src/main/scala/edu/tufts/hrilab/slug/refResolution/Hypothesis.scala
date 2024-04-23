/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution

import edu.tufts.hrilab.fol.{Variable,Symbol}

case class Hypothesis(assignments: Map[Variable, Symbol], likelihood: Double) {
  def merge(other: Hypothesis) :Hypothesis={
    Hypothesis(assignments++other.assignments, likelihood*other.likelihood)
  }
}
