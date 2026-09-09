/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

public class EntityScore {
  private final boolean inMainClause;
  private final int synProm;
  private final int recency;
  private final double bonus;

  public EntityScore(boolean inMainClause, int synProm, int recency, double bonus) {
    this.inMainClause = inMainClause;
    this.synProm = synProm;
    this.recency = recency;
    this.bonus = bonus;
  }

  public boolean inMainClause() {
    return inMainClause;
  }

  public int synProm() {
    return synProm;
  }

  public int recency() {
    return recency;
  }

  public double bonus() {
    return bonus;
  }
}
