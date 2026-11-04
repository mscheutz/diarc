package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.fol.Predicate;

import java.io.Serializable;

public class KioskSettings implements Serializable {
  Predicate kioskAction;
  Long kioskTimeout;
  Boolean enabled;
}