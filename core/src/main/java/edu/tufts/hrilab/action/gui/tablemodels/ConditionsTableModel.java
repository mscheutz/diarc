/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui.tablemodels;

import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;

import javax.swing.table.DefaultTableModel;

/**
 * Model for the JTable containing conditions.
 * Provides various helper methods and takes care of handling the actual java types.
 */
public class ConditionsTableModel extends DefaultTableModel {
  private final static String[] columnNames = {"Type", "Condition"};

  /**
   * Initializes the table with the above defined column names.
   */
  public ConditionsTableModel(ActionDBEntry dbe) {
    super(columnNames, 0);
    setConditions(dbe);
  }

  /**
   * Sets the conditions of a DBEntry in the table.
   * @param dbe DBEntry
   */
  public void setConditions(ActionDBEntry dbe) {
    setRowCount(0); // Remove all rows
    dbe.getPreConditions().forEach(c -> addRow(new Object[] {c.getType(), c}));
    dbe.getOverallConditions().forEach(c -> addRow(new Object[] {c.getType(), c}));
    dbe.getObligationConditions().forEach(c -> addRow(new Object[] {c.getType(), c}));
  }

  /**
   * Export the current conditions to an ActionDBEntry Builder object.
   * @param adbeb builder
   * @return true if success
   */
  public boolean exportConditions(ActionDBEntry.Builder adbeb) {
    for(int i=0; i<getRowCount(); i++) {
      Condition condition = (Condition)getValueAt(i, 1);
      if(condition != null) {
        adbeb.addCondition(condition);
      }
      else return false;
    }
    return true;
  }

  /**
   * JTable uses this method to determine the default renderer/
   * editor for each cell.
   *
   *  @param c  the column being queried
   *  @return the class represented in the column
   */
  @Override
  public Class getColumnClass(int c) {
    switch(c) {
      case 0 : return ConditionType.class;
      case 1 : return Condition.class;
    }
    return String.class;
  }


  /**
   * Model for condition sub-parts (map of predicates to observable state).
   */
  static public class Subparts extends DefaultTableModel {
    public Subparts(Condition c) {
      super(new String[]{"Predicate", "Observable"}, 0);
      if (c != null) {
        c.getPredicates().forEach((p, o) -> addRow(new Object[]{p.toString(), o}));
      }
    }

    public Condition export(ConditionType type) {
      Condition.Disjunction d = new Condition.Disjunction(type);
      for (int i = 0; i < getRowCount(); i++) {
        String predicate = (String) getValueAt(i, 0);
        Observable observable = (Observable) getValueAt(i, 1);
        if(predicate != null && predicate.length() > 0 && observable != null) {
          d.or(Factory.createPredicate(predicate), observable);
        }
        else return null; // Malformed condition.
      }
      return new Condition(d);
    }

    @Override
    public Class getColumnClass(int c) {
      switch (c) {
        case 0:
          return String.class;
        case 1:
          return Observable.class;
      }
      return String.class;
    }
  }
}
