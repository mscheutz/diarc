/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui.tablemodels;

import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;

import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.DefaultTableModel;
import java.awt.*;

/**
 * Model for the JTable containing effects.
 * Provides various helper methods and takes care of handling the actual java types.
 */
public class EffectsTableModel extends DefaultTableModel {
  private final static String[] columnNames = {"Type", "Predicate", "Observe", "SE"};

  /**
   * Initializes the table with the above defined column names.
   */
  public EffectsTableModel(ActionDBEntry dbe) {
    super(columnNames, 0);
    setEffects(dbe);
  }

  /**
   * Sets the roles of a DBEntry in the table.
   * @param dbe DBEntry
   */
  public void setEffects(ActionDBEntry dbe) {
    setRowCount(0); // Remove all rows
    dbe.getEffects().forEach(e -> addRow(
        new Object[] {e.getType(), e.toString(), e.getObservable(), false}));
  }

  /**
   * Export the current effects to an ActionDBEntry Builder object.
   * @param adbeb builder
   * @return true if success
   */
  public boolean exportEffects(ActionDBEntry.Builder adbeb) {
    for(int i=1; i<getRowCount(); i++) { // Skip first (default) post condition
      EffectType type =(EffectType)getValueAt(i, 0);
      String predicate = (String)getValueAt(i, 1);
      Observable obs = (Observable)getValueAt(i,2);

      if(predicate != null && predicate.length() > 0 && type != null && obs != null) {
        adbeb.addEffect(new Effect(Factory.createPredicate(predicate), type, obs));
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
      case 0 : return EffectType.class;
      case 1 : return String.class; // Predicate as string
      case 2 : return Observable.class;
      case 3 : return Boolean.class;
    }
    return String.class;
  }

  /**
   * Disable editing of the first row (default post condition)
   * @param rowIndex
   * @param columnIndex
   * @return
   */
  @Override
  public boolean isCellEditable(int rowIndex, int columnIndex) {
    return rowIndex != 0;
  }

  /**
   * Custom renderer to highlight errors in cells.
   */
  public static class Renderer extends DefaultTableCellRenderer {
    @Override
    public Component getTableCellRendererComponent(JTable table, Object value,
                                                   boolean isSelected, boolean hasFocus, int row, int column) {
      Component c = super.getTableCellRendererComponent(table, value, isSelected,
          hasFocus, row, column);

      // Reserved row for default post condition
      if (row == 0) {
        setForeground(new Color(0, 0, 255));
      } else {
        setForeground(Color.BLACK);
      }
      return c;
    }
  }
}
