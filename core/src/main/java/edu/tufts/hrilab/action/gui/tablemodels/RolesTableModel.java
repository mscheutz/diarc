/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui.tablemodels;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.util.Utilities;

import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.DefaultTableModel;
import java.awt.*;

/**
 * Model for the JTable containing roles.
 * Provides various helper methods and takes care of handling the actual java types.
 */
public class RolesTableModel extends DefaultTableModel {
  private final static String[] columnNames = {"Name", "Type", "Default", "Return"};

  /**
   * Initializes the table with the above defined column names.
   */
  public RolesTableModel(ActionDBEntry dbe) {
    super(columnNames, 0);
    setRoles(dbe);
  }

  /**
   * Sets the roles of a DBEntry in the table.
   * @param dbe DBEntry
   */
  public void setRoles(ActionDBEntry dbe) {
    setRowCount(0); // Remove all rows
    for (int i = 0; i < dbe.getNumRoles(); i++) {
      addRow(new Object[] {
          dbe.getRoleName(i),
          dbe.getRoleJavaType(i).getCanonicalName(),
          dbe.getRoleDefault(i),
          dbe.getRoleReturn(i)
      });
    }
  }

  /**
   * Export the current roles to an ActionDBEntry Builder object.
   * @param adbeb builder
   * @return true if success
   */
  public boolean exportRoles(ActionDBEntry.Builder adbeb) {
    for(int i=1; i<getRowCount(); i++) { // skip ?actor variable
      String name = (String)getValueAt(i, 0);
      Class type = Utilities.getClass((String)getValueAt(i, 1));
      String def = (String) getValueAt(i, 2);
      Boolean ret = (getValueAt(i, 3) != null) ? (Boolean)getValueAt(i, 3) : false;

      if(name != null && name.length() > 1 && (name.startsWith("?") || name.startsWith("!"))
          && type != null) {
        adbeb.addRole(new ActionBinding.Builder(name, type).setDefaultValue(def).setIsReturn(ret).build());
      }
      else return false;
    }
    return true;
  }

  /**
   * JTable uses this method to determine the default renderer/
   * editor for each cell.  If we didn't implement this method,
   * then the return column would contain text ("true"/"false"),
   * rather than a check box.
   *
   *  @param c  the column being queried
   *  @return the class represented in the column
   */
  @Override
  public Class getColumnClass(int c) {
    return (c==3) ? Boolean.class : String.class;
  }

  /**
   * Disable editing of the first row (actor variable)
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

      Color error = new Color(150,0,0);

      // Reserved row for actor
      if(row == 0) {
        setForeground(new Color(0,0,255));
      }
      else if(value != null) {
        // Check for variable name
        if (column == 0) {
          String name = (String)value;

          // Check for conflicts
          boolean conflict = false;
          for(int i=0; i<table.getRowCount(); i++) {
            if(i != row && table.getValueAt(i, 0) != null &&
                ((String)table.getValueAt(i, 0)).equalsIgnoreCase(name)) {
              conflict = true;
            }
          }

          if (conflict || ((!name.startsWith("?") && !name.startsWith("!")))) {
            setForeground(error);
          } else {
            setForeground(Color.BLACK);
          }
        }
        // Check for variable type
        else if (column == 1) {
          String type = (String) value;
          if (Utilities.getClass(type) == null) {
            setForeground(error);
          } else {
            setForeground(Color.BLACK);
          }
        }
      }
      return c;
    }
  }
}
