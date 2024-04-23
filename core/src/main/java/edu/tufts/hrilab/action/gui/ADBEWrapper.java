/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.db.ActionDBEntry;


/**
 * This class wraps an ActionDBEntry and provides methods to edit this entry without actually
 * changing anything within the ActionDBEntry (final fields).
 *
 * This class also provides a toString() method giving a name for the action. This is used
 * throughout the GUI whenever a String representation of the ActionDBEntry is needed.
 */
public class ADBEWrapper {
  private ActionDBEntry action;
  String name;

  public ADBEWrapper(ActionDBEntry adbe) {
    action = adbe;
    name = adbe.getType();
  }

  public String getID() {
    return Integer.toString(action.hashCode());
  }

  public ActionDBEntry getEntry() {
    return action;
  }

  public void replaceEntry(ActionDBEntry adbe) {
    action = adbe;
  }

  public void rename(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  @Override
  public String toString() {
    StringBuilder name = new StringBuilder(this.name);
    boolean first = true;

    for(int i=0; i< action.getNumRoles(); i++) {
      if(!action.getRoleLocal(i)) { // Get the public roles only
        if (first) {
          name.append("(");
          first = false;
        } else name.append(", ");
        name.append(action.getRoleName(i));
      }
    }

    if(!first) name.append(")");
    return name.toString();
  }
}
