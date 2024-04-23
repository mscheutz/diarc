/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import edu.tufts.hrilab.polycraft.util.SymbolResolver;

public class Select extends Active {
  protected String item;

  public Select(String item) {
    this.item = item;
  }

  /**
   * Select with no item. This deselects the selected item.
   */
  public Select() {
    this.item = null;
  }

  @Override
  public String getCommand() {
    if (item == null) {
      return "SELECT_ITEM";
    } else {
      return "SELECT_ITEM " + SymbolResolver.toPolyItem(item);
    }
  }
}