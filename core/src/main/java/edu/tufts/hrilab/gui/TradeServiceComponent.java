/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import edu.tufts.hrilab.diarc.DiarcComponent;


/**
 * TradeServiceComponent. A DIARC component that just serves as a marker in a
 * config to start the <code>TradeServiceAdapter</code>.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see TradeServiceAdapter
 */
public class TradeServiceComponent extends DiarcComponent implements GuiProvider {
    //==========================================================================
    // Implement methods | GuiProvider
    //==========================================================================
    @Override
    public String[] getAdapterClassNames() {
        return new String[]{
                TradeServiceAdapter.class.getName()
        };
    }
}
