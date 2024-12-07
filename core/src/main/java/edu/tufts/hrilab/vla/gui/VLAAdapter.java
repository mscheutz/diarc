package edu.tufts.hrilab.vla.gui;

import edu.tufts.hrilab.gui.GuiAdapter;
import java.util.Collection;

public class VLAAdapter extends GuiAdapter {

    //==========================================================================
    // Constructors
    //==========================================================================
    public VLAAdapter(Collection<String> groups) {
        super(groups);
    }

    /**
     * {@inheritDoc}
     *
     * @return {@inheritDoc}
     */
    @Override
    public String getPathRoot() {
        return "vla";
    }

    /**
     * {@inheritDoc}
     *
     * @return {@inheritDoc}
     */
    @Override
    protected boolean providesTradeServices() {
        return false;
    }
}
