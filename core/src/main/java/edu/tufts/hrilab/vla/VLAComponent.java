package edu.tufts.hrilab.vla;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.gui.GuiProvider;
import edu.tufts.hrilab.vla.gui.VLAAdapter;

import javax.annotation.Nonnull;

public class VLAComponent extends DiarcComponent implements GuiProvider {
    //==========================================================================
    // Implement methods | GuiProvider
    //==========================================================================
    @Nonnull
    @Override
    public String[] getAdapterClassNames() {
        return new String[]{
                VLAAdapter.class.getName()
        };
    }
}
