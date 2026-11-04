package edu.tufts.hrilab.ava.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.gui.GuiProvider;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.Arrays;

public class AvaMediaProvider extends DiarcComponent implements GuiProvider {

  @Override
  public String[] getAdapterClassNames() {
    return new String[] {AvaMediaAdapter.class.getName()};
  }
}
