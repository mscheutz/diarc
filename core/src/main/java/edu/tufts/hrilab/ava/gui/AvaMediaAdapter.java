package edu.tufts.hrilab.ava.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.util.Collection;

public class AvaMediaAdapter extends GuiAdapter {

  public AvaMediaAdapter(Collection<String> groups) {
    super(groups);
  }

  @Override
  protected boolean providesTradeServices() {
    return false;
  }

  @Override
  protected void handleMessage(JsonObject message) {

  }

  @Override
  protected String getPathRoot() {
    return "media";
  }
}
