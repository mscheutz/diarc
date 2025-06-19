package edu.tufts.hrilab.ava.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.util.Collection;

public class AvaGUIAdapter extends GuiAdapter {

  private TRADEServiceInfo setNLInput;

  public AvaGUIAdapter(Collection<String> groups) {
    super(groups);
  }

  @Override
  protected void init() {
    try {
      setNLInput = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("setNLInput").argTypes(String.class, String.class, String.class));
    } catch (TRADEException e) {
      log.error("Failed to find TRADE services", e);
      throw new RuntimeException(e);
    }
  }

  @Override
  protected boolean providesTradeServices() {
    return false;
  }

  @Override
  protected void handleMessage(JsonObject message) {
    String input = message.get("input").getAsString();
    String speaker = message.get("speaker").getAsString();
    String addressee = message.get("addressee").getAsString();
   try {
     setNLInput.call(void.class,input,speaker,addressee);
   } catch (TRADEException e) {
     log.error("Error calling setNLInput from AvaGuiProvider");
   }
  }

  @Override
  protected String getPathRoot() {
    return "avaBehaviors";
  }
}
