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

  private void signalAvaExternalCommandReceived() {
    try {
      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
      service.call(void.class,"Received External Command...");
    } catch (TRADEException e) {
      log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
    }
    new Thread(() -> {
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        log.error("[handleMessage] exception sleeping in clearDisplay thread");
      }
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[handleMessage] Exception displaying text", e);
      }
    }).start();

    try {
      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("playAudioClip").argTypes(String.class));
      service.call(void.class,"config/edu/tufts/hrilab/ava/ding.wav");
    } catch (TRADEException e) {
      log.warn("[handleMessage] exception playing audio clip on Ava", e);
    }
  }

  @Override
  protected void handleMessage(JsonObject message) {
    signalAvaExternalCommandReceived();
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
