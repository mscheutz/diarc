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

public class AvaGuiProvider extends DiarcComponent implements GuiProvider {

  //TODO: gather relevant objects to populate behaviors from consultants and update (locations, videos)

  @TRADEService
  public void setNLInput(String input, String speaker, String addressee) {
    Utterance.Builder utterance = new Utterance.Builder()
            .setWords(Arrays.asList(input.split(" ")))
            .setSpeaker(Factory.createSymbol(speaker))
            .setAddressee(Factory.createSymbol(addressee))
            .setIsInputUtterance(true);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class))
              .call(void.class,utterance.build());
    } catch (TRADEException e) {
      log.error("[setNLInput] reportRecognizedSpeech failed.", e);
    }
  }

  @Override
  public String[] getAdapterClassNames() {
    return new String[] {AvaGUIAdapter.class.getName(), AvaTaughtActionsAdapter.class.getName()};
  }
}
