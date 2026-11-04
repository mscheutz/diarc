package edu.tufts.hrilab.config.ava;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class AvaSpoke extends DiarcConfiguration {

  public void runConfiguration() {

    createInstance(edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent.class, "-config ava.simspeech -speaker evan -addressee ava ");
  }

}


