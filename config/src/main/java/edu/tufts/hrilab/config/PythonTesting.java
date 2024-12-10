package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.util.Util;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.*;

public class PythonTesting extends DiarcConfiguration {

  @Override
  public void runConfiguration() {
    createInstance(GoalManagerComponent.class,
                    "-beliefinitfile agents/agents.pl " +
                    "-asl kinova/kinova_examples.asl ");
    String file = "examples.minimal_example";
    PythonWrapper wrapper = new PythonWrapper(file);
  }
}