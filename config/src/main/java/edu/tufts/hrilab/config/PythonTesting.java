package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.util.Util;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.*;

public class PythonTesting extends DiarcConfiguration {

  @Override
  public void runConfiguration() {

    String classpath = System.getProperty("java.class.path");
//    log.info(classpath);
//    String[] classpathEntries = classpath.split(File.pathSeparator);
//    for(String path : classpathEntries) {
//      log.info(path);
//    }

    createInstance(GoalManagerComponent.class,
                    "-beliefinitfile agents/agents.pl " +
                    "-asl kinova/kinova_examples.asl ");

    ProcessBuilder processBuilder = new ProcessBuilder();
    processBuilder.command("python3", "/home/mfawn/code/diarc/diarc/core/src/main/python/examples/minimal_example.py", classpath);
    processBuilder.redirectErrorStream(true);
    processBuilder.inheritIO();
//
//    try {
//      Process process = processBuilder.start();
//
//      // Read output stream
//      BufferedReader outputReader = new BufferedReader(new InputStreamReader(process.getInputStream()));
//      BufferedReader errorReader = new BufferedReader(new InputStreamReader(process.getErrorStream()));
//
//      // Print output
//      String line;
//      while ((line = outputReader.readLine()) != null) {
//        System.out.println("OUTPUT: " + line);
//      }
//
//      // Print errors
//      while ((line = errorReader.readLine()) != null) {
//        System.err.println("ERROR: " + line);
//      }
//
//      int exitCode = process.waitFor();
//      log.info("Python script exited with code: " + exitCode);
//    } catch (IOException | InterruptedException e) {
//      e.printStackTrace();
//    }
  }
}