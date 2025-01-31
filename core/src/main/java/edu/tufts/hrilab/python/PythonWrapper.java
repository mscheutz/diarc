package edu.tufts.hrilab.python;

import java.io.*;

public class PythonWrapper {

  private final String file;
  private Process process;

  public PythonWrapper(String file) {
    this.file = file;
  }

  public void start() {
    Thread processThread = new Thread(() -> {
      try {
        // Create the ProcessBuilder and start the process
        String classpath = System.getProperty("java.class.path");
        ProcessBuilder processBuilder = new ProcessBuilder();
        processBuilder.command("python3", "-m", file, classpath);
        processBuilder.inheritIO();
        process = processBuilder.start();

        process.waitFor();
      } catch (IOException | InterruptedException e) {
        System.err.println("Error while launching or monitoring the process: " + e.getMessage());
      }
    });

    // Register a shutdown hook to terminate the process if it is still running
    Runtime.getRuntime().addShutdownHook(new Thread(() -> {
      if (process != null && process.isAlive()) {
        System.out.println("Forcing process termination...");
        process.destroy();
      }
    }));

    // Start the thread to monitor the process
    processThread.start();
  }
}
