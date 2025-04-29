package edu.tufts.hrilab.python;

import java.io.*;
import java.net.UnknownHostException;

public class PythonWrapper {

  private final String file;
  private Process process;
  private boolean spoke = false;

  public PythonWrapper(String file) {
    this.file = file;
  }

  public PythonWrapper(String file, boolean spoke) {
    this.file = file;
    this.spoke = spoke;
  }

  /**
   * Starts the python process. Some important side effects:
   * - Passes in the "--classpath" arg to python. As a result, Python modules that use argparse will need to use parse_known_args
   * - Starts a shutdown hook to end the python process when the JVM is shut down

   */
  public void start() {
    Thread processThread = new Thread(() -> {
      try {
        // Create the ProcessBuilder and start the process
        String classpath = System.getProperty("java.class.path");
        ProcessBuilder processBuilder = new ProcessBuilder();
        processBuilder.command("python3", "-m", file, "--classpath", classpath);
        processBuilder.inheritIO();

        // Launches the python process as a TRADE spoke
        if(spoke) {
          setSpoke(processBuilder);
        }

        process = processBuilder.start();

        process.waitFor();
      } catch (IOException | InterruptedException e) {
        System.err.println("Error while launching or monitoring the process: " + e.getMessage());
      }
    });

    // Shutdown hook to make sure python process ends when JVM ends
    Runtime.getRuntime().addShutdownHook(new Thread(() -> {
      if (process != null && process.isAlive()) {
        System.out.println("Forcing process termination...");
        process.destroy();
      }
    }));

    processThread.start();
  }

  private void setSpoke(ProcessBuilder processBuilder) {
    try {
      // Create a temporary spoke file
      File tempFile = File.createTempFile("trade_spoke", ".properties");
      tempFile.deleteOnExit(); // Ensure deletion on program exit

      // Define TRADE config
      String content = "STARTBROADCAST=false\n" +
              "STARTDISCOVERY=false\n" +
              "STARTACCEPTINGCONNECTIONS=false\n" +
              "SERVERIPS=\n" +
              "SCANFORSERVER=\n" +
              "CONNECTCONTAINERS="+ "127.0.0.1@10002";

      try (BufferedWriter writer = new BufferedWriter(new FileWriter(tempFile))) {
        writer.write(content);
      }

      //Set env var to spoke
      processBuilder.environment().put("TRADE_PROPERTIES_PATH", tempFile.getAbsolutePath());
    } catch (UnknownHostException e) {
      System.out.println("Unable to get IP Address: " + e.getMessage());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
