package edu.tufts.hrilab.python;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PythonWrapper {

  private Logger log = LoggerFactory.getLogger(PythonWrapper.class);
  private final String file;
  private Process process;
  private boolean spoke = false;
  private String hubContainerIpPort = "127.0.0.1@10002";

  public PythonWrapper(String file) {
    this.file = file;
  }

  /**
   * If running python module as a TRADE spoke (i.e., spoke == true), this constructor uses the
   * default 127.0.0.1@10002 hub address and port.
   *
   * @param file python module to start
   * @param spoke if true, connect to TRADE as a spoke (requires a TRADE hub to be running)
   */
  public PythonWrapper(String file, boolean spoke) {
    this.file = file;
    this.spoke = spoke;
  }

  /**
   * If running python module as a TRADE spoke (i.e., spoke == true), this constructor requires setting the
   * TRADE hub address and port.
   *
   * @param file python module to start
   * @param spoke if true, connect to TRADE as a spoke (requires a TRADE hub to be running)
   * @param hubContainer TRADE hub to connect to of the form ip_adress@port (e.g., 127.0.0.1@10002)
   */
  public PythonWrapper(String file, boolean spoke, String hubContainer) {
    this.file = file;
    this.spoke = spoke;
    if (hubContainer != null && !hubContainer.isEmpty()) {
      this.hubContainerIpPort = hubContainer;
    }
  }

  /**
   * Starts the python process. Some important side effects:
   * - Passes in the "--classpath" arg to python. As a result, Python modules that use argparse will need to use parse_known_args
   * - Starts a shutdown hook to end the python process when the JVM is shut down

   */
  // Todo: Add python arguments as args to this func
  public void start(String... pythonArgs) {
    Thread processThread = new Thread(() -> {
      try {
        // Create the ProcessBuilder and start the process
        String classpath = System.getProperty("java.class.path");
        ProcessBuilder processBuilder = new ProcessBuilder();
        List<String> commandArgs = new ArrayList<>(Arrays.asList("python3", "-m", file, "--classpath", classpath));
        commandArgs.addAll(Arrays.asList(pythonArgs));
        processBuilder.command(commandArgs);
        processBuilder.inheritIO();

        // Launches the python process as a TRADE spoke
        if(spoke) {
          setSpoke(processBuilder);
        }

        process = processBuilder.start();

        process.waitFor();
      } catch (IOException | InterruptedException e) {
        log.error("Error while launching or monitoring the process.", e);
      }
    });

    // Shutdown hook to make sure python process ends when JVM ends
    Runtime.getRuntime().addShutdownHook(new Thread(() -> {
      if (process != null && process.isAlive()) {
        log.info("Forcing process termination...");
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
              "CONNECTCONTAINERS="+ hubContainerIpPort;

      try (BufferedWriter writer = new BufferedWriter(new FileWriter(tempFile))) {
        writer.write(content);
      }

      //Set env var to spoke
      processBuilder.environment().put("TRADE_PROPERTIES_PATH", tempFile.getAbsolutePath());

    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
