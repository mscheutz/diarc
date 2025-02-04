package edu.tufts.hrilab.python;

import java.io.*;
import java.net.InetAddress;
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

  public void start() {
    Thread processThread = new Thread(() -> {
      try {
        // Create the ProcessBuilder and start the process
        String classpath = System.getProperty("java.class.path");
        ProcessBuilder processBuilder = new ProcessBuilder();
        processBuilder.command("python3", "-m", file, classpath);
        processBuilder.inheritIO();
        if(spoke) {
          setSpoke(processBuilder);
        }
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

  private void setSpoke(ProcessBuilder processBuilder) {
    try {
      InetAddress ip = InetAddress.getLocalHost();
      System.out.println("Local IP Address: " + ip.getHostAddress());
      // Create a temporary file
      File tempFile = File.createTempFile("trade_spoke", ".properties");
      tempFile.deleteOnExit(); // Ensure deletion on program exit

      // Define properties content
      String content = "STARTBROADCAST=false\n" +
              "STARTDISCOVERY=false\n" +
              "STARTACCEPTINGCONNECTIONS=false\n" +
              "SERVERIPS=\n" +
              "CONNECTCONTAINERS="+ "127.0.0.1@10002";
//              "CONNECTCONTAINERS=" + ip + "@10001";

      System.out.println(content);
      // Write content to file
      try (BufferedWriter writer = new BufferedWriter(new FileWriter(tempFile))) {
        writer.write(content);
      }
      processBuilder.environment().put("TRADE_PROPERTIES_PATH", tempFile.getAbsolutePath());

    } catch (UnknownHostException e) {
      System.out.println("Unable to get IP Address: " + e.getMessage());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
