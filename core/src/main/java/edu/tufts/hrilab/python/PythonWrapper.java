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

        // Wait for the process to complete (blocking)
        process.waitFor();
      } catch (IOException | InterruptedException e) {
        System.err.println("Error while launching or monitoring the process: " + e.getMessage());
      }
    });

    // Register a shutdown hook to terminate the process if it is still running
    Runtime.getRuntime().addShutdownHook(new Thread(() -> {
      if (process != null && process.isAlive()) {
        try (OutputStream outputStream = process.getOutputStream()) {
          // Write "shutdown" to the process's standard input
          // Todo: Why doesn't this connect to python's stdin?
          outputStream.write("shutdown\n".getBytes());
          outputStream.flush();
        } catch (IOException e) {
          System.err.println("Error while sending 'shutdown' command: " + e.getMessage());
        }

        // Wait briefly to allow the process to respond, then terminate
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          System.err.println("Interrupted while waiting for process shutdown.");
        }

        if (process.isAlive()) {
          System.out.println("Forcing process termination...");
          process.destroy();
        }
      }
    }));

    // Start the thread to monitor the process
    processThread.setDaemon(true); // Make it a daemon thread so it won't block JVM termination
    processThread.start();
  }
}
