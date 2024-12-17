package edu.tufts.hrilab.python;

import java.io.*;

public class PythonWrapper implements Runnable {


  private Thread thread;
  private final String file;
  private boolean running = false;
  private Process process;

  public PythonWrapper(String file) {
    this.file = file;
  }

  public void start() {
    running = true;
    thread = new Thread(this);
    thread.start();
  }

  public void stop() {
    running = false;
  }

  private void shutdown() {
    if (thread != null && thread.isAlive()) {
      try {
        // Attempt graceful shutdown
        System.out.println("Attempting shutdown: ");

        OutputStream stdin = process.getOutputStream();
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(stdin));
        writer.write("shutdown\n");
        writer.flush();
        writer.close();

        // Wait for process to exit
        process.waitFor();
      } catch (IOException | InterruptedException e) {
        e.printStackTrace();
      } finally {
        System.out.println("ensuring shutdown: ");

        // Ensure process termination
        if (thread.isAlive()) {
          thread.interrupt();
        }
      }
    }
  }

  @Override
  public void run() {
    String classpath = System.getProperty("java.class.path");
    ProcessBuilder processBuilder = new ProcessBuilder();
    processBuilder.command("python3", "-m", file, classpath);
    processBuilder.inheritIO();
    try {
      process = processBuilder.start();
      int exitCode = process.waitFor();
      System.out.println("Python script exited with code: " + exitCode);
    } catch (IOException | InterruptedException e) {
      e.printStackTrace();
    } finally {
      System.out.println("finally run");
      shutdown();
    }
  }
}
