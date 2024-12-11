package edu.tufts.hrilab.python;

import java.io.*;

public class PythonWrapper {
  public PythonWrapper(String file) {
    String classpath = System.getProperty("java.class.path");
    ProcessBuilder processBuilder = new ProcessBuilder();
    processBuilder.command("python3", "-m", file, classpath);
    processBuilder.inheritIO();
    Process process = null;
    try {
      process = processBuilder.start();
      int exitCode = process.waitFor();
      System.out.println("Python script exited with code: " + exitCode);
    } catch (IOException | InterruptedException e) {
      e.printStackTrace();
    } finally {
      if (process != null) {
//        OutputStream stdin = process.getOutputStream();
//        writer = new BufferedWriter(new OutputStreamWriter(stdin));
//        try {
//          writer.write("shutdown"); // Send shutdown signal
//          writer.flush();
//          writer.close();
//        } catch (IOException e) {
//          throw new RuntimeException(e);
//        }
        if (process.isAlive()) {
          // Todo: This is super unsafe! Figure out how to write to python to make this better
          process.destroyForcibly();
        }
      }
    }
  }
}
