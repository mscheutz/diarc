package edu.tufts.hrilab.python;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class PythonWrapper {
  public PythonWrapper(String file) {
    String classpath = System.getProperty("java.class.path");
    ProcessBuilder processBuilder = new ProcessBuilder();
    processBuilder.command("python3", "-m", file, classpath);
    processBuilder.redirectErrorStream(true);
    processBuilder.inheritIO();

    try {
      Process process = processBuilder.start();

      // Read output stream
      BufferedReader outputReader = new BufferedReader(new InputStreamReader(process.getInputStream()));
      BufferedReader errorReader = new BufferedReader(new InputStreamReader(process.getErrorStream()));

      // Print output
      String line;
      while ((line = outputReader.readLine()) != null) {
        System.out.println("OUTPUT: " + line);
      }

      // Print errors
      while ((line = errorReader.readLine()) != null) {
        System.err.println("ERROR: " + line);
      }

      int exitCode = process.waitFor();
//      log.info("Python script exited with code: " + exitCode);
    } catch (IOException | InterruptedException e) {
      e.printStackTrace();
    }
  }
}
