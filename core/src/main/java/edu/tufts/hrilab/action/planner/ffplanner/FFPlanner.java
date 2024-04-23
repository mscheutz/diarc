/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.planner.ffplanner;

import edu.tufts.hrilab.action.planner.Planner;
import edu.tufts.hrilab.util.Util;

import java.io.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class FFPlanner extends Planner {
  /**
   * The following system property must be set in the JVM.
   */
  protected String exec_path = System.getProperty("diarc.planner.ff");

  @Override
  protected String plan(File domain, File problem) {
    log.info("Using planner executable: {}", exec_path);

    ProcessBuilder processBuilder = new ProcessBuilder();
    processBuilder.command(exec_path, "-o", domain.getAbsolutePath(), "-f", problem.getAbsolutePath(), "-s", "0");

    try {
      // start process outside of executor so process.destroy can be called on hanging process
      Process process = processBuilder.start();

      // start executor/thread to handle output from planner
      ExecutorService executor = Executors.newSingleThreadExecutor();
      Future<String> future = executor.submit(() -> {
        StringBuilder output = new StringBuilder();
        BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));

        String line;
        while ((line = reader.readLine()) != null) {
          log.debug(line);
          output.append(line + "\n");
        }

        int exitVal = process.waitFor();
        if (exitVal == 0) {
          return generateSolution(output.toString());
        } else {
          return null;
        }

      });

      // get results from planner thread
      executor.shutdown(); // does not interrupt current tasks, but allows awaitTermination to work
      try {
        // wait for planner task to complete
        if (executor.awaitTermination(15, TimeUnit.SECONDS)) {
          // planner task finished normally
          try {
            return future.get();
          } catch (InterruptedException | ExecutionException e) {
            log.error("Error getting resulting plan from planner thread.", e);
          }
        } else {
          // kill hanging planner task
          while (!executor.isTerminated()) {
            log.warn("Planner thread could not be stopped. Escalating...");
            Util.Sleep(2000);
            process.destroy();
          }
          return null;
        }
      } catch (InterruptedException e) {
        log.error("Error waiting for planner thread to terminate.", e);
        return null;
      }

    } catch (IOException e) {
      log.error("Exception while trying to plan.", e);
      return null;
    }
    return null;
  }

  private String generateSolution(String output) {
    String[] lines = output.split("\n");
    StringBuilder result = new StringBuilder();

    boolean stepsStarted = false;
    for (String line : lines) {
      if (line.startsWith("step")) {
        stepsStarted = true;
      }

      if (stepsStarted) {
        int colonIndex = line.lastIndexOf(":");
        if (colonIndex == -1) {
          break;
        } else {
          result.append("(");
          result.append(line.substring(colonIndex + 2).toLowerCase());
          result.append(")\n");
        }
      }
    }

    if (!stepsStarted) {
      return null;
    } else {
      return result.toString();
    }
  }

  public static void main(String[] args) {
    String domainFilename = args[0];
    String problemFilename = args[1];
    File domainFile = new File(domainFilename);
    File problemFile = new File(problemFilename);

    FFPlanner planner = new FFPlanner();
    planner.exec_path = System.getProperty("user.dir") + "/../thirdparty/Metric-FF-v2.1/ff";
    String plan = planner.plan(domainFile, problemFile);
    System.out.println("Generated Plan:\n\n" + plan);
  }

}
