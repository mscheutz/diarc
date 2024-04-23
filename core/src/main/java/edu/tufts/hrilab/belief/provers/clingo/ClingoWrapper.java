/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.provers.clingo;


import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import org.apache.commons.io.FileUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

public class ClingoWrapper {
  private String[] clingo;
  private File input = null;
  private Path error = null;
  private Path output = null;
  private static Logger log = LoggerFactory.getLogger(ClingoWrapper.class);

  /***since it writes to file this class is not thread safe**/
  public ClingoWrapper() {

    input = new File("clingo_input_temp.lp");
    error = new File("clingo_error_temp.lp").toPath();
    output = new File("clingo_output_temp.lp").toPath();

    this.clingo = new String[3];
    this.clingo[0] = "clingo";
    this.clingo[1] = input.getAbsolutePath();
    this.clingo[2] = "--verbose=0";
  }

  public boolean isSatisfiable(String theory) {
    try {
      log.debug("Theory:\n" + theory);
      log.debug("Calling clingo :" + this.clingo[1]);
      FileUtils.writeStringToFile(input, theory);

      long startTime = System.currentTimeMillis();
      Process clingo = new ProcessBuilder(this.clingo) //
              .redirectError(ProcessBuilder.Redirect.to(error.toFile()))
              .redirectOutput(ProcessBuilder.Redirect.to(output.toFile())).start();
      clingo.waitFor();

      long endTime = System.currentTimeMillis();
      long totalDuration = endTime - startTime;
      log.info("Clingo execution time (ms): " + totalDuration);

      String answers = FileUtils.readFileToString(output.toFile());
      log.debug("Answers:\n" + answers);
      if (answers.contains("SATISFIABLE") && !answers.contains("UNSATISFIABLE")) {
        return true;
      }

    } catch (IOException e) {
      log.error("Cannot launch clingo process.", e);
    } catch (InterruptedException e) {
      log.error("Clingo process was interrupted.", e);
    }

    return false;
  }


  public List<Map<Variable, Symbol>> parseModels(String theory, List<Variable> args, String pred) {
    List<Map<Variable, Symbol>> ret = new LinkedList<>();
    try {
      log.debug("Theory:\n" + theory);
      log.debug("Calling clingo :" + this.clingo[1]);
      FileUtils.writeStringToFile(input, theory);

      long startTime = System.currentTimeMillis();
      Process clingo = new ProcessBuilder(this.clingo) //
              .redirectError(ProcessBuilder.Redirect.to(error.toFile()))
              .redirectOutput(ProcessBuilder.Redirect.to(output.toFile())).start();
      clingo.waitFor();
      long endTime = System.currentTimeMillis();
      long totalDuration = endTime - startTime;
      log.debug("Clingo execution time (ms): " + totalDuration);

      String answers = FileUtils.readFileToString(output.toFile());
      log.debug("Clingo answer:\n" + answers);

      // iterate over all pred and extract the arguments
      int fromIndex = 0;
      log.debug("pred " + pred + ", args" + args);
      while (true) {
        log.debug("answer is " + answers);
        int index = answers.indexOf(pred + "(", fromIndex);
        log.debug(index + " " + pred + "( in " + answers);
        if (index == -1)
          break;
        int diff_parenthesis = 1;
        fromIndex = index + pred.length() + 1;
        int pos = fromIndex;
        int arg_pos = 0;
        Map<Variable, Symbol> belief = new HashMap<>();

        while (true) {
          if (answers.charAt(pos) == '(') {
            diff_parenthesis++;
          } else if (answers.charAt(pos) == ')') {
            diff_parenthesis--;
          } else if (answers.charAt(pos) == ',' && diff_parenthesis == 1) {
            log.debug("found an arg");

            //found an arg
            String name = answers.substring(fromIndex, pos);
            Symbol arg = new Symbol(name.trim());
            log.debug("name: " + arg + "," + name);
            if (arg_pos < args.size()) {
              belief.put(args.get(arg_pos), arg);
            } else {
              belief.clear();
              break;
            }
            fromIndex = pos + 1;
            arg_pos++;
          } else if (diff_parenthesis == 0) {
            // the last arg
            String name = answers.substring(fromIndex, pos - 1);
            Symbol arg = new Symbol(name.trim());
            if (arg_pos < args.size()) {
              belief.put(args.get(arg_pos), arg);
            } else {
              belief.clear();
              break;
            }

            break;
          }

          pos++;
        }
        fromIndex = pos;
        ret.add(belief);
      }

    } catch (IOException e) {
      log.error("Cannot launch clingo process.", e);

    } catch (InterruptedException e) {
      log.error("Clingo process was interrupted.", e);
    }

    return ret;
  }


}
